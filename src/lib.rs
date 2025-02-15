#![no_std]
#![feature(abi_avr_interrupt)]
#![cfg_attr(test, no_main)]
#![feature(custom_test_frameworks)]
#![test_runner(crate::test_runner)]

// Dummy test runner to prevent warnings
#[cfg(test)]
fn test_runner(_tests: &[&dyn Fn()]) {}

// Only include this panic handler when not testing
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
	loop {}
}

use core::panic::PanicInfo;

use avr_device::interrupt::Mutex;

// Arduino to 74HC595 Connections

/// Arduino D4 -> 74HC595 RCLK (Pin 12): Transfers shift register to storage register
const LATCH_DIO: u8 = 4;
/// Arduino D7 -> 74HC595 SRCLK (Pin 11): Shifts bits into register
const CLK_DIO: u8 = 7;
/// Arduino D8 -> 74HC595 SER (Pin 14): Serial data input
const DATA_DIO: u8 = 8;

// 7-Segment Display Layout:
//      A
//     ----
// F  |    | B
//    |  G |
//     ----
// E  |    | C
//    |    |
//     ----  • DP
//      D
//
// Segment bit pattern: A,B,C,D,E,F,G,DP (1 = OFF for Common Anode)
const DIGIT_PATTERNS: [u8; 10] = [
	0b11000000, // 0: Shows ABCDEF       (all segs except G)
	0b11111001, // 1: Shows BC           (right side segs)
	0b10100100, // 2: Shows ABDEG        (top, middle, bottom)
	0b10110000, // 3: Shows ABCDG        (all except F,E)
	0b10011001, // 4: Shows FGBC         (top-right side with middle)
	0b10010010, // 5: Shows AFGCD        (top-left side with middle)
	0b10000010, // 6: Shows AFGEDC       (all except B)
	0b11111000, // 7: Shows ABC          (top and right side)
	0b10000000, // 8: Shows ABCDEFG      (all segments)
	0b10010000, // 9: Shows ABCDFG       (all except E)
];

// Digit Selection (1 = ON for Common Anode)
// Each bit controls one digit through a transistor via the second 74HC595
const DIGIT_SELECT: [u8; 4] = [
	0b00000001, // Q0: Rightmost digit  (#1)
	0b00000010, // Q1: Second digit     (#2)
	0b00000100, // Q2: Third digit      (#3)
	0b00001000, // Q3: Leftmost digit   (#4)
];

/// Adds decimal point to a digit pattern by setting the DP bit to 0.
///
/// The DP (decimal point) is the MSB in our pattern, so we clear it
/// Input pattern: DP,G,F,E,D,C,B,A where DP is MSB
/// Returns the pattern with DP enabled (set to 0 for common anode)
#[allow(dead_code)]
fn with_decimal(pattern: u8) -> u8 {
	// Clear the MSB (DP segment) to turn it on (0 = ON for common anode)
	pattern & 0b01111111
}

/// Global counter instance protected by Mutex for safe access from interrupts.
/// - Uses static mut because we need a single mutable instance for the program.
/// - Wrapped in Mutex to ensure safe access from both main loop and interrupts
/// - Initialized with Counter::new() which must be a const fn.
static mut DISPLAY: Mutex<Display> = Mutex::new(Display::new());

/// Defines behavior for managing a 4-digit counter display
///
/// This trait handles:
/// - Value incrementing and rollover logic
/// - Timing-based updates using Arduino's millisecond counter
/// - Synchronization between display updates and time intervals
trait Counter {
	/// Increments a display value with rollover at 10000.
	///
	/// # Arguments
	/// * `value` - Current display value (0-9999)
	///
	/// # Returns
	/// * New value after increment, wrapped to 0 if exceeding 9999
	///
	/// This function ensures the display stays within 4 digits by using
	/// modulo arithmetic (value % 10000) for rollover handling
	fn increment(&mut self, value: u16) -> u16 {
		let value = (value + 1) % 10000;
		value
	}

	/// Updates counter value based on elapsed milliseconds.
	///
	/// # Arguments
	/// * `current_time` - Current system time in milliseconds from `arduino_millis()`
	/// * `last_update` - Timestamp of last counter update
	/// * `value` - Current display value (0-9999)
	///
	/// # Returns
	/// * Tuple of (new_value, new_last_update)
	///
	/// Updates occur every 1000ms (1 second) and include:
	/// 1. Checking time elapsed since last update
	/// 2. Incrementing counter if 1 second has passed
	/// 3. Updating timestamp for next interval
	fn update_counter(
		&mut self,
		current_time: u32,
		mut last_update: u32,
		mut value: u16,
	) -> (u16, u32) {
		if current_time - last_update >= 1000 {
			value = self.increment(value);
			last_update = current_time;
		}

		(value, last_update)
	}

	/// Displays the current value on the 4-digit display using multiplexing.
	///
	/// # Arguments
	/// * `value` - Current display value (0-9999)
	///
	/// # Hardware Operation
	/// Uses two 74HC595 shift registers in series:
	/// 1. First 74HC595: Controls the 7 segments (A-G) plus decimal point
	/// 2. Second 74HC595: Controls digit selection via transistors
	fn display_counter(&self, value: u16) {
		// Step 1. Calculate individual digits from value
		// Convert value into array of individual digits
		let digits = [
			(value / 1000) % 10, // Thousands place
			(value / 100) % 10,  // Hundreds place
			(value / 10) % 10,   // Tens place
			value % 10,          // Ones place
		];

		// Update display using multiplexing
		unsafe {
			for i in 0..4 {
				// Step 2. Look up segment pattern for current digit
				let pattern = DIGIT_PATTERNS[digits[i] as usize];
				// Step 3. Look up digit selection pattern
				let select = DIGIT_SELECT[i];

				// Step 4. Disable latch (RCLK low) to prepare for data
				arduino_digital_write(LATCH_DIO, 0);
				// Step 5. Shift segment pattern to first 74HC595
				shift_out(DATA_DIO, CLK_DIO, 1, pattern);
				// Step 6. Shift digit select to second 74HC595
				shift_out(DATA_DIO, CLK_DIO, 1, select);
				// Step 7. Enable latch (RCLK high) to update display outputs
				arduino_digital_write(LATCH_DIO, 1);
			}
		}
	}
}

/// Manages a 4-digit 7-segment display that shows a counter value.
///
/// The display uses two 74HC595 shift registers in series:
/// - First 74HC595 controls individual segments (A-G, DP)
/// - Second 74HC595 selects which digit is active
///
/// Display updates happen through multiplexing:
/// 1. Only one digit is lit at a time
/// 2. We cycle through all digits rapidly
/// 3. Persistence of vision makes it appear continuous
struct Display {
	/// Current counter value to display (0-9999)
	/// This value automatically wraps back to 0 after 9999
	value: u16,

	/// Timestamp of last counter increment in milliseconds
	/// Used to ensure consistent 1-second intervals between updates
	/// Retrieved from arduino_millis() which counts from power-on
	last_update: u32,
}

impl Display {
	/// Creates a new Display instance with initial values.
	///
	/// Must be const fn because it's used in static initialization
	/// of the global DISPLAY mutex. The display starts at zero
	/// and begins counting immediately after arduino_setup()
	const fn new() -> Self {
		Self {
			value: 0,
			last_update: 0,
		}
	}

	/// Main update function called in the Arduino loop.
	///
	/// The update_counter() and display_counter() implementations
	/// come from the Counter trait, which handles the timing logic
	/// and hardware-level display control
	fn tick(&mut self) {
		// Step 1. Get current time from Arduino's millisecond counter
		let current_time = unsafe { arduino_millis() };
		// Step 2. Update counter value if 1 second has elapsed
		let (value, last_update) =
			self.update_counter(current_time, self.last_update, self.value);

		self.value = value;
		self.last_update = last_update;

		// Step 3. Refresh the physical display with the new value
		self.display_counter(self.value)
	}
}

impl Counter for Display {}

/// Shifts out 8 bits of data to a shift register using bit-banging.
///
/// # Parameters
/// - data_pin: Pin connected to shift register's SER (serial data input)
/// - clock_pin: Pin connected to shift register's SRCLK (shift register clock)
/// - bit_order: 1 for MSBFIRST, 0 for LSBFIRST
/// - value: 8-bit value to shift out
fn shift_out(data_pin: u8, clock_pin: u8, bit_order: u8, mut value: u8) {
	unsafe {
		for _ in 0..8 {
			// Step 1: Extract next bit based on bit_order
			let bit = if bit_order == 1 {
				// MSB first: extract leftmost bit (0x80 = 0b10000000)
				let b = (value & 0x80) != 0;
				value <<= 1; // Shift left to get next bit ready
				b
			} else {
				// LSB first: extract rightmost bit (0x01 = 0b00000001)
				let b = (value & 1) != 0;
				value >>= 1; // Shift right to get next bit ready
				b
			};

			// Step 2: Set data pin to current bit value
			arduino_digital_write(data_pin, bit as u8);

			// Step 3: Pulse clock pin
			arduino_digital_write(clock_pin, 1); // Rising edge
			arduino_digital_write(clock_pin, 0); // Falling edge (data shifts in)
		}
	}
}

/// Arduino setup function - called once at startup.
///
/// Configures pin modes and initializes serial communication
#[no_mangle]
extern "C" fn arduino_setup() {
	unsafe {
		arduino_pin_mode(LATCH_DIO, 1); // Set LATCH pin as OUTPUT
		arduino_pin_mode(CLK_DIO, 1); // Set CLOCK pin as OUTPUT
		arduino_pin_mode(DATA_DIO, 1); // Set DATA pin as OUTPUT
		arduino_serial_begin(9600); // Initialize serial at 9600 baud
	}
}

/// Arduino main loop function - called repeatedly.
///
/// Updates and display value with interrupt safety
/// Uses critical section to safely access global COUNTER
#[no_mangle]
extern "C" fn arduino_loop() {
	// Create interrupt-free section to safely access global state
	avr_device::interrupt::free(|_| {
		#[allow(static_mut_refs)]
		unsafe {
			DISPLAY.get_mut().tick();
		}
	});
}

// External Arduino C functions we call from Rust
// These map directly to Arduino's C++ functions
extern "C" {
	/// Sets digital pin output value (HIGH=1, LOW=0)
	fn arduino_digital_write(pin: u8, value: u8);
	/// Configures pin mode (OUTPUT=1, INPUT=0)
	fn arduino_pin_mode(pin: u8, mode: u8);
	/// Returns milliseconds since program start
	fn arduino_millis() -> u32;
	/// Initializes serial communication at specified baud rate
	fn arduino_serial_begin(baud: u32);
	/// Prints value to serial monitor with newline
	#[allow(dead_code)]
	fn arduino_serial_println(value: u32);
}
