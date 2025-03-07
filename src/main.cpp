//
// Arduino main stub file. Calls into Rust.
//

#include <Arduino.h>

extern "C" void arduino_setup();
extern "C" void arduino_loop();

void setup()
{
	// Initialize serial communication
	Serial.begin(9600);
	// Call into Rust setup function
	arduino_setup();
}

void loop()
{
	// Call into Rust loop function
	arduino_loop();
}
