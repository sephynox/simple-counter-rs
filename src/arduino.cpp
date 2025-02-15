#include <Arduino.h>

extern "C"
{
	void arduino_digital_write(uint8_t pin, uint8_t value)
	{
		digitalWrite(pin, value);
	}

	void arduino_pin_mode(uint8_t pin, uint8_t mode)
	{
		pinMode(pin, mode);
	}

	uint32_t arduino_millis()
	{
		return millis();
	}

	void arduino_serial_begin(uint32_t baud)
	{
		Serial.begin(baud);
	}

	extern "C" void arduino_serial_println(uint32_t value)
	{
		Serial.println(value);
	}
}