#include <Arduino.h>

extern "C"
{
	void debug_println(uint8_t level, const char *msg)
	{
		const char *level_str;

		switch (level)
		{
		case 0:
			level_str = "ERROR";
			break;
		case 1:
			level_str = "WARN";
			break;
		case 2:
			level_str = "INFO";
			break;
		case 3:
			level_str = "DEBUG";
			break;
		case 4:
			level_str = "TRACE";
			break;
		default:
			level_str = "UNKNOWN";
			break;
		}

		Serial.print("[");
		Serial.print(level_str);
		Serial.print("] ");

		Serial.print(msg);
		Serial.println();
	}

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
}