//
// Arduino main stub file. Calls into Rust.
//

#include <Arduino.h>

extern "C" void arduino_setup();
extern "C" void arduino_loop();

void setup()
{
	arduino_setup();
}

void loop()
{
	arduino_loop();
}
