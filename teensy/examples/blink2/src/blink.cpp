// This example is taken and modified from the Teensyduino 'Blink' sketch
// See: https://www.pjrc.com/teensy/teensyduino.html
#include <WProgram.h>

static const int ledPin = 13;

#define SKETCH_ALIKE

#ifdef SKETCH_ALIKE

// the setup() method runs once, when the sketch starts
void setup()
{
    // initialize the digital pin for OUTPUT
    pinMode(ledPin, OUTPUT);
}

// the loop() method runs over and over again,
// as long as the board has power
void loop()
{
    digitalWrite(ledPin, HIGH); // set the LED on
    delay(500);                 // wait for half a second
    digitalWrite(ledPin, LOW);  // set the LED off
    delay(500);                 // wait for half a second
}

// This is needed as the starting point, as the standard main.cpp gets
// excluded during the build process.
extern "C" int main()
{
    setup();
    for (;;)
    {
        loop();
        yield();
    }
}

#else

extern "C" int main()
{
    // setup()
    pinMode(ledPin, OUTPUT);
    for (;;)
    {
        // loop()
        digitalWrite(ledPin, HIGH); // set the LED on
        delay(500);                 // wait for half a second
        digitalWrite(ledPin, LOW);  // set the LED off
        delay(500);                 // wait for half a second

        yield();                    // yield() is mandatory!
    }
}
#endif
