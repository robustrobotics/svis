#include <WProgram.h>

static const int ledPin = 13;

extern "C" int main()
{
  pinMode(ledPin, OUTPUT);

  while(true) {
    digitalWrite(ledPin, HIGH); // set the LED on
    delay(200);                 // wait for half a second
    digitalWrite(ledPin, LOW);  // set the LED off
    delay(2000);                 // wait for half a second

    yield();                    // yield() is mandatory!
  }
}
