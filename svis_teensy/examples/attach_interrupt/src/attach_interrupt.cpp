/* Serial Monitor Example, Teensyduino Tutorial #3
   http://www.pjrc.com/teensy/tutorial3.html

   After uploading this to your board, use Serial Monitor
   to view the message.  When Serial is selected from the
   Tools > USB Type menu, the correct serial port must be
   selected from the Tools > Serial Port AFTER Teensy is
   running this code.  Teensy only becomes a serial device
   while this code is running!  For non-Serial types,
   the Serial port is emulated, so no port needs to be
   selected.

   This example code is in the public domain.
*/

#include "WProgram.h"

/* Interrupt Example code for two-buttons */

  /* no debouncing 
  // NOTE: I'm using hardware debounce IC.
  // Manufacturer Part Number: MAX6817EUT+T / Digi-Key Part Number: MAX6817EUT+TCT-ND /  Price 5.24$ US
  // MAX6817 ±15kV ESD-Protected, Dual , CMOS Switch Debouncer, also available MAX6816 Single / MAX6818 Octal
  // there are 63kΩ (typical) pullup resistors connected to each input */

unsigned long time;
unsigned long loop_num = 0;    // loop counter

// constants won't change. Used here to set pin numbers:
// Pin 13: Arduino has an LED connected on pin 13
// Pin 11: Teensy 2.0 has the LED on pin 11
// Pin  6: Teensy++ 2.0 has the LED on pin 6
// Pin 13: Teensy 3.x has the LED on pin 13
const int ledPin = 13;  // the number of the pin for activity-indicator

//Setup the Buttons
const int BUTTON1 = 7;  // for button 1 [ pin# 7 ]  button is pull up to 3.3v
const int BUTTON2 = 21;  // for button 2 [ pin# 21 ]    button is pull up to 3.3v

// Variables will change:
unsigned long previousMillis = 0;        // will store last time print loop counter was updated
unsigned long interval = 1000;           // interval at which to print loop counter (milliseconds)

volatile boolean flagB1;
volatile boolean flagB2;

// BUTTON 1 interrupt handler
void isrButton1 ()
{
  flagB1 = true;
}  // ******** end of isr Button 1 ********

// BUTTON 2 interrupt handler
void isrButton2 ()
{
  flagB2 = true;
}  // ******* end of isr Button 2  ********

void ResetBUTTONFlag()
{
  if (flagB1 == true)//reset BUTTON 1 flag + show led activity-indicator
  {
    // BUTTON 1 interrupt has occurred
    digitalWriteFast(ledPin, HIGH); //led on / activity-indicator
    delay(10); //delay to see the led blink
    digitalWriteFast(ledPin, LOW); //led off
    flagB1 = false; //reset flag
  }
  if (flagB2 == true)//reset BUTTON 2 flag + show led activity-indicator
  {
    // BUTTON 2 interrupt has occurred
    digitalWriteFast(ledPin, HIGH); //led on / activity-indicator
    delay(10); //delay to see the led blink
    digitalWriteFast(ledPin, LOW); //led off
    flagB2 = false; //reset flag
  }
}

extern "C" int main()
{
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT); // Set pin to OUTPUT for activity-indicator.

  //TEST LED on Power Up
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWriteFast(ledPin, LOW);
  delay(1000);

  // initialize pins for interrupts.
  // On Teensy3.x, the digital pins default to disable, not inputs.
  // You need to use pinMode to make the pin act as an input.
  // INPUT_PULLUP is a Teensy extension. On regular Arduino boards, digitalWrite the only way to access the pullup resistor.

  //pinMode(BUTTON1, INPUT); // button 1 with external pull-up resistor 10Kohms connected to 3.3V
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  /*On Teensy 3.x, interrupt 0 maps to pin 0, interrupt 1 maps to pin 1,
    interrupt 2 maps to pin 2, and so on.
    Every digital pin works with attachInterrupt.
  */
  // setup interrupts
  //RISING/HIGH/CHANGE/LOW/FALLING
  attachInterrupt (7, isrButton1, RISING);  // attach BUTTON 1 interrupt handler [ pin# 7 ]
  attachInterrupt (BUTTON2, isrButton2, RISING);  // attach BUTTON 2 interrupt handler
  
  while(true) {
    if (flagB1 == true)
      {
        Serial.println("BUTTON 1 interrupt has occurred");
        Serial.print("Time: ");
        time = millis();
        //prints time since program started
        Serial.println(time);
        ResetBUTTONFlag(); //If BUTTON interrupt has occurred, reset flag.
      }
    if (flagB2 == true)
      {
        Serial.println("BUTTON 2 interrupt has occurred");
        Serial.print("Time: ");
        time = millis();
        //prints time since program started
        Serial.println(time);
        ResetBUTTONFlag(); //If BUTTON interrupt has occurred, reset flag.
      }

    // Print without using the delay() function
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
      // save the last time you print loop counter
      previousMillis = currentMillis;

      Serial.print("loop counter: ");
      Serial.println(loop_num);
      loop_num++;
    }
    // put your main code here, to run repeatedly:

    yield();                    // yield() is mandatory!
  }
}
