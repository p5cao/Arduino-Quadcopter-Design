/*
  ATmega128RFA1 Dev Board Basic Chat
  by: Jim Lindblom
      SparkFun Electronics
  date: July 3, 2012
  License: Beerware. Feel free to use, reuse, and modify this code
  as you please. If you find it useful, you can buy me a beer.

  This code sets up the ATmega128RFA1's wireless transciever in
  the most basic way possible to serve as a serial gateway.
  Serial into the ATmega128RFA1's UART0 will go out the RF radio.
  Data into the RF radio will go out the MCU's UART0.
*/

#include "radio.h"

void setup()
{
  Serial.begin(9600);  // Start up serial
  rfBegin(11);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)
  
  // Send a message to other RF boards on this channel
  rfPrint("ATmega128RFA1 Dev Board Online!\r\n");
}

void loop()
{

  if (rfAvailable())  // If serial comes in...
  {
    char a = rfRead();
    rfWrite(a); // ...send it out the radio.
    Serial.write(a);
  }
}
