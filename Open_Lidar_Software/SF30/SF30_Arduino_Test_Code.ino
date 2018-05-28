/*
  SF30 Serial Demo V1.0 
  Author: Zakia Ben Youss
  Copyright (c) 2015 Parallax Inc.
  See end of file for terms of use.


This demo uses the serial input on the SF30 High Speed Laser Rangefinder to initiate a new range value to be sent serially to the Arduino. 
Detects distance and displays it in meters on the Arduino Terminal.

NOTE:
-This demo assumes that the SF30 is set to a UART speed of 38400 Baud.
-This demo assumes that the SF30 is set to a 0.03m Snapshot Resolution. 
-This demo assumes that the SF30 is set to a 1144/sec Serial Port Update Rate.

See the product manual and instructions for details on changing the settings

You can find the manual by going to www.parallax.com and searching for SF30, and clicking Downloads & Documentation on the product page.

Wiring Diagram:

Pin_GND (Black)  On SF30 Laser Rangefinder - GND (Vss On Some Boards)
Pin_+5V (Red)    On SF30 Laser Rangefinder - +5V (Vdd On Some Boards)
Pin_TXD (Yellow) On SF30 Laser Rangefinder - Arduino RX Pin (10)
Pin_RXD (Orange) On SF30 Laser Rangefinder - Arduino TX Pin (11)

  
  http://learn.parallax.com/propeller-c-simple-devices/
  
*/

#include <SoftwareSerial.h>                                            

#define terminal_baud_rate    115200                                   // terminal baud rate 115200
#define serial_Rxd               10                                    // These pin definitions can be changed to match your wiring
#define serial_Txd               11
#define sf30_baud_rate         38400                                   // Sf30 baud rate  (Can be changed, refer to the sf30 manual)

SoftwareSerial sf30_serial(serial_Rxd, serial_Txd);                    // The pins used for the second serial port


float distance;                                                        // The Laser Range Finder Distance Variable
int Byte_L, Byte_H;

void setup()                                          
{
  Serial.begin(terminal_baud_rate);                                     // Open the main USB serial port on the Arduino ready for the terminal application
  while (!Serial);                                                      // Wait for serial port to connect.
  sf30_serial.begin(sf30_baud_rate);                                    // Open the second serial port to connect to the sf30
}

void loop()
{  
    sf30_serial.available();
    while (sf30_serial.available() > 0)                                 // Wait here for the next character
    {
    Byte_H = sf30_serial.read();                                        // Store the byte into Byte_H
    Byte_L = sf30_serial.read();                                        // Store the byte into Byte_L
    distance = (float(Byte_L))/256 + Byte_H;                            // Get distance from the bytes
    Serial.print("Distance in meters = ");                              // Print "Distance in meters = "
    Serial.println(distance, 4);                                        // Print values, then go to a new lines
    delay(10);                                                          // Pause .1 sec
    sf30_serial.available();                                            // Check buffer if byte available
    }
}
