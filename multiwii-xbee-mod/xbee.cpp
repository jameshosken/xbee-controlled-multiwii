#include "xbee.h"
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"

#include <SoftwareSerial.h> // added!!!! /////////////////////////////////////////////////////////
SoftwareSerial XBee(2, 6); // RX, TX

const int channels = 4;


unsigned char receivedArr[channels * 2];
uint16_t valueArray[channels];

int counter = 0;
bool counting = false;
bool datain = false;

void xbee_setup() {
  XBee.begin(9600);
}

unsigned int BitShiftCombine( unsigned char high_byte, unsigned char low_byte)
{

  unsigned int result = high_byte * 256 + low_byte;
  return result;

}

void get_xbee(int *pdata) {
  // ensure the serial packets are there, added the 0x00 check for the second byte (xbee was occasionally sending two 7E start bytes)
//  digitalWrite(LED_BUILTIN, LOW);

  counting = false;
  counter = 0;

  
  while (XBee.available()) {
    // If data comes in from XBee, send it out to serial monitor
    byte val = XBee.read();

    if ((char)val == '$') {
      counting = true;

    }

    if (counting) {
      if ((char)val == '#') {
        datain = true;
        break;
      }
      
      if (counter >= 1) {
        receivedArr[counter - 1] = (unsigned char)val;
      }
      counter++;
    }
  }
  
  if (datain) {
//    digitalWrite(LED_BUILTIN, HIGH);
    datain = false;
    for (int i = 0; i < channels * 2; i += 2) {
      valueArray[i / 2] = BitShiftCombine(receivedArr[i + 1], receivedArr[i]);
    }
    pdata[1] = valueArray[0];
    pdata[2] = valueArray[1];
    pdata[3] = valueArray[2];
    pdata[4] = valueArray[3];
  }
}



///////////////////
///////////////////
/*


  while (XBee.available())

  { // If data comes in from XBee, send it out to serial monitor
  byte val = XBee.read();

  if ((char)val == '<') {
    counting = true;

  }

  if (counting) {
    if (counter == 18) {
      datain = true;
    }
    if (counter > 2 && counter <= 18) {

      receivedArr[counter - 3] = (unsigned char)val;

    }

    counter++;
  }
  }
  //  Serial.println();

  if (datain) {
  digitalWrite(LED_BUILTIN, HIGH);
  datain = false;
  for (int i = 0; i < 16; i += 2) {
    valueArray[i / 2] = BitShiftCombine(receivedArr[i + 1], receivedArr[i]);
  }
  pdata[1] = valueArray[0];
  pdata[2] = valueArray[1];
  pdata[3] = valueArray[2];
  pdata[4] = valueArray[3];
  }
  else pdata[0] = 1;     //failsafe is initiated


  if (mySerial.available() > 21) {
  if (mySerial.read() == 0x7E) {
   if (mySerial.read() == 0x00) {
     for (int i = 0; i < 9; i++) {
       byte discardByte = mySerial.read();    // disregard the start byte, the second byte, and the 9 following bytes
     }

     int samp = 1;        //number of xbee samples
     int in = 4;          //number of xbee ADC inputs
     int MSB[in * samp];  //define array sizes
     int LSB[in * samp];
     int data[in * samp];
     int avedata[in];
     int mydata[in];
     int olddata[in];
     int failsafe_flag = 0;

     //collect xbee UART serial data
     for (int i = 0; i < in * samp; i++) {
       MSB[i] = mySerial.read();
       LSB[i] = mySerial.read();
     }
     for (int i = 0; i < in * samp; i++) {
       data[i] = LSB[i] + MSB[i] * 256;
     }

     //average the xbee samples and correlate to multiwii from 1000 to 2000
     for (int i = 0; i < in; i++) {
       avedata[i] = 0;                             // initialize average data array to zero
       for (int j = 0; j < samp; j++) {            // add the xbee samples
         avedata[i] = avedata[i] + data[i + in * j];
       }

       avedata[i] = avedata[i] / samp;             // average the xbee samples
       if (avedata[i] > 0) {
         // process data "IF" it doesnt appear to be corrupt, i.e. -2200
         if (i < 2 ) {
           if (avedata[i] > 820) {
             mydata[i] = (int)1000;
           }
           if (avedata[i] <= 820 && avedata[i] > 580) {
             mydata[i] = (int)(-2.083 * avedata[i] + 2708.333);
           }
           if (avedata[i] <= 580 && avedata[i] >= 440) {
             mydata[i] = 1500;
           }
           if (avedata[i] < 440 && avedata[i] >= 230) {
             mydata[i] = (int)(-2.381 * avedata[i] + 2547.619);
           }
           if (avedata[i] < 230) {
             mydata[i] = (int)2000;
           }
         }
         if (i >= 2 && i < 4) {
           if (avedata[i] > 820) {
             mydata[i] = (int)2000;
           }
         }
         if (avedata[i] <= 820 && avedata[i] > 580) {
           mydata[i] = (int)(2.083 * avedata[i] + 291.667);
         }
         if (avedata[i] <= 580 && avedata[i] >= 440) {
           mydata[i] = 1500;
         }
         if (avedata[i] < 440 && avedata[i] >= 245) {
           mydata[i] = (int)(2.381 * avedata[i] + 452.381);
         }
         if (avedata[i] < 245) {
           mydata[i] = (int)1000;
         }
       }
       olddata[i] = mydata[i];                  // store these values just incase the next reading is corrupt             failsafe_flag=0+failsafe_flag;           }           else if (avedata[i] == 0){             mydata[i] = 1500;             failsafe_flag = 1;           }         }                  // 0 = roll, 1 = yaw, 2 = pitch, 3 = throttle         //return array         if (failsafe_flag >0) {pdata[0] = 1;} else pdata[0] = 0;  //initate failsafe if data appears corrupt
       pdata[1] = mydata[0];
       pdata[2] = mydata[1];
       pdata[3] = mydata[2];
       pdata[4] = mydata[3];
     }
   }
   else pdata[0] = 1;     //failsafe is initiated
  }
  else pdata[0] = 1;     //failsafe is initiated
  }
  else pdata[0] = 1;     //failsafe is initiated
*/



/*
  if (XBee.available() > 22) {
    if ((char)XBee.read() == '$') {
      if ((char)XBee.read() == 'M') {
        if ((char)XBee.read() == '<') {
          if ((unsigned int)XBee.read() == 200) {
            if (XBee.read() == 0x00) {

              int samp = 1;        //number of xbee samples
              int in = 4;          //number of xbee ADC inputs
              int MSB[in * samp];  //define array sizes
              int LSB[in * samp];
              int data[in * samp];
              int avedata[in];
              int mydata[in];
              int olddata[in];
              int failsafe_flag = 0;

              //collect xbee UART serial data
              for (int i = 0; i < in * samp; i++) {
                MSB[i] = XBee.read();
                LSB[i] = XBee.read();
              }
              for (int i = 0; i < in * samp; i++) {
                data[i] = LSB[i] + MSB[i] * 256;
              }

              //average the xbee samples and correlate to multiwii from 1000 to 2000
              for (int i = 0; i < in; i++) {
                avedata[i] = 0;                             // initialize average data array to zero
                for (int j = 0; j < samp; j++) {            // add the xbee samples
                  avedata[i] = avedata[i] + data[i + in * j];
                }

                avedata[i] = avedata[i] / samp;             // average the xbee samples
                if (avedata[i] > 0) {
                  // process data "IF" it doesnt appear to be corrupt, i.e. -2200
                  if (i < 2 ) {
                    if (avedata[i] > 820) {
                      mydata[i] = (int)1000;
                    }
                    if (avedata[i] <= 820 && avedata[i] > 580) {
                      mydata[i] = (int)(-2.083 * avedata[i] + 2708.333);
                    }
                    if (avedata[i] <= 580 && avedata[i] >= 440) {
                      mydata[i] = 1500;
                    }
                    if (avedata[i] < 440 && avedata[i] >= 230) {
                      mydata[i] = (int)(-2.381 * avedata[i] + 2547.619);
                    }
                    if (avedata[i] < 230) {
                      mydata[i] = (int)2000;
                    }
                  }
                  if (i >= 2 && i < 4) {
                    if (avedata[i] > 820) {
                      mydata[i] = (int)2000;
                    }
                  }
                  if (avedata[i] <= 820 && avedata[i] > 580) {
                    mydata[i] = (int)(2.083 * avedata[i] + 291.667);
                  }
                  if (avedata[i] <= 580 && avedata[i] >= 440) {
                    mydata[i] = 1500;
                  }
                  if (avedata[i] < 440 && avedata[i] >= 245) {
                    mydata[i] = (int)(2.381 * avedata[i] + 452.381);
                  }
                  if (avedata[i] < 245) {
                    mydata[i] = (int)1000;
                  }
                }
                olddata[i] = mydata[i];                  // store these values just incase the next reading is corrupt             failsafe_flag=0+failsafe_flag;           }           else if (avedata[i] == 0){             mydata[i] = 1500;             failsafe_flag = 1;           }         }                  // 0 = roll, 1 = yaw, 2 = pitch, 3 = throttle         //return array         if (failsafe_flag >0) {pdata[0] = 1;} else pdata[0] = 0;  //initate failsafe if data appears corrupt
                pdata[1] = mydata[0];
                pdata[2] = mydata[1];
                pdata[3] = mydata[2];
                pdata[4] = mydata[3];



                { // If data comes in from XBee, send it out to serial monitor
                  byte val = XBee.read();

                  if ((char)val == '<') {
                    counting = true;

                  }

                  if (counting) {
                    if (counter == 18) {
                      datain = true;
                    }
                    if (counter > 2 && counter <= 18) {

                      receivedArr[counter - 3] = (unsigned char)val;

                    }

                    counter++;
                  }
                }
                //  Serial.println();

                if (datain) {
                  digitalWrite(LED_BUILTIN, HIGH);
                  datain = false;
                  for (int i = 0; i < 16; i += 2) {
                    valueArray[i / 2] = BitShiftCombine(receivedArr[i + 1], receivedArr[i]);
                  }
                  pdata[1] = valueArray[0];
                  pdata[2] = valueArray[1];
                  pdata[3] = valueArray[2];
                  pdata[4] = valueArray[3];
                }
              }
            }
          }
        }
      }
    }
    else pdata[0] = 1;     //failsafe is initiated
*/
