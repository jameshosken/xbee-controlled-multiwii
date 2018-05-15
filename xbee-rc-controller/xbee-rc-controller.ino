//
// An RC radio controller for MultiWii quadcopters.
// Carl Stubens, 2013
//
// v0.1
//
// Modified by James Hosken 2018

#include <Wire.h>

// Configuration:
#define READ_JOYSTICKS      1
#define SEND_MSP            1
#define PRINT_ANALOG        0
#define PRINT_BUTTONS       0
#define PRINT_RC_SIGNALS    0
#define REQUIRE_Z_BUTTONS   0 

// Communications
#define BAUDRATE      9600
#define BAUDRATE_XBEE 9600
#define POLL_PERIOD 20 // 20ms --> 50Hz
#define PIN_PPM 4


// The opcode (message type ID) and message length for 
// the MultiWii serial protocol's "SET_RAW_RC" message type.
#define MSP_SET_RAW_RC        200
#define MSP_SET_RAW_RC_LENGTH 16

// RC signal range to send to quad
#define RC_MIN 1000
#define RC_MID 1500
#define RC_MAX 2000

// The indices of the 8 signals in the following arrays
#define ROLL     0
#define PITCH    1
#define YAW      2
#define THROTTLE 3
#define AUX1     4
#define AUX2     5
#define AUX3     6
#define AUX4     7

// Calibration settings for each axis.
// Each axis has a minimum and maximum value.
int calib[4][2] = { 
                    {0,  1024}, // roll 
                    {0,  1024}, // pitch
                    {0,  1024}, // yaw
                    {1024, 0}  // throttle (INVERTED)
                  };
                    
// The indices of the minimum and maximum values in 
// the above calibration arrays.
#define MIN 0
#define MAX 1

// RC signals to send to the quad
// format: { roll, throttle, yaw, pitch, 0, 0, 0, 0 }
uint16_t rc_signals[4] = { 1234 }; 

// Buffer for storing the serializes byte form of the RC signals
uint8_t rc_bytes[8] = { 0 };

// We'll use SoftwareSerial to communicate with the XBee:
#include <SoftwareSerial.h>
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX


//init pins to read roll, pitch, yaw, throttle
int channels = 4;

int ypin = A3;
int tpin = A1;
int rpin = A2;
int ppin = A0;

void setup() {
  Serial.println();
  
  // Initialize serial connections
  Serial.begin(BAUDRATE);       // USB connection
  XBee.begin(BAUDRATE_XBEE);    // Xbee connection
  
  //Initialise analog read pins:
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
  Serial.println("Done.");
  delay(500);
}

void loop() {
  // Read data from nunchucks and compute RC signals
  if(READ_JOYSTICKS) {
        
    get_rc_signals(rc_signals);
  }
 
  // Send MultiWii Serial Protocol message
  if(SEND_MSP) {
    get_rc_bytes(rc_signals, rc_bytes);
    send_msp(MSP_SET_RAW_RC, rc_bytes, MSP_SET_RAW_RC_LENGTH); 
  }

  // Print debugging information, if configured to do so:

  if(PRINT_ANALOG) {
    print_analog();
  }



  if(PRINT_RC_SIGNALS) {
    for(int i = 0; i < channels; i++) {
      Serial.print(rc_signals[i], DEC);
      Serial.print(' ');
    }
  }

  if(PRINT_ANALOG | PRINT_BUTTONS | PRINT_RC_SIGNALS) {
    Serial.println();
    Serial.flush();
  }
  
  delay(POLL_PERIOD);
  
} // End main loop


void send_msp(uint8_t opcode, uint8_t * data, uint8_t n_bytes) {
  uint8_t checksum = 0;
   
  // Send the MSP header and message length
  XBee.write("$");
  Serial.write("$");
   
  // Send the data bytes
  for(int i = 0; i < channels*2; i++) {
    XBee.write(data[i]);
    
    Serial.print(data[i], HEX);
    Serial.print("\t");
  }
   
  // Send the endchar
  XBee.write('#');
  Serial.print('#');
  Serial.println();
}



// BELOW contains functions which deal with translating joystick 
// data into RC control signals wich are equivalent to the PWM RC 
// values expected by MultiWii.

// Apply a calibration tuple (min, max) to some input 
// value, and map the output onto (RC_MIN, RC_MAX).
int apply_calib(int x, int * calib) {
  x = map(x, calib[MIN], calib[MAX], RC_MIN, RC_MAX);
  x = constrain(x, RC_MIN, RC_MAX);
  return x;
}

// Set the RC signal array to the default values.
void zero_rc_signals(uint16_t * rc) {
  for(int i = 0; i < channels; i++) {
    rc[i] = RC_MID;
  }
  rc[THROTTLE] = RC_MIN;
}

// Compute RC signals from the latest nunchuk data.
void get_rc_signals(uint16_t * rc) {
  zero_rc_signals(rc);
  rc[ROLL]     = apply_calib(analogRead(rpin),     calib[ROLL]);
  rc[THROTTLE] = apply_calib(analogRead(tpin), calib[THROTTLE]);
  rc[YAW]      = apply_calib(analogRead(ypin),      calib[YAW]);
  rc[PITCH]    = apply_calib(analogRead(ppin),    calib[PITCH]);
}

// Set the RC bytes buffer to all zeros.
void zero_rc_bytes(uint8_t * buff) {
  for(int i = 0; i < channels*2; i++) {
    buff[i] = 0;  
  }  
}

// Compute the RC message data bytes based on the given RC signals.
void get_rc_bytes(uint16_t * rc, uint8_t * buff) {
  int j = 0;
  for(int i = 0; i < channels; i++) {
    buff[j++] = rc[i] & 0xFF;        // LSB first
    buff[j++] = (rc[i] >> 8) & 0xFF; // MSB second
  }  
  
}


//PRINT ANALOG//

// Print the latest analog joystick data from nunchuks
void print_analog() {
  Serial.print(analogRead(rpin));
  Serial.print(' ');
  Serial.print(analogRead(ypin));
  Serial.print(' ');
  Serial.print(analogRead(ppin));
  Serial.print(' ');
  Serial.print(analogRead(tpin));
  Serial.print(' '); 
}


