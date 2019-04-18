// Quad board 
// Note: Please maintain this code without further modification directly on it
// by: Yangsheng Hu and Pengcheng Cao; Date: 6:40pm, Apr. 17th, 2019

#include "radio.h"

void driveMotors(int motorN,int dutyCycle);

int disarm_mode = 1; // armed mode: arm = 0; disarmed mode (default): 1. Note: press BTN2 to disarm. 
int dutyCycle = 0; // for PWM input of four motors. Lowest(0, default)

unsigned long startMillis; // time counter
 
// all the info data sent on radio
struct Quad_Info{
  int magic; // symbol
  int gimbalYaw;
  int gimbalThrottle;
  int gimbalRoll;
  int gimbalPitch;
  int disarm_mode;
};

void setup() {
  // put your setup code here, to run once:
  const int SERIAL_BAUD = 9600; // Baud rate for serial port
  Serial.begin(SERIAL_BAUD); // Start up serial
  delay(100);

  // set up the motor pins
  pinMode(8,OUTPUT); // pin8 for motor1
  pinMode(3,OUTPUT); // pin3 for motor2
  pinMode(4,OUTPUT); // pin4 for motor3
  pinMode(5,OUTPUT); // pin5 for motor4

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  // *** set up the radio
  const int CHANNEL = 11; // radio channel number
  rfBegin(CHANNEL);// Initialize ATmega128RFA1 radio on channel (can be 11-26)

  // Send a message to other RF boards on this channel
  rfPrint("ATmega128RFA1 Dev Board Online!\r\n");

}

void loop() {
  // put your main code here, to run repeatedly:

// Serial.println(startMillis);
  // *** set up the millis()
  startMillis = millis();  //initial start time
  if (startMillis<1000){
    // Send a disarm_mode message to remote board when reset
    Quad_Info quad_reset;
    uint8_t b[sizeof(quad_reset)]; // a buffer to store the radio data
    int len = sizeof(quad_reset); // the length (number of characters)
    quad_reset.magic = 66; // magic number for communication matching
    quad_reset.disarm_mode = 1;
    memcpy(b, &quad_reset, len); // transform struct data into bytes and store it into the buffer 'b'
    rfWrite(b,len); // ... send it out the radio  
    delay(10);
  }
  
  
  
  Quad_Info from_remote; // used to receive data from the remote board
  uint8_t b[sizeof(from_remote)]; // a buffer to store the radio data
  int len = sizeof(from_remote); // the length (number of characters)

  if (rfAvailable()) // If serial comes in... 
  {
    //*** Efficient way:
    // input data should not be read as a whole before the magic number is checked
    rfRead(b, 2); // Our magic number only take up 2 characters in the small-endian order such as: magic = b[1],b[0] = 00 66
    int magic_receive = b[0]; // small-endian order for each quantity, i.e., since int magic has two characters, then the lower digit character goes in first.
    // Serial.println(magic_receive);
    if (magic_receive==66){
      rfRead(&b[2],len-2); // read the rest if the magic number is matching ours
      
      // Quad_Info from_remote;
      memcpy(&from_remote,b,sizeof(from_remote)); // reconstruct the orignal 'struct' type data
      // Serial.write(from_remote.magic); // show the magic number in the serial port

      if (from_remote.magic==66){    // if the magic number is matching ours, then it is our data and should be processed
        // Serial.println((int)from_remote.disarm_mode); delay(10);
        disarm_mode = from_remote.disarm_mode;
        dutyCycle = from_remote.gimbalThrottle;     
      }
    }
    
//    //******echo for testing
//    char a = rfRead();
//    rfWrite(a); // ...send it out the radio.
//    Serial.write(a);
//    //***************

  }
  
  if (disarm_mode==0){ //arm mode
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    
    // drive the motorN, where N=1,2,3,4 and dutyCycle [0,255]
    driveMotors(1,dutyCycle);
    driveMotors(2,dutyCycle);
    driveMotors(3,dutyCycle);
    driveMotors(4,dutyCycle);
    // Serial.println("Once more");    
  } else { // disarm mode
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    dutyCycle = 0;
    driveMotors(1,dutyCycle);
    driveMotors(2,dutyCycle);
    driveMotors(3,dutyCycle);
    driveMotors(4,dutyCycle);
  }
  

}

void driveMotors(int motorN,int dutyCycle){
  // decide which pin to drive
  if (dutyCycle<0 || dutyCycle>255){
    Serial.println("Oops ... duty cycle should be within [0,255].");
    while(1);
  }
  switch (motorN){
    case 1: // motor 1 will be drived
      analogWrite(8,dutyCycle); break;
    case 2: // motor 2 will be drived
      analogWrite(3,dutyCycle); break;
    case 3: // motor 3 will be drived
      analogWrite(4,dutyCycle); break;
    case 4: // motor 4 will be drived
      analogWrite(5,dutyCycle); break;
    default: // print error messages
      Serial.println("Oops ... only motor 1,2,3,4 are available");
      while(1);
  }
}
