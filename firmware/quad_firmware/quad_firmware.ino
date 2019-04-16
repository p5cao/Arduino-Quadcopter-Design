#include "radio.h"
int M1pin = 3;
int M2pin = 4;
int M3pin = 5;
int M4pin = 8;


typedef struct{
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte btn1;
  byte btn2;
  byte btn_up;
  byte btn_down;
  byte btn_left;
  byte btn_right;
  byte btn_mid;
  byte armed;
}
rfsignal;

rfsignal b1;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(M1pin, OUTPUT);
  pinMode(M2pin, OUTPUT);
  pinMode(M3pin, OUTPUT);
  pinMode(M4pin, OUTPUT);
  Serial.begin(9600);
  Serial.println("speed 0 to 255");
  rfBegin(11);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)
  
  // Send a message to other RF boards on this channel
  rfPrint("ATmega128RFA1 Dev Board Online!\r\n");

}

void loop() {
  rfsignal b;
  int len;
  if (rfAvailable())
  {
    rfWrite(b1.throttle);
    rfWrite(b1.armed);
    rfWrite(b1.btn1);
    
  int speed = b.throttle;
  Serial.write(speed);
  int armed = b.armed;
  int blink = 1;
  if (blink){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second 
  }
  if (speed >= 0 && speed <= 255 && b.armed == 1)
  {
    analogWrite(M1pin, speed);
    analogWrite(M2pin, speed);
    analogWrite(M3pin, speed);
    analogWrite(M4pin, speed);
  }
  }
  rfPrint(b1.throttle);
  delay(100);
}
