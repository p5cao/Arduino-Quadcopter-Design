// Remote control board 
// Note: Please maintain this code without further modification directly on it
// by: Yangsheng Hu and Pengcheng Cao; Date: 6:40pm, Apr. 17th, 2019

#include <quad_remote.h>
#include <radio.h>
#include <EEPROM.h>


void readGimbal(int serialDisp); // for testing
int readGimbal_calibrated(int Value,int minVal, int maxVal);

void update_display();
void btn1_pressed(bool);
void btn2_pressed(bool);

void btn_right_pressed(bool down);
void btn_center_pressed(bool down);

void arm_monitor(int gimbalYaw, int gimbalThrottle, int gimbalRoll, int gimbalPitch);// arm the board when conditions are satisfied

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 100;  //the value is a number of milliseconds

int disarm_mode = 1; // armed mode: arm = 0; disarmed mode (default): 1. Note: press BTN2 to disarm.
int calibrate_mode = 0; // uncalibrated mode (default): 0; calibrated mode: 1. Note: press BTN1 to change it
int counter_CalMode = 0; // Calbration procedures counter. 1,2:Yaw,3,4:Throttle,5,6:Roll,7,8:Pitch


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

  quad_remote_setup(); //Should be put in the front(especially of radio setting), or it's not working.

  // The buttons and the knob trigger these call backs.       
  btn1_cb = btn1_pressed;
  btn2_cb = btn2_pressed;
  btn_right_cb = btn_right_pressed;
  btn_center_cb =  btn_center_pressed;

  // *** set up the radio
  const int CHANNEL = 11; // radio channel number
  rfBegin(CHANNEL);// Initialize ATmega128RFA1 radio on channel (can be 11-26)
  // Send a message to other RF boards on this channel
  // rfPrint("ATmega128RFA1 Dev Board Online!\r\n");

  // *** set up the millis()
  startMillis = millis();  //initial start time

}



void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis(); // current time

  // read the input values from gimbals
//  int gimbalYaw = analogRead(A0);
//  int gimbalThrottle = analogRead(A1);
//  int gimbalRoll = analogRead(A2);
//  int gimbalPitch = analogRead(A3);

  int minVal, maxVal;
  EEPROM.get(1,minVal); EEPROM.get(3,maxVal);
  int gimbalYaw = readGimbal_calibrated(analogRead(A0), minVal, maxVal); //509~53

  EEPROM.get(5,minVal); EEPROM.get(7,maxVal);
  int gimbalThrottle = readGimbal_calibrated(analogRead(A1), minVal, maxVal); // 70~516
  
  EEPROM.get(9,minVal); EEPROM.get(11,maxVal);
  int gimbalRoll = readGimbal_calibrated(analogRead(A2), minVal, maxVal); // 76~524

  EEPROM.get(13,minVal); EEPROM.get(15,maxVal);
  int gimbalPitch = readGimbal_calibrated(analogRead(A3), minVal, maxVal); // 518~68

  arm_monitor(gimbalYaw, gimbalThrottle, gimbalRoll, gimbalPitch);
  
  // int serialDisp = 0;
  // readGimbal(serialDisp); // testing on the serial monitor

  // auxiliary display for calibration
  if (calibrate_mode & disarm_mode){
    if (counter_CalMode==1 || counter_CalMode==2){
            lcd.setCursor(11,1); lcd.print("     "); delay(200);
            lcd.setCursor(13,1); 
            lcd.print(gimbalYaw);          
    } else if (counter_CalMode==3 || counter_CalMode==4){
            lcd.setCursor(11,1); lcd.print("     "); delay(200);
            lcd.setCursor(13,1); 
            lcd.print(gimbalThrottle);     
    } else if (counter_CalMode==5 || counter_CalMode==6){
            lcd.setCursor(11,1); lcd.print("     "); delay(200);
            lcd.setCursor(13,1);  
            lcd.print(gimbalRoll);    
    } else if (counter_CalMode==7 || counter_CalMode==8){
            lcd.setCursor(11,1); lcd.print("     "); delay(200);
            lcd.setCursor(13,1); 
            lcd.print(gimbalPitch);   
    }
  }

  
  
  // Serial.println("Coming hehe");
  Quad_Info measurements;
  measurements.magic = 66;
  measurements.gimbalYaw = gimbalYaw;
  measurements.gimbalThrottle = gimbalThrottle;
  measurements.gimbalRoll = gimbalRoll;
  measurements.gimbalPitch = gimbalPitch;
  measurements.disarm_mode = disarm_mode;
  // Serial.println(gimbalYaw);
  
  uint8_t b[sizeof(measurements)]; // a buffer to store the radio data
  int len = sizeof(measurements); // the length (number of characters)
    
  // send radio every period milliseconds (so that not too much radio)
  if (currentMillis - startMillis >= period){
    // send the radio of measurements data
    
    // Serial.print(len);
    memcpy(b, &measurements, len); // transform struct data into bytes and store it into the buffer 'b'
    // b[len] = 0;
    //Serial.print(sizeof(b));
    //****
    rfWrite(b,len); // ... send it out the radio
    
    //Serial.write((char*)b); //write
    //Serial.println(); delay(100);
  
    startMillis = currentMillis;  //IMPORTANT: update the clock counter 
  } 

  // read the radio if there is something
  if (len = rfAvailable())
  {
    // Serial.print(len);

    //*** Efficient way:
    // input data should not be read as a whole before the magic number is checked
    rfRead(b, 2); // Our magic number only take up 2 characters in the small-endian order such as: magic = b[1],b[0] = 00 66
    int magic_receive = b[0]; // small-endian order for each quantity, i.e., since int magic has two characters, then the lower digit character goes in first.
    // Serial.println(magic_receive);
    if (magic_receive==66){
      rfRead(&b[2],len-2); // read the rest if the magic number is matching ours
      
      Quad_Info r_measurements;
      memcpy(&r_measurements,b,sizeof(r_measurements)); // reconstruct the orignal 'struct' type data
      // Serial.write(r_measurements.magic); // show the magic number in the serial port

      if (r_measurements.magic==66){    // if the magic number is matching ours, then it is our data and should be processed
        Serial.println((int)r_measurements.disarm_mode);
        disarm_mode = r_measurements.disarm_mode;
        if (disarm_mode==1){
          lcd.clear(); //lcd.home(); lcd.print("Disarmed Mode!");
        }
        delay(10);      
      }
    }

  //***************************************
//    //*** Another way: read the whole data first. If the magic number is not matching ours, then ignore it.
//    rfRead(b, len); // read the data into the buffer
//    Quad_Info r_measurements;
//    memcpy(&r_measurements,b,sizeof(r_measurements)); // reconstruct the orignal 'struct' type data
//    // Serial.write(r_measurements.magic); // show the magic number in the serial port
//
//    if (r_measurements.magic==66){    // if the magic number is matching ours, then it is our data and should be processed
//      Serial.println((int)r_measurements.gimbalYaw);
//      delay(10);      
//    }
  //***************************************
  
  }
  
  // The following part is only excuted when typing something in the textbox of Serial Monitor
  if (len = Serial.available()){
    for (int i = 0; i<len; i++){
      b[i] = Serial.read(); //***repeting
    }
    rfWrite(b,len);// ... send it out the radio
  }

}


int readGimbal_calibrated(int Value,int minVal, int maxVal){
  // Note: Value--raw measured data; minVal,maxVal--the possible min and max measurements
  Value = map(Value,minVal,maxVal,0,255); // map to range [0,255]
  Value = constrain(Value,0,255); // a saturation constraint
  return Value;
}


// the following function is used for testing on the serial monitor
void readGimbal(int serialDisp){ 
  // read the input values from gimbals
  int gimbalYaw = analogRead(A0); // analog pin 0
  int gimbalThrottle = analogRead(A1); // analog pin 1
  int gimbalRoll = analogRead(A2); // analog pin 2
  int gimbalPitch = analogRead(A3); // analog pin 3

  // map the input values to the range 0~255
  gimbalYaw = map(gimbalYaw,509,53,0,255);
  gimbalThrottle = map(gimbalThrottle,70,516,0,255);
  gimbalRoll = map(gimbalRoll,76,524,0,255);
  gimbalPitch = map(gimbalPitch,518,68,0,255);

  // constrain the calibrated values within 0~255
  gimbalYaw = constrain(gimbalYaw,0,255);
  gimbalThrottle = constrain(gimbalThrottle,0,255);
  gimbalRoll = constrain(gimbalRoll,0,255);
  gimbalPitch = constrain(gimbalPitch,0,255);

  if (serialDisp){
    Serial.print("Yaw: ");
    Serial.print(gimbalYaw); Serial.print('\t');
    
    Serial.print("Throttle: ");
    Serial.print(gimbalThrottle); Serial.print('\t');
    
    Serial.print("Roll: ");
    Serial.print(gimbalRoll); Serial.print('\t');
    
    Serial.print("Pitch: ");
    Serial.println(gimbalPitch); // Starting from another line after printing
    delay(1); // delay in between reads for stability    
  }
}


void update_display(){
    if (calibrate_mode & disarm_mode){
      counter_CalMode = counter_CalMode+1;
      switch (counter_CalMode) {
        case 1: // calibrate the Yaw
          lcd.setCursor(0, 0);
          lcd.print("Yaw2leftmost");
          lcd.setCursor(0,1);
          lcd.print("hit CENTER "); break;
        case 2: // calibrate the Yaw
          EEPROM.update(1,analogRead(A0));
          lcd.setCursor(0, 0);
          lcd.print("Yaw2rightmost");
          lcd.setCursor(0,1);
          lcd.print("hit CENTER "); break;
        case 3: // calibrate the Throttle
          EEPROM.update(3,analogRead(A0));
          lcd.setCursor(0, 0);
          lcd.print("Throttle2lowest");
          lcd.setCursor(0,1);
          lcd.print("hit CENTER "); break;  
        case 4: // calibrate the Throttle
          EEPROM.update(5,analogRead(A1));
          lcd.setCursor(0, 0);
          lcd.print("Throttle2highest");
          lcd.setCursor(0,1);
          lcd.print("hit CENTER "); break;         
        case 5: // calibrate the Roll
          EEPROM.update(7,analogRead(A1));
          lcd.setCursor(0, 0);
          lcd.print("Roll2leftmost");
          lcd.setCursor(0,1);
          lcd.print("hit CENTER "); break;
        case 6: // calibrate the Roll
          EEPROM.update(9,analogRead(A2));
          lcd.setCursor(0, 0);
          lcd.print("Roll2rightmost");
          lcd.setCursor(0,1);
          lcd.print("hit CENTER "); break;
        case 7: // calibrate the Pitch
          EEPROM.update(11,analogRead(A2));
          lcd.setCursor(0, 0);
          lcd.print("Pitch2lowest");
          lcd.setCursor(0,1);
          lcd.print("hit CENTER "); break;
        case 8: // calibrate the Pitch
          EEPROM.update(13,analogRead(A3));
          lcd.setCursor(0, 0);
          lcd.print("Pitch2highest");
          lcd.setCursor(0,1);
          lcd.print("hit CENTER ");
          break; 
        default: //reset
          EEPROM.update(15,analogRead(A3));
          counter_CalMode = 0; calibrate_mode = 0;//reset
      }      
    }
}

void btn1_pressed(bool down) {
  if(down) {
    // Serial.println("btn1 down");
    calibrate_mode = (calibrate_mode+1)%2;
    counter_CalMode = 0; // reset from the first mode to tune
    if (calibrate_mode & disarm_mode){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calibration ON");  
      lcd.setCursor(0,1); 
      lcd.print("Press btnCENTER");
    } 
  } else {
    // Serial.println("btn1 up");    
  }
}

void btn2_pressed(bool down) {
  if(down) {
    // Serial.println("btn2 down");
    disarm_mode = 1; // disarm both the remote and quad boards.
    lcd.clear(); // lcd.home(); lcd.print("Disarmed Mode!");
  }else {
    Serial.println("btn2 up");    
  }
}

void btn_right_pressed(bool down) {
  if(down) {
    Serial.println("right down");
    // update_display();
  } else {
    Serial.println("right up");    
  }
}

    
void btn_center_pressed(bool down) {
  if(down) {
    // Serial.println("center down");
    // update_display();
    lcd.clear(); delay(100);
    update_display();
  } else {
    Serial.println("center up");    
  }
}

void arm_monitor(int gimbalYaw, int gimbalThrottle, int gimbalRoll, int gimbalPitch){
  if (calibrate_mode==0){ // only able to arm when not calibrating
    if (gimbalYaw<=3 && gimbalThrottle<=3 && gimbalRoll>=252 && gimbalPitch<=3){
      disarm_mode = 0; // turn into armed mode
      lcd.clear(); lcd.home(); lcd.print("Armed Mode!");
    }  
  }
}
