
#include <radio.h>

#include <quad_remote.h>  


// Header file with pin definitions and setup
int i = 0;
int calibrated = 0;
int btn1_flag = 0;
int btn2_flag = 0;
int yaw_left = 0;
int yaw_right = 0;
int throttle_top = 0;
int throttle_btm = 0;
int roll_left = 0;
int roll_right = 0;
int pitch_top = 0;
int pitch_btm = 0;

struct b1{
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
};





int Gymap ;
int Gtmap;
int Grmap;
int Gpmap;
int armed = 0;


//void knobs_update();
void knob_pressed(bool);
//Enter calibration mode by pressing button 1
void btn1_pressed(bool);
void btn2_pressed(bool);


void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  quad_remote_setup();

    // The buttons and the knob trigger these call backs.       
  //knobs_update_cb = knobs_update; 
  //knob1_btn_cb = knob_pressed;
  btn1_cb = btn1_pressed;
  btn2_cb = btn2_pressed;

  //knobs_update(); // Initialize the knob
}



void btn1_pressed(bool down){
  if(down){
    lcd.clear();
    lcd.print("Calibration Mode");
    lcd.setCursor(0, 1);
    lcd.print("Press BTN2...");
    btn1_flag = 1;
  }
}

void btn2_pressed(bool down){
  if (down && btn1_flag == 1){
    lcd.setCursor(0, 1);
    lcd.print("BTN2 is pressed");
    btn2_flag = 1;
    int yaw_left = 0;
    int yaw_right = 0;
    int throttle_top = 0;
    int throttle_btm = 0;
    int roll_left = 0;
    int roll_right = 0;
    int pitch_top = 0;
    int pitch_btm = 0;
  }
}

void Gimbal_Cali(){
    calibrated = 0;
    // Set Left Gimbal
    if (yaw_left == 0 && yaw_right == 0){
    lcd.setCursor(0, 1);
    lcd.print("push yaw to left");
    }
    if (Gymap >= 255){
      Gymap = map(Gymap,0,Gymap,0,255);
      yaw_left = 1;
      lcd.clear();
      lcd.print("Calibration Mode");
    }
    if (yaw_left == 1 && yaw_right == 0){
      
      
      lcd.setCursor(0, 1);
      lcd.print("yaw to right");
      if (Gymap <= 1){
        yaw_right = 1;
        Gymap = map(Gymap,Gymap,255,0,255);
      }
    }
    if (yaw_right == 1 && throttle_top == 0){
      lcd.setCursor(0, 1);
      lcd.print("thrt to top");
      if (Gtmap >= 255){
        Gtmap = map(Gtmap,0,Gtmap,0,255);
        throttle_top = 1;
      }
    }
    if (throttle_top == 1 && throttle_btm == 0){
      lcd.setCursor(0, 1);
      lcd.print("thrt to btm");
      if (Gtmap <= 1){
        Gtmap = map(Gtmap,Gtmap,255,0,255);
        throttle_btm = 1;
      }
    }

    //Set Right Gimbal
    if (throttle_btm==1 && pitch_top == 0){
      lcd.setCursor(0, 1);
      lcd.print("pitch to top");
      if (Gpmap <= 1){
        Gpmap = map(Gpmap,Gpmap,255,0,255);
        pitch_top = 1;
      }
    }
    if (pitch_top == 1 && pitch_btm == 0){
      lcd.setCursor(0, 1);
      lcd.print("pitch to btm");
      if (Gpmap >= 255){
        Gpmap = map(Gpmap,Gpmap,255,0,255);
        pitch_btm = 1;
      }
    }
    if(roll_left == 0 && pitch_btm == 1){
      lcd.setCursor(0, 1);
      lcd.print("roll to left");
      if (Grmap <= 1){
        Grmap = map(Grmap,Grmap,255,0,255);
        roll_left = 1;
      }
    }
     if(roll_left == 1 && roll_right ==0){
          lcd.setCursor(0, 1);
        lcd.print("roll to right");
      if (Grmap >= 255){
        Grmap = map(Grmap,0,Grmap,0,255);
        roll_right = 1;
     }
     }
if(roll_left == 1 && roll_right ==1){
lcd.clear();
btn2_flag = 0;
}
}



void loop() {
  // put your main code here, to run repeatedly:
    int Gyaw = analogRead(A0);
    int Gthrottle = analogRead(A1);
    int Groll = analogRead(A2);
    int Gpitch = analogRead(A3);
    //map Gimbal readings
    Gtmap = map(Gthrottle,68,515,0,255);
    constrain(Gtmap,0,255);
    Gymap = map(Gyaw,53,505,0,255);
    constrain(Gymap,0,255);
    Grmap = map(Groll,75,520,0,255);
    constrain(Grmap,0,255);
    Gpmap = map(Gpitch,67,511,0,255);
    constrain(Gpmap,0,255);
    //calibrate gimbals
    if (btn2_flag == 1 && armed == 0){
    Gimbal_Cali();
    }
//    char buffer [60];
//    i = sprintf(buffer,"throttle:%d yaw:%d roll:%d pitch:%d\n", Gtmap, Gymap, Grmap, Gpmap);
//    for (int I =0; I<= i; I++)
//    Serial.print(buffer[I]);
//    delay(100);  
      // delay in between reads for stability
    b1.throttle = Gtmap;
    b1.yaw = Gymap;
    b1.roll = Grmap;
    b1.pitch = Gpmap;
    b1.btn1 = 1;
    b1.btn2 = 0;
    b1.btn_up = 0;
    b1.btn_down = 0;
    b1.btn_left = 0;
    b1.btn_right =0;
    b1.btn_mid = 0;
    b1.armed = 0;
    if (rfAvailable()){
      rfRead(b1.throttle);
      rfWrite(b1.throttle);
      }

    }
