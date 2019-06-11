// Quad board 
// Note: Based on the version 4 from "Date: 15:13pm, May. 13th, 2019". Now, PID for all directions are added.
// by: Yangsheng Hu and Pengcheng Cao

// Remark: 1. Although Baud rate 115200 (the highest the best in principle) is OK for Example code "ahrs_quad.ino", we 
//         could only work with up to 19200 when we incorporate IMU measurements into the total code.
//         2. Make sure everytime you are using 'sec'. Note that the unit for millis() is 'msec'. So remember to divide it by 1000.
//         3. (int) 20ms/1000 = 0 sec. So you should use float type for the time keeper.
//         4. Be careful about the radio channel and magic number. My home WIFI disturb my radio with magic 65 and channel 11.
//         5. One motor device (NO.1) seems not working in the same way as others and has been replaced
#include "radio.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Simple_AHRS.h>

// Create LSM9DS0 board instance.
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag(), &lsm.getGyro());

#define FDS_MODE B00000100 // For setting register CTRL_REG7_XL: data sent to output register and FIFO; 
                          // No need for FDS_MODE here since Table.70 in the datasheet is not right. Fig.8 is right.

void setupSensor()
{
  // Instruction: high pass filter with GYPO (since biased) and low pass filter with Accelerometer (since noisy).
  // Set data rate for G and XL.  Set G low-pass cut off.  (Section 7.12)
  // Note: ODR_50 is not accurately 50Hz. The datasheet gives 59.5Hz. 
  //       However, 238hz ODR is used (o.k. as long as higher than 1/(now-last)sec). Higher one will make settings like ODR ratio more flexible.
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG1_G,  ODR_238 | G_BW_G_01 );  //238hz ODR + 76 Hz LPF1 cutoff. 29hz LPF2 cutoff if enabled.

  // Set low-pass and high-pass G filter frequency divider (Section 7.13-7.14)
  // --Approach 1: On Fig.28, the signal goes through HPF without LPF2
//  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG2_G, G_OUTSEL_HP); // path through HPF without LPF2
//  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_G, G_HP_CUT_0010); // enable HPF + 4 Hz cutoff (for ODR=238hz). No need to set G_HP_EN since LPF2 is not used.
    // --Approach 2: On Fig.28, the signal goes through HPF and then LPF2(filter high freq noise caused by HPF)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG2_G, G_OUTSEL_HP_LP); // path through HPF and LPF2
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_G, G_HP_EN | G_HP_CUT_0101); // enable HPF + 0.5 Hz cutoff (for ODR=238hz). Followed by LPF2 with 29Hz cutoff.
 
 
  // Enable the XL (Section 7.23)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG5_XL, XL_ENABLE_X | XL_ENABLE_Y | XL_ENABLE_Z);

  // Set low-pass XL filter frequency divider (Section 7.25)
  // Note: FDS_MODE not needed since the datasheet there is wrong. Fig.8 and Table.70(wrong) contradicts.
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL, HR_MODE | XL_LP_ODR_RATIO_9); // data sent to output register and FIFO. XL LPF2 cutoff freq = ODR/9
  
  // enable mag continuous (Section 8.7)
  lsm.write8(MAGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_M, B00000000); // continuous mode

  // This only sets range of measurable values for each sensor.  Setting these manually (I.e., without using these functions) will cause incorrect output from the library.
  lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(Adafruit_LSM9DS1::LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_500DPS); // could be used to change the range of gyro values (set the max change rate)
}



void driveMotors(int motorN,int dutyCycle);
float complementaryFilter(float angle_m, float velocity_m, float last_angle, float delta_t, float gain);

int disarm_mode = 1; // armed mode: arm = 0; disarmed mode (default): 1. Note: press BTN2 to disarm. 
int dutyCycle = 0; // for PWM input of four motors. Lowest(0, default)

unsigned long startMillis; // time counter
unsigned long last_radio = millis(); // sending radio not that fast, e.g. every period millisecs
const unsigned long period = 100;  //the value is a number of milliseconds
float last = millis(); // the last time mark point for the IMU sampling
                // Note: we should use float type if we are using 'sec' unit, since (int) 20 msec/1000 = 0 sec


float roll_rate_regression[3];
float filtered_pitch = 0; // using complementary filter
float filtered_pitch_last = 0;
float filtered_pitch_sum = 0; // for integral of PID
float filtered_pitch_df = 0;  // for derivative of PID
float filtered_pitch_df_1 = 0; // the value from last time instant (for filtering design)
float ref_pitch = 0; // reference signal from the remote gimbal
float err_pitch = 0; // tracking error
float err_pitch_last = 0;
float alpha=0; // Integration saturation. alpha=0 no integration; alpha=1 with integration.
float beta=0; // Changing the integration rate. 0: no integration; 1: the fastest.


float filtered_roll = 0; // using complementary filter
float filtered_roll_last = 0;
float filtered_roll_sum = 0; // for integral of PID
float filtered_roll_df = 0;
float filtered_roll_df_1 = 0;
float ref_roll = 0; // reference signal from the remote gimbal
float err_roll = 0; // tracking error
float err_roll_last = 0;
float alpha_roll = 0;
float beta_roll = 0;


float filtered_yaw = 0; // using complementary filter
float filtered_yaw_last = 0;
float filtered_yaw_sum = 0; // for integral of PID
float ref_yaw = 0; // reference signal from the remote gimbal
float err_yaw = 0; // tracking error
float err_yaw_last = 0;
float alpha_yaw = 0;
float gamma_yaw = 0; // scaling for Kp


float PID_output; // The output of PID controller for pitch
float PID_output_1=0; // The PID output from last time instant (for 1st order LP filter design)

float PID_output_roll;
float PID_output_yaw;

// all the info data sent on radio
struct Quad_Info{
  int magic; // symbol
  int gimbalYaw;
  int gimbalThrottle;
  int gimbalRoll;
  int gimbalPitch;
  int disarm_mode;
  float PID_Kp; //PID tuning parameters
  float PID_Ki;
  float PID_Kd;
  float filtered_pitch;
  float PID_output;
//  float PID_P;
//  float PID_D;
//  float PID_I;
  // quad_data_t orientation; // no need to send IMU data in fact
};

Quad_Info from_remote; // used to receive data from the remote board

void setup() {
  // put your setup code here, to run once:
  const int SERIAL_BAUD = 19200; // Baud rate for serial port
  Serial.begin(SERIAL_BAUD); // Start up serial
  delay(100);

  // set up the motor pins
  pinMode(8,OUTPUT); // pin8 for motor1
  pinMode(3,OUTPUT); // pin3 for motor2
  pinMode(4,OUTPUT); // pin4 for motor3
  pinMode(5,OUTPUT); // pin5 for motor4
  // set up the LED pins
  pinMode(9,OUTPUT); // pin9 for a group of LED
  pinMode(34,OUTPUT); // pin34 for a group of LED

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialise the LSM9DS0 board.
  //if(!lsm.begin()) //*****previous one
  while(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS1 detected ... Check your wiring or I2C ADDR!"));
    //while(1);
  }
  
  // Setup the sensor gain and integration time.
  setupSensor();

  
  // *** set up the radio
  const int CHANNEL = 11; // radio channel number
  rfBegin(CHANNEL);// Initialize ATmega128RFA1 radio on channel (can be 11-26)

  // Send a message to other RF boards on this channel
  rfPrint("ATmega128RFA1 Dev Board Online!\r\n");

  // *** set up the initial PID parameters
  from_remote.PID_Kp = 0.55;
  from_remote.PID_Kd = 0.2;
  from_remote.PID_Ki = 0.002;
}



void loop() {
  // put your main code here, to run repeatedly:

  Quad_Info FCB_info; // the FCB measurements (may Including IMU) sending back to the remote
  
// Serial.println(startMillis);
   // *** set up the millis()
  startMillis = millis();  //initial start time
  if (startMillis<1000){
    // Send a disarm_mode message to remote board when reset
    Quad_Info quad_reset;
    uint8_t b[sizeof(quad_reset)]; // a buffer to store the radio data
    int len = sizeof(quad_reset); // the length (number of characters)
    quad_reset.magic = 67; // magic number for communication matching
    quad_reset.disarm_mode = 1;
    FCB_info.disarm_mode = 1; // New added for updating FCB_info
    memcpy(b, &quad_reset, len); // transform struct data into bytes and store it into the buffer 'b'
    rfWrite(b,len); // ... send it out the radio  
    delay(10);
  } 

  //*******************IMU measurements
  float now = millis(); // Note: we should use float type if we are using 'sec' unit, since (int) 20 msec/1000 = 0 sec
  quad_data_t orientation_m;
  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getQuadOrientation(&orientation_m))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
     Serial.print(now - last);
    // Serial.print(F(" "));
    // Serial.print(orientation_m.roll+0.4);
    // Serial.print(orientation_m.roll+3.4); //Serial.print(orientation_m.roll+1.65+0.7);
    // Serial.print(F(" "));
    // Serial.print(orientation_m.pitch-1.75+0.3); // Calibration for education board
    // Serial.print(orientation_m.pitch-2.75-1.4-0.1); // Calibration for our self-designed board
    // Serial.print(F(" "));
    // Serial.print(-orientation_m.roll_rate);

//    int i= (roll_rate_regression[0]+roll_rate_regression[1]+roll_rate_regression[2])/3;
//    Serial.print(i);
//    roll_rate_regression[2] = roll_rate_regression[1]; 
//    roll_rate_regression[1] = roll_rate_regression[0];
//    roll_rate_regression[0] = orientation_m.roll_rate;
    
    // Serial.print(F(" "));
    // Serial.print(-orientation_m.pitch_rate);
    // Serial.print(F(" "));

    // filtered_pitch = complementaryFilter(orientation_m.pitch-1.75+0.3, -orientation_m.pitch_rate, filtered_pitch_last, (now-last)/1000, 0.98);
    filtered_pitch = complementaryFilter(orientation_m.pitch-2.75-1.4-0.1, -orientation_m.pitch_rate, filtered_pitch_last, (now-last)/1000, 0.98);
    // Serial.print(filtered_pitch);
    // Serial.print(F(" "));
    ref_pitch = map(from_remote.gimbalPitch,0,255,-10,10); // map gimbal numbers into degrees
    if(abs(ref_pitch)<3){ //dead zone
      ref_pitch = 0;
    }
    err_pitch = filtered_pitch - ref_pitch;
    if(abs(err_pitch)<1){ //dead zone
      err_pitch = 0;
    }

    // Serial.print(ref_pitch); Serial.print(F(" ")); 
    
    
    // filtered_roll = complementaryFilter(orientation_m.roll+0.4, -orientation_m.roll_rate, filtered_roll_last, (now - last)/1000, 0.98);
    filtered_roll = complementaryFilter(orientation_m.roll+3.4, -orientation_m.roll_rate, filtered_roll_last, (now - last)/1000, 0.98);
    // Serial.print(filtered_roll);
    // Serial.print(F(" ")); 
    ref_roll = map(from_remote.gimbalRoll,0,255,-10,10); // map gimbal numbers into degrees
    if(abs(ref_roll)<3){ //dead zone
      ref_roll = 0;
    }
    err_roll = filtered_roll - ref_roll;
    if(abs(err_roll)<1){ //dead zone
      err_roll = 0;
    } 
    
    
    //Serial.print(orientation_m.yaw_rate);
    filtered_yaw = orientation_m.yaw_rate; // No filtering needed
    ref_yaw = map(from_remote.gimbalYaw,0,255,-10,10);
    if(abs(ref_yaw)<3){ //dead zone
      ref_yaw = 0;
    }    
    err_yaw = filtered_yaw - ref_yaw;
    if(abs(err_yaw)<1){ //dead zone
      err_yaw = 0;
    }

    // **********************Pitch control
    if(disarm_mode==1){
      filtered_pitch_sum = 0;
    }
    filtered_pitch_sum = filtered_pitch_sum + alpha*beta*err_pitch*(now-last)/1000;
    //*****Option 1: filtered derivative(reduce the noise)
    filtered_pitch_df = (err_pitch-err_pitch_last)*1000/(now-last);
    //filtered_pitch_df = 0.09597*filtered_pitch_df_1+0.904*filtered_pitch_df; filtered_pitch_df_1 = filtered_pitch_df;
    filtered_pitch_df = 0.1*filtered_pitch_df_1+0.9*filtered_pitch_df; filtered_pitch_df_1 = filtered_pitch_df;
    //PID_output = 0.78*err_pitch+0.26*filtered_pitch_df+0.08*filtered_pitch_sum;
//    if(dutyCycle<120){
//      PID_output = 0.74*err_pitch+0.26*filtered_pitch_df+0.1*filtered_pitch_sum;
//    }else{
//      PID_output = 1.0*err_pitch+0.3*filtered_pitch_df+0.1*filtered_pitch_sum;
//    }
    //PID_output = 0.86*err_pitch+0.28*filtered_pitch_df+0.1*filtered_pitch_sum;
    PID_output = from_remote.PID_Kp*err_pitch+from_remote.PID_Kd*filtered_pitch_df+from_remote.PID_Ki*filtered_pitch_sum;
    //*****
    //*****Option 2: normal derivative
    //PID_output = 0.78*err_pitch+0.16*(err_pitch-err_pitch_last)*1000/(now-last)+0.01*filtered_pitch_sum;
    //PID_output = from_remote.PID_Kp*err_pitch+from_remote.PID_Kd*(err_pitch-err_pitch_last)*1000/(now-last)+from_remote.PID_Ki*filtered_pitch_sum;
    //PID_output = 0.04394*PID_output_1+0.9561*PID_output; PID_output_1 = PID_output; // filter the PID_output
    //*****

    //***** Handling integration saturation
    if(dutyCycle==0 ){// Note: condition during tuning--> || from_remote.PID_Ki==0
      filtered_pitch_sum = 0;
    }else if(PID_output<-25){// only integrate the positive err
      if(err_pitch>0){
        alpha = 1; 
      }else {
        alpha = 0;
      }
    }else if(PID_output>25){// only integrate the negative err
      if(err_pitch>0){
        alpha = 0;  
      }else {
        alpha = 1;
      }
    }else{
      alpha = 1;
    }
    //alpha=1;
    //*****

    //***** Changing the integration speed
//    if(abs(err_pitch)<=10){// fastest rate
//      beta = 1.5;  
//    }else if(abs(err_pitch)>10 && abs(err_pitch)<=20){ //normal rate
//      beta = 1;
//    }else if(abs(err_pitch)>20 && abs(err_pitch)<20+60){
//      beta = (60-abs(err_pitch)+20)/60;
//    }else{ // no integration
//      beta = 0; 
//    }
    beta=1;
    //*****
    
    // PID_output = constrain(PID_output,-50,50); // constraint of the control effort
    // PID_output = 0; // no PID input
    filtered_pitch_last = filtered_pitch;
    err_pitch_last = err_pitch; 
    // Serial.print(PID_output);
     Serial.println(F(" "));

    // **********************Roll control
    if(disarm_mode==1){
      filtered_roll_sum = 0;
    }
    filtered_roll_sum = filtered_roll_sum + alpha_roll*beta_roll*err_roll*(now-last)/1000;
    //*****Option 1: filtered derivative(reduce the noise)
    filtered_roll_df = (err_roll-err_roll_last)*1000/(now-last);
    filtered_roll_df = 0.1*filtered_roll_df_1+0.9*filtered_roll_df; filtered_roll_df_1 = filtered_roll_df;
    PID_output_roll = from_remote.PID_Kp*err_roll+from_remote.PID_Kd*filtered_roll_df+from_remote.PID_Ki*filtered_roll_sum;
    //PID_output_roll = 0.78*err_roll+0.26*filtered_roll_df+0.02*filtered_roll_sum;

    //*****
    //*****Option 2: normal derivative
    //PID_output_roll = 0.78*err_roll+0.26*(err_roll-err_roll_last)*1000/(now-last)+0.05*filtered_roll_sum;  
    //PID_output_roll = from_remote.PID_Kp*err_roll+from_remote.PID_Kd*(err_roll-err_roll_last)*1000/(now-last)+from_remote.PID_Ki*filtered_roll_sum;  
    //*****

    //***** Handling integration saturation
    if(dutyCycle==0){// Note: condition during tuning--> || from_remote.PID_Ki==0
      filtered_roll_sum = 0;
    }else if(PID_output_roll<-25){// only integrate the positive err
      if(err_roll>0){
        alpha_roll = 1; 
      }else {
        alpha_roll = 0;
      }
    }else if(PID_output_roll>25){// only integrate the negative err
      if(err_roll>0){
        alpha_roll = 0;  
      }else {
        alpha_roll = 1;
      }
    }else{
      alpha_roll = 1;
    }
    //alpha_roll=1;
    //*****

    //***** Changing the integration speed
//    if(abs(err_roll)<=10){// fastest rate
//      beta_roll = 1.5;  
//    }else if(abs(err_roll)>10 && abs(err_roll)<=20){ //normal rate
//      beta_roll = 1;
//    }else if(abs(err_roll)>20 && abs(err_roll)<20+60){
//      beta_roll = (60-abs(err_roll)+20)/60;
//    }else{ // no integration
//      beta_roll = 0; 
//    }
    beta_roll=1;
    //*****
    
    filtered_roll_last = filtered_roll; 
    err_roll_last = err_roll;  
    //PID_output_roll = 0;
    //Serial.print(PID_output_roll); Serial.print(F(" "));
    //Serial.print(filtered_roll); Serial.println(F(" "));

    // *********************Yaw control
    if(disarm_mode==1){
      filtered_yaw_sum = 0;
    }
//    if(abs(err_yaw)<70){
//      gamma_yaw = 0.3;
//    }else if(abs(err_yaw)>=70 && abs(err_yaw)<120){
//      gamma_yaw = 0.6;
//    }else{
//      gamma_yaw = 1;
//    }
    gamma_yaw = 1;
    filtered_yaw_sum = filtered_yaw_sum + alpha_yaw*err_yaw*(now-last)/1000;
    PID_output_yaw = 1.5*err_yaw+0*(err_yaw-err_yaw_last)*1000/(now-last)+0.8*filtered_yaw_sum; 
    // PID_output_yaw = gamma_yaw*from_remote.PID_Kp*err_yaw+from_remote.PID_Kd*(err_yaw-err_yaw_last)*1000/(now-last)+from_remote.PID_Ki*filtered_yaw_sum;  
    
    //***** Handling integration saturation
    if(dutyCycle==0){// Note: condition during tuning--> || from_remote.PID_Ki==0
      filtered_yaw_sum = 0;
    }else if(PID_output_yaw<-5){// only integrate the positive err
      if(err_yaw>0){
        alpha_yaw = 1; 
      }else {
        alpha_yaw = 0;
      }
    }else if(PID_output_yaw>5){// only integrate the negative err
      if(err_yaw>0){
        alpha_yaw = 0;  
      }else {
        alpha_yaw = 1;
      }
    }else{
      alpha_yaw = 1;
    }
    //alpha_yaw=1;
    //*****
    
    filtered_yaw_last = filtered_yaw;
    err_yaw_last = err_yaw;
    //PID_output_yaw = 0;
  }
  last = now;
  //*************************************************

  // The following code are sending IMU info back to the remote, which is actually not necessary unless for displaying
//  unsigned long now_radio = millis();
//  if(startMillis >= 1000){
//    // Send the all the FCB measurements (including IMU) back to the remote
//    // FCB_info.orientation = orientation_m;
//    FCB_info.PID_output = PID_output_yaw;
//    FCB_info.filtered_pitch = orientation_m.yaw_rate;// filtered_pitch; 
////    FCB_info.PID_P = err_pitch;
////    FCB_info.PID_D = filtered_pitch_df;
////    FCB_info.PID_I = filtered_pitch_sum;
//    if (now_radio - last_radio >= period){
//      uint8_t b[sizeof(FCB_info)]; // a buffer to store the radio data
//      int len = sizeof(FCB_info); // the length (number of characters)
//      FCB_info.magic = 66; // magic number for communication matching
//      memcpy(b, &FCB_info, len); // transform struct data into bytes and store it into the buffer 'b'
//      rfWrite(b,len); // ... send it out the radio  
//      delay(10);
//      last_radio = now_radio;
//      // **************ing pay attention to disarm_mode. To solve the conflict, two different magic number are used to distinguish the reset state from normal state.
//    }
//  }



  
//  Quad_Info from_remote; // used to receive data from the remote board. This line of code has been put right before loop().
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
        FCB_info.disarm_mode = disarm_mode; // New added for updating FCB_info
        dutyCycle = from_remote.gimbalThrottle;     
      }
    }
    
//    //******echo for testing
//    char a = rfRead();
//    rfWrite(a); // ...send it out the radio.
//    Serial.write(a);
//    //***************

  }
  
  if (disarm_mode==0 && dutyCycle>3){ //arm mode.  
    // Note: The board should be disarmed when throttel=0, or it will often reset your board due to a large current drawing
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    
    // drive the motorN, where N=1,2,3,4 and dutyCycle [0,255]. For education board
//    driveMotors(1,constrain(dutyCycle+PID_output+PID_output_roll-PID_output_yaw,0,255)); //+PID_output
//    driveMotors(2,constrain(dutyCycle-PID_output+PID_output_roll+PID_output_yaw,0,255)); //-
//    driveMotors(3,constrain(dutyCycle-PID_output-PID_output_roll-PID_output_yaw,0,255)); //-
//    driveMotors(4,constrain(dutyCycle+PID_output-PID_output_roll+PID_output_yaw,0,255)); //+ 

    // drive the motors on our self-designed board. 1,2 front, 3,4 back
    driveMotors(1,constrain(dutyCycle+PID_output+PID_output_roll-PID_output_yaw,0,255)); //-PID_output
    driveMotors(2,constrain(dutyCycle+PID_output-PID_output_roll+PID_output_yaw,0,255)); //-
    driveMotors(3,constrain(dutyCycle-PID_output-PID_output_roll-PID_output_yaw,0,255)); //+
    driveMotors(4,constrain(dutyCycle-PID_output+PID_output_roll+PID_output_yaw,0,255)); //+ 

    //driveMotors(1,constrain(dutyCycle,0,255));
    
    //analogWrite(9,dutyCycle); // LED
    //analogWrite(34,dutyCycle); // LED
    analogWrite(9,0); // LED
    analogWrite(34,0); // LED
    // Serial.println("Once more");    
  } else { // disarm mode
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    dutyCycle = 0;
    driveMotors(1,dutyCycle);
    driveMotors(2,dutyCycle);
    driveMotors(3,dutyCycle);
    driveMotors(4,dutyCycle);

    analogWrite(9,20); // LED
    analogWrite(34,20); // LED
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

float complementaryFilter(float angle_m, float velocity_m, float last_angle, float delta_t, float gain){
  // do the complementary filtering by simplying weighting the angle and the integral of velocity
  // Note: delta_t should be with unit 's' instead of 'ms'
  return gain*(last_angle+delta_t*velocity_m)+(1-gain)*angle_m;
  // return (last_angle+delta_t*velocity_m)+angle_m;
}
