#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>           // I2C for MPU6050 & BMP180
#include <Adafruit_MPU6050.h>     // Adafruit MPU6050 driver
#include <Adafruit_Sensor.h>      // Required by Adafruit_MPU6050
#include <Adafruit_BMP085.h>      // Adafruit BMP085/BMP180 driver


// MPU6050 IMU object
Adafruit_MPU6050 mpu;

// BMP180 barometer object (optional altitude)
Adafruit_BMP085 bmp;

// =========================
//   Joystick Packet Struct
// =========================
struct Direction {
  int x; // -127 ... +127
  int y; // -127 ... +127
};

struct struct_message {
  Direction left;   // left.x = yaw_input   | left.y = throttle_input
  Direction right;  // right.x = roll_setpt | right.y = pitch_setpt
  bool startToggle; // New boolean field for start/stop toggle
};


struct_message incomingData; 

volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;

int ESCfreq=500;
float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0.5; float IAnglePitch=IAngleRoll;
float DAngleRoll=0.007; float DAnglePitch=DAngleRoll;
int RateCalibrationNumber;
volatile float ReceiverValue[4]; 
#define PI 3.1415926535897932384626433832795

float PRateRoll = 0.625;
float IRateRoll = 2.1;
float DRateRoll = 0.0088;

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

uint32_t LoopTimer;
// float t=0.004;    
unsigned long prevTime = 0;  //time cycle

// INITIALIZING THE MOTORS
Servo mot1; //FR CW
Servo mot2; //BR CCW
Servo mot3; //BL CW
Servo mot4; // FL CCW

// ASSIGNING PINS THE ESC MOTORS
const int mot1_pin = 13; // ccw back right
const int mot2_pin = 12; //cw front right
const int mot3_pin = 26; //is 14 for some designed FC on perforated baords ccw front left
const int mot4_pin = 27; //cw back left
const int LED_PIN=2;

// PID VALUES
volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

int ThrottleIdle = 1150;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;


float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;



void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x08);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  // AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29; //*1/(3.142/180);
  // AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;
  // Compute Roll and Pitch using atan2 (robust method)
 AngleRoll  = atan2(AccY, AccZ) * 180.0 / PI;
 AnglePitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

}



void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);



void setup() {
   Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(LED_PIN,OUTPUT);
Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

   if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found");
    // Blink LED rapidly to indicate error
    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

 	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  delay(1000);
  mot1.attach(mot1_pin,1000,2000);
  delay(1000);
  mot1.setPeriodHertz(ESCfreq);
  delay(100);
  mot2.attach(mot2_pin,1000,2000);
  delay(1000);
  mot2.setPeriodHertz(ESCfreq);
  delay(100);
  mot3.attach(mot3_pin,1000,2000);
  delay(1000);
  mot3.setPeriodHertz(ESCfreq);
  delay(100);
  mot4.attach(mot4_pin,1000,2000);
  delay(1000);
  mot4.setPeriodHertz(ESCfreq);
  delay(100);

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
   delay(500);
WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    while (true) { delay(100); } // Halt on ESP-NOW error
  }
//   RateCalibrationRoll=0.27;
// RateCalibrationPitch=-0.85;
// RateCalibrationYaw=-2.09;
// AccXCalibration=0.03;
// AccYCalibration=0.01;
// AccZCalibration=-0.07;
 for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    AccXCalibration+=AccX;
    AccYCalibration+=AccY;
    AccZCalibration+=AccZ;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  AccXCalibration/=2000;
  AccYCalibration/=2000;
  AccZCalibration/=2000;



  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW Receiver Ready");
    Serial.println("Setup Complete. Awaiting Arming Command.\n");

    for ( int count =0; count < 5; count++){
       digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
      
     }
LoopTimer = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
//enter your loop code here
 unsigned long now = micros();
  float t = (now - prevTime) / 1000000.0; // convert microseconds to seconds
  prevTime = now;
 gyro_signals();


RateRoll -= RateCalibrationRoll;
RatePitch -= RateCalibrationPitch;
RateYaw -= RateCalibrationYaw;

AccX -= AccXCalibration ;
AccY -= AccYCalibration ;
AccZ -= AccZCalibration;

  // AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29;
  // AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;
  //updated version of it
   AngleRoll  = atan2(AccY, AccZ) * 180.0 / PI;
 AnglePitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;


complementaryAngleRoll=0.991*(complementaryAngleRoll+RateRoll*t) + 0.009*AngleRoll;
complementaryAnglePitch=0.991*(complementaryAnglePitch+RatePitch*t) + 0.009*AnglePitch;
// Clamping complementary filter roll angle to ±20 degrees
complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);
Serial.println(complementaryAngleRoll);
Serial.println(complementaryAnglePitch);



DesiredAngleRoll=0.1*(ReceiverValue[0]-1500);
DesiredAnglePitch=0.1*(ReceiverValue[1]-1500);
InputThrottle=ReceiverValue[2];
DesiredRateYaw=0.15*(ReceiverValue[3]-1500);


// Inlined PID equation for Roll
ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
PtermRoll = PAngleRoll * ErrorAngleRoll;
ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
DesiredRateRoll = PIDOutputRoll;
PrevErrorAngleRoll = ErrorAngleRoll;
PrevItermAngleRoll = ItermRoll;
Serial.println(DesiredRateRoll);

ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
PtermPitch = PAnglePitch * ErrorAnglePitch;
ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
DesiredRatePitch = PIDOutputPitch;
Serial.println(DesiredRatePitch);
PrevErrorAnglePitch = ErrorAnglePitch;
PrevItermAnglePitch = ItermPitch;

// Compute errors
ErrorRateRoll = DesiredRateRoll - RateRoll;
ErrorRatePitch = DesiredRatePitch - RatePitch;
ErrorRateYaw = DesiredRateYaw - RateYaw;

// Roll Axis PID
PtermRoll = PRateRoll * ErrorRateRoll;
ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

// Update output and previous values for Roll
InputRoll = PIDOutputRoll;
PrevErrorRateRoll = ErrorRateRoll;
PrevItermRateRoll = ItermRoll;

// Pitch Axis PID
PtermPitch = PRatePitch * ErrorRatePitch;
ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

// Update output and previous values for Pitch
InputPitch = PIDOutputPitch;
PrevErrorRatePitch = ErrorRatePitch;
PrevItermRatePitch = ItermPitch;

// Yaw Axis PID
PtermYaw = PRateYaw * ErrorRateYaw;
ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);  // Clamp ItermYaw to [-400, 400]
DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);  // Clamp PIDOutputYaw to [-400, 400]


// Update output and previous values for Yaw
InputYaw = PIDOutputYaw;
PrevErrorRateYaw = ErrorRateYaw;
PrevItermRateYaw = ItermYaw;


  // if (InputThrottle > 1800)
  // {
  //   InputThrottle = 1800;
  // }

  
  MotorInput1 =  (InputThrottle + InputRoll + InputPitch + InputYaw); // back right - clockwise
  MotorInput2 =  (InputThrottle +InputRoll - InputPitch - InputYaw); // front right -  counterclockwise
  MotorInput3 =  (InputThrottle - InputRoll - InputPitch + InputYaw); // front left  -  clockwise
  MotorInput4 =  (InputThrottle - InputRoll + InputPitch - InputYaw); //back left - counterclockwise


  if (MotorInput1 > 2000)
  {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000)
  {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000)
  {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000)
  {
    MotorInput4 = 1999;
  }


// int ThrottleIdle = 1150;
// int ThrottleCutOff = 1000;
  // if (MotorInput1 < ThrottleIdle)
  // {
  //   MotorInput1 = ThrottleIdle;
  // }
  // if (MotorInput2 < ThrottleIdle)
  // {;
  //   MotorInput2 = ThrottleIdle;
  // }
  // if (MotorInput3 < ThrottleIdle)
  // {
  //   MotorInput3 = ThrottleIdle;
  // }
  // if (MotorInput4 < ThrottleIdle)
  // {
  //   MotorInput4 = ThrottleIdle;
  // }

 if (!incomingData.startToggle) {
  // Reset all complementary angles
  complementaryAngleRoll = 0;
  complementaryAnglePitch = 0;

  // Reset desired angles
  DesiredAngleRoll = 0;
  DesiredAnglePitch = 0;
  DesiredRateYaw = 0;
  DesiredRateRoll = 0;
  DesiredRatePitch = 0;

  // Reset errors
  ErrorAngleRoll = 0;
  ErrorAnglePitch = 0;
  PrevErrorAngleRoll = 0;
  PrevErrorAnglePitch = 0;

  ErrorRateRoll = 0;
  ErrorRatePitch = 0;
  ErrorRateYaw = 0;

  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;

  // Reset PID terms
  PtermRoll = 0;
  ItermRoll = 0;
  DtermRoll = 0;
  PrevItermAngleRoll = 0;
  PrevItermRateRoll = 0;

  PtermPitch = 0;
  ItermPitch = 0;
  DtermPitch = 0;
  PrevItermAnglePitch = 0;
  PrevItermRatePitch = 0;

  PtermYaw = 0;
  ItermYaw = 0;
  DtermYaw = 0;
  PrevItermRateYaw = 0;

  // Reset outputs
  PIDOutputRoll = 0;
  PIDOutputPitch = 0;
  PIDOutputYaw = 0;

  InputRoll = 0;
  InputPitch = 0;
  InputYaw = 0;

  InputThrottle = 0;

  // Optional: Stop motors safely
  MotorInput1 = 1000;
  MotorInput2 = 1000;
  MotorInput3 = 1000;
  MotorInput4 = 1000;

 mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
}

// Calculate motor control values directly
mot1.writeMicroseconds(MotorInput1);
mot2.writeMicroseconds(MotorInput2);
mot3.writeMicroseconds(MotorInput3);
mot4.writeMicroseconds(MotorInput4);


  // while (micros() - LoopTimer < (t*1000000));
  // {
  //   LoopTimer = micros();
  // }

}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len) {
  if (len != sizeof(incomingData)) {
    Serial.println("Received data size mismatch!");
    return;
  }
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  // Get raw joystick values
  int joyY_throttle = incomingData.left.y;
  int joyX_yaw      = incomingData.left.x;
  int joyX_roll     = incomingData.right.x;
  int joyY_pitch    = incomingData.right.y;
ReceiverValue[2]=map(joyY_throttle, -127, 127, 2000, 1000);
ReceiverValue[0]=map(joyX_roll, -127, 127, 1000, 2000);
ReceiverValue[3]=map(joyX_yaw, -127, 127, 1000, 2000);
ReceiverValue[1]=map(joyY_pitch, -127, 127, 2000, 1000);
Serial.println(ReceiverValue[0]);
Serial.println(ReceiverValue[1]);
Serial.println(ReceiverValue[2]);
Serial.println(ReceiverValue[3]);
Serial.println(incomingData.startToggle);
  
  // Debug print joystick mapping and armed state
  
  // Serial.print(" | Joy T: ");   Serial.print(joyY_throttle);
  // Serial.print(" -> Thr: ");   Serial.print(throttleCmd);
  // Serial.print(" | JxY: ");   Serial.print(joyX_yaw);
  // Serial.print(" -> dY: ");    Serial.print(desiredYawRate, 1);
  // Serial.print(" | JxR: ");   Serial.print(joyX_roll);
  // Serial.print(" -> dR: ");    Serial.print(desiredRoll, 1);
  // Serial.print(" | JyP: ");   Serial.print(joyY_pitch);
  // Serial.print(" -> dP: ");    Serial.println(desiredPitch, 1);
}
