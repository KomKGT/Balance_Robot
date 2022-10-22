#include <WiFi.h>

// mqtt library
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

/**KOMGRICH**/
#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else //ESP32
 #include <WiFi.h>
#endif
#include <ModbusIP_ESP8266.h>

const int Holding_Register = 100;
//IPAddress DoitboardIP(192,168,1,160);
IPAddress DoitboardIP(10,61,5,212);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255, 255, 254, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);
const int LOOP_COUNT = 10; //for slave
ModbusIP mb;
ModbusIP sl;
/**KOMGRICH**/

// WiFi
const char *ssid = "FIBOWIFI"; // Enter your WiFi name
const char *password = "Fibo@2538";  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "broker.emqx.io";
const char *topic = "esp32/test";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 8883;

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

// Fill-in information from your Blynk Template here
#define BLYNK_TEMPLATE_ID "TMPLI1016nrY"
#define BLYNK_DEVICE_NAME "2WheelsCarESP32"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial

// ------------------------------------------------------------------------
// blynk
#include "BlynkEdgent.h"
#include "math.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


//#include "glitchfree.h"
//config

const int MpuInterruptPin=34;  // ขา Interrupt Sensor MPU6050 สำหรับส่งค่าให้ esp32
#define MpuIntPin 15

struct JStickData {
  int Xvalue, Yvalue, Up=1, Down=1, Left=1, Right=1, JButton=1;
};

struct MpuYawPitchRoll {
  float yaw, pitch, roll;
};

JStickData JStick; // create Joy Stick data
MpuYawPitchRoll YawPitchRoll;  // create Structure YawPitchRoll

//#define BLYNK_DEBUG
// ------------------------------------------------------------------------
#define APP_DEBUG

// import library for interface MPU6050 with using i2c protocol
#include "Wire.h"

//#include <MPU6050_light.h>

//multitasking
TaskHandle_t Task1;
TaskHandle_t Task2;

// ------------------------------------------------------------------------
// Declaration
// ------------------------------------------------------------------------
//TwoWire I2CWire = TwoWire(0);
// define mpu is object from MPU6050
#define OUTPUT_READABLE_YAWPITCHROLL

MPU6050 mpu;
double input, output;


bool blinkState = false;

unsigned long mh_buffer;
unsigned long servo_buffer;
uint32_t I2Cclock = 10000;
//initiate timer variable
hw_timer_t * timer = NULL;

// import library for drive stepper motor
#include "stepperMotor.h"

// define Mystepper is object from steppermotor class using three parameters (microstep, directionpin, pulsepin)
steppermotor Mystepper(800, 5, 18, 27, 26);

int             LoopTimeMsec = 12;
float           LoopTimeMicrosec = LoopTimeMsec * 1000;
unsigned long   ProgStartTime;  //general Start Time
const int       StartDelay = 5; // msec
float DeltaPos       = 0.0;
float DeltaForward   = 0.0;
float DeltaTurning   = 0.0;
float PositionAB     = 0.0;
float HoldPosition   = 0.0;
float StepsA         = 0.0;
float StepsB         = 0.0;
float CalJstickX     = 0;
float CalJstickY     = 0;
bool  spinning       = false;
bool  spinningOld    = false ;
bool  moving         = false;
int   skipPos; // wait befor starting position controll
int freqA;
int freqB;
//PWM
int PWM_FREQUENCY = 1000; 
int PWM_CHANNEL = 1;
int PWM_RESOLUTION = 8; 
int GPIOPIN1 = 18;
int GPIOPIN2 = 26;
int dutyCycle = 127;
double frequency = 200;

unsigned long mpu_timestamp;

bool firstRun= true;
boolean DirForward=true;
// ------------------------------------------------------------------------
/*  Twiddle Auto Tuning ( params,  dparams);
    https://martin-thoma.com/twiddle/
    Twiddle is an algorithm that tries to find a good choice of parameters p for an algorithm A that returns an error.
*/

float  K = 1.0;
double Kp = 3; //4     Kp = 8 on the rough surface with 300 constant
double Ki = 0; //
double Kd = 0; //0.1
float  Ka = 0.0;
float  Kx = 0.0; 
float pTerm;
float iTerm;
float dTerm;
float error;
float prevError;
float integrated_error;
float SetAngle;
float iAngle;
float eSpeed;
float    Last_error;

//initiate PID for Pos term
volatile float  KPos = 1.0;
volatile double KpPos = 0; //4
volatile double KiPos = 0; //
volatile double KdPos = 0.008; //0.1
volatile float  KaPos = 0;
volatile float  KxPos = 0.0; 
float LastK;
float timeChange;
unsigned long Now;
unsigned long Last_time;
volatile int     PositionA;
volatile int     PositionB;
boolean first=true;
boolean firstPos =true;
float pTermPos;
float iTermPos;
float dTermPos;
float errorPos;
float prevErrorPos;
float integrated_errorPos;
float SetPos;
float iPos;
float eSpeedPos;
float    Last_errorPos;


uint32_t         duty;
uint32_t         period;
uint32_t         Speed ;
int              FifoOverflowCnt;
#include "Twiddle.h"
//const bool AutoTuning = true;  // sets Twiddle on
const bool AutoTuning = false;
// Auto Tuning both PID Controler
//Twiddle Tuning ( 7, Kp , Ki , Kd ,  Ka , KpPos , KiPos , KdPos , KaPos,
//                 0.1,             0.1,           0.01,            0.01,            0.01,              0.01,              0.01,            0.01);
// Auto Tuning only Position Controler

struct PidParameter {
  float          K = 1.0;
  float          Kp = 9;
  float          Ki = 4;
  float          Kd = 0;
  float          Ka = 0;
  float          Kx = 0.0; //
};
struct PidParameterPos {
  float          K = 1.0;
  float          Kp = 0.0;
  float          Ki = 0.0;
  float          Kd = 0.08 ;
  float          Ka = 0.0 ;
  float          Kx = 0.0;
};
PidParameter PidParams;
PidParameterPos PidParamsPos;

Twiddle Tuning ( 3,  PidParamsPos.Kp , PidParamsPos.Ki , PidParamsPos.Kd , PidParamsPos.Ka, PidParams.Kp , PidParams.Ki , PidParams.Kd ,  PidParams.Ka ,
                 0.005,             0.001,           0.001,            0.001,            0.001,              0.01,              0.01,            0.01);
//initiate PID Term
// MH sensor
unsigned long mh_timestamp;
int MHsensor_digital_status;
int MHsensor_analog_status;

// P-Action
BLYNK_WRITE(V0)
{
  Kp = param.asDouble();
  Serial.print("Kp : ");
  Serial.println(Kp);
}

// D-Action
BLYNK_WRITE(V1)
{
  Kd =   param.asDouble();
  Serial.print("Kd : ");
  Serial.println(Kd);
}

// I-Action
BLYNK_WRITE(V2)
{
  Ki =   param.asDouble();
  Serial.print("Ki : ");
  Serial.println(Ki);
}
// Uncomment your board, or configure a custom board in Settings.h
//#define USE_WROVER_BOARD
//#define USE_TTGO_T7
//#define USE_ESP32C3_DEV_MODULE
//#define USE_ESP32S2_DEV_KIT
float average;
// --------------------------------------------------------------
void  PIDAutoTuning()  //Twiddle Auto Tuning
// --------------------------------------------------------------
{
  static int skipT = 0;
  skipT++;
  if (skipT > 10) {
    skipT = 11;

    // average = Tuning.next( Robot.PositionAB, PidParams.Kp , PidParams.Ki , PidParams.Kd ,  PidParams.Ka , PidParamsPos.Kp , PidParamsPos.Ki , PidParamsPos.Kd , PidParamsPos.Ka );
    //a little bit redundancy due to no time to submit project
    average = Tuning.next( (HoldPosition - PositionAB), PidParamsPos.Kp , PidParamsPos.Ki , PidParamsPos.Kd , PidParamsPos.Ka, PidParams.Kp , PidParams.Ki , PidParams.Kd ,  PidParams.Ka);
    KpPos = PidParamsPos.Kp;
    KiPos = PidParamsPos.Ki;
    KdPos = PidParamsPos.Kd;
    KaPos = PidParamsPos.Ka;
    Kp = PidParams.Kp;
    Ki = PidParams.Ki;
    Kp = PidParams.Kp;
    Ka = PidParams.Ka;

    //PidControler.changePIDparams(PidParams); // PID Parameter setzen
    //PidControlerPos.changePIDparams(PidParamsPos); // PID Parameter setzen
  }
}

//initiate interrupt function for MPU 6050
volatile bool mpuInterrupt = false;     // ระบุสัญญาณขา interrupt ไปเป็นสัญญาณ HIGH
void dmpDataReady() {
  mpuInterrupt = true;  // indicates whether MPU interrupt pin has gone high
  digitalWrite(MpuIntPin, !digitalRead(MpuIntPin)); // Toggle  Pin for reading the Frequenzy
}

//MPU 6050
boolean          Running        = false;     // Speed Control is running
float            TuningValue;
float            setPoint = 0;
float            Calibration = 0; //-2.8 ; //-3.2;
// ------------------------------------------------------------------------
// orientation/motion vars
// ------------------------------------------------------------------------
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// ------------------------------------------------------------------------
// MPU control/status vars
// ------------------------------------------------------------------------
bool     dmpReady = false;  // set true if DMP init was successful
uint8_t  mpuIntStatus;      // holds actual interrupt status byte from MPU
uint8_t  devStatus;         // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;         // count of all bytes currently in FIFO
uint8_t  fifoBuffer[64];    // FIFO storage buffer


// --------------------- Sensor aquisition  -------------------------
MpuYawPitchRoll ReadMpuRunRobot()
// --------------------- Sensor aquisition  -------------------------
// wait for MPU interrupt or extra packet(s) available
// --------------------------------------------------------------------
{
  if (mpuInterrupt or fifoCount >= packetSize) {
    if  (mpuInterrupt) mpuInterrupt = false;  // reset interrupt flag
    //digitalWrite(LED_PIN, HIGH); // blink LED to indicate activity
    //    Angle = ReadMpuRunRobot6050() - CalibrationAngle;
    YawPitchRoll = ReadMpuRunRobot6050();
    YawPitchRoll.roll  +=  Calibration;
    // blinkState = !blinkState;
   // digitalWrite(LED_PIN, LOW);
  }
  return YawPitchRoll ;
}
// -------------------------------------------------------------------- -
MpuYawPitchRoll ReadMpuRunRobot6050()
// -------------------------------------------------------------------- -
{
  static float pitch_old;
  static float yaw_old;
  static float yaw_tmp;
  static float yaw_delta;
  //  static float pitch;

  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    //    mpu.setDMPEnabled(false);
    mpu.resetFIFO();
    FifoOverflowCnt ++;
    fifoCount = 0;
    YawPitchRoll.pitch = pitch_old;
    return YawPitchRoll;

  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  // Register 58  lnterrupt Status INT_STATUS
  // MPU-6500 Register Map and Descriptions Revision  2.1
  // Bit [1] DMP_INT This bit automatically sets to 1 when the DMP interrupt has been generated.
  // Bit [0] RAW_DATA_RDY_INT1 Sensor Register Raw Data sensors are updated and Ready to be read.
  if ((mpuIntStatus) & 0x02 || (mpuIntStatus & 0x01)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)  fifoCount = mpu.getFIFOCount();

    while (fifoCount > 0) {
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount = mpu.getFIFOCount();
      //      fifoCount -= packetSize;
    }
    // the yaw/pitch/roll angles (in degrees) calculated from the quaternions coming
    // from the FIFO. Note this also requires gravity vector calculations.
    // Also note that yaw/pitch/roll angles suffer from gimbal lock (for
    // more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    // mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    YawPitchRoll.pitch = -(ypr[1] * 180 / M_PI); //pitch
    YawPitchRoll.roll = -(ypr[2] * 180 / M_PI); //roll
    yaw_tmp  = (abs(ypr[0] * 180 / M_PI));
    yaw_delta = yaw_tmp - yaw_old;
    yaw_old = yaw_tmp;
    YawPitchRoll.yaw  += yaw_delta;

    // actual quaternion components in a [w, x, y, z]
    // YawPitchRoll.pitch = (q.y) * 180;
    // YawPitchRoll.yaw = (q.z );
    // YawPitchRoll.yaw =  mapfloat(YawPitchRoll.yaw , -1.0, 1.0, 0.0, 360.0);
  }
  pitch_old = YawPitchRoll.pitch ;
  return  YawPitchRoll ;
}
// --------------------------------------------------------------
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//PID for calculate position
/**********************************************************************/
float  calculatePos (float iPos, float isetPos )
/**********************************************************************/
// new calculation of Steps per Second // PID algorithm
{
  Now = micros();
  if (firstPos) {
    firstPos = false;
    Last_time = Now;
    integrated_errorPos = 0;
  }
  timeChange = (Now - Last_time)  ;
  timeChange = timeChange / 1000.0;  // in millisekunden
  errorPos = isetPos -  iPos;

  if ( timeChange != 0) {
    dTermPos =  1000.0 * KdPos * (errorPos - Last_errorPos) /  timeChange  ;
  }

  integrated_errorPos = integrated_errorPos  + ( errorPos * timeChange );
  iTermPos =   KiPos * integrated_errorPos / 1000.0;

  pTermPos =   KpPos  * errorPos + ( KaPos * integrated_errorPos ); // modifying Kp

  // Compute PID Output in Steps per second
  eSpeedPos = KPos * (pTermPos + iTermPos + dTermPos) ;

  /*Remember something*/
  Last_time  = Now;
  Last_errorPos = errorPos;

  //  digitalWrite(TestPIDtime, !digitalRead(TestPIDtime)); // Toggle  Pin for reading the Frequenzy
  // eSpeed = constrain (eSpeed , -500.0 , 500.0 ); // 10 Steps per Second because Microstep
 
 return eSpeedPos;  // Steps per Second
}

//PID for stability
/**********************************************************************/
float  calculate (float iAngle, float isetPoint )
/**********************************************************************/
// new calculation of Steps per Second // PID algorithm
{
  Now = micros();
  if (first) {
    first = false;
    Last_time = Now;
    integrated_error = 0;
  }
  timeChange = (Now - Last_time)  ;
  timeChange = timeChange / 1000.0;  // in millisekunden
  if (integrated_error > 0 & error < 0){
      integrated_error = 0; 
    } 
    if (integrated_error < 0 & error > 0){
      integrated_error = 0;
    }
  error = isetPoint -  iAngle;

  if ( timeChange != 0) {
    dTerm =  1000.0 * (sl.Hreg(102)/10) * (error - Last_error) /  timeChange  ;
  }

  integrated_error = integrated_error  + ( error * timeChange );
  iTerm =   (sl.Hreg(101)/10) * integrated_error / 1000.0;

  pTerm =   (sl.Hreg(100)/10)  * error + ( Ka * integrated_error ); // modifying Kp

  // Compute PID Output in Steps per second
  eSpeed = K * (pTerm + iTerm + dTerm) ;
  eSpeed = constrain(eSpeed,-4000,4000);
  /*Remember something*/
  Last_time  = Now;
  Last_error = error;

  //  digitalWrite(TestPIDtime, !digitalRead(TestPIDtime)); // Toggle  Pin for reading the Frequenzy
  // eSpeed = constrain (eSpeed , -500.0 , 500.0 ); // 10 Steps per Second because Microstep
 
 return eSpeed;  // Steps per Second
}

//checking position A,B
void ISR_PWMA() {
  if (Mystepper.DirForward)   {
    PositionA ++;
  } else {
    PositionA --;
  }
}
//---------------------------------------------------------------------/
void ISR_PWMB() {
  //  if ( PidControlerPos.DirForward ) {
  if (Mystepper.DirForward) {
    PositionB ++;
  }  else  {
    PositionB --;
  }
}


void MpuInit()
// --------------------------------------------------------------
// MPU6050_6Axis_MotionApps20.h
// 0x02,   0x16,   0x02,   0x00, 0x09  => 50msec 20 Hz
// This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
// 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
// DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))
// It is important to make sure the host processor can keep up with reading and processing
// the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.

{
  // after Reset of Arduino there is no Reset of MPU
  pinMode(MpuInterruptPin, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(152);
  mpu.setYGyroOffset(-13);
  mpu.setZGyroOffset(11);
  mpu.setZAccelOffset(1021);
  mpu.setXAccelOffset(-1621); //
  mpu.setYAccelOffset(-2813);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(MpuInterruptPin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    //    mpu.resetFIFO();  // after Reset importand

  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
   // lcd.clear();
  //  lcd.setCursor(0, 0);
  //  lcd.print( "Error MPU6050 "  );
    do  {} while ( 1 == 1);
  }
}

/**********************************************************************/
void Run(MpuYawPitchRoll YawPitchRoll , int &iPositionA, int &iPositionB, JStickData JStick)
/**********************************************************************/
{ /*
    Stepper Motor: Unipolar/Bipolar, 200 Steps/Rev => 1.8 degree per step
    Wheel Diameter 60mm = > Distance per Pulse Dpp = d * pi / 200 =  1,25 mm
    Distance per Revolution =  250,2 mm
    Max 1000 Steps per Second = 5 Revolutions => 12500 mm Distance

    "iPositionA" in  microsteps
    8 microsteps = 1 full Step
    1 microstepp = 0,125 full steps
    after division one change in "PositionA" = 0.01 microstepp and 0,0125 full steps = 0,01727 mm
  */
  const int     tDelay = 10;

  PositionAB = ((float(iPositionA ) + float(iPositionB )) / 100);
/*
  Serial.print(iPositionA );  //grau
  Serial.print(",");
  Serial.print(iPositionB );  //grau
  Serial.print(",");
  Serial.print(HoldPosition );  //grau
  Serial.println(" ");
*/
  /********************* driverless **************************************/
  if (firstRun) {
    firstRun = false;
    skipPos = - (2 * tDelay); // first time wait a little bit longer
    CalJstickX = JStick.Xvalue;  // calibrate Joy Stick
    CalJstickY = JStick.Yvalue;
    spinningOld = false;
   // pMotorA->RunMode();
   // pMotorB->RunMode();
    HoldPosition = PositionAB;
  }
  //JStick.Xvalue = JStick.Xvalue - CalJstickX ;
  //JStick.Yvalue = JStick.Yvalue - CalJstickY;
  JStick.Xvalue=0;
  JStick.Yvalue=0;
  DeltaForward = float(JStick.Yvalue) / 100.0 ;
  DeltaTurning = float(JStick.Xvalue ) / 4.0;

  if ((abs(JStick.Xvalue) > 5 ) || (abs(JStick.Yvalue) > 5 ) )  {
    spinning = true;
    if (!spinningOld) {  // changing to spinning
    }
    spinningOld = true;
    HoldPosition = PositionAB;
  } else {
    spinning = false;
    if (spinningOld) {  // changing to not spinning
      skipPos = - (2 * tDelay); // wait a little bit longer
    }
    spinningOld = false;
  }

  if (!spinning) {
    if (++skipPos  >= tDelay) { // to delay the calls, Position Control should be 10 times lower than Motor Control
      skipPos = 0;
      // PID calculation Delta Position
      // DeltaPos is necessary for the robot to keep its position clean after moving forward or backward
      DeltaPos =  calculatePos(HoldPosition, PositionAB);
      // slowly adjust the hold position to the current position
      // so, DeltaPos is slowly reduced and the pPidControler
      // works again without offset
      HoldPosition = (0.9 * HoldPosition) + ( 0.1 * PositionAB);
      Serial.print("\tPositionAB:");
      Serial.print(PositionAB);
      Serial.print("\tHoldposition:");
      Serial.print(HoldPosition);
      Serial.print("\tDeltaPos:");
      Serial.print(DeltaPos);
      Serial.print("\n");
    }
  }
 
  // PID calculation of new steps
  StepsA = calculate(YawPitchRoll.roll, -DeltaForward + DeltaPos );
  StepsB = StepsA;

  StepsA =  StepsA  + DeltaTurning;  // Moving right or left
  StepsB =  StepsB  - DeltaTurning;
  Serial.print("\tStepsA:");
      Serial.print(StepsA);
      Serial.print("\n");
  StepsA = Mystepper.stepperMove(0, StepsA);  // run the motors
  StepsB = Mystepper.stepperMove(1, StepsB);
  StepsA = constrain(StepsA,-2000,2000);
  uint32_t freqA_abs = abs(StepsA); // only positive Steps
  uint32_t freqB_abs = abs(StepsB);  // direction via PinDir

  //ptrpwm->setFreq2( freqA_abs, freqB_abs  );
  ledcWriteTone(PWM_CHANNEL,freqA_abs); //300 can stand
  ledcWrite(PWM_CHANNEL,127);



}

void setup()
{ 
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
// MH sensor
//  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(34,INPUT);  
  Serial.begin(115200); // initial serial communication
  //connect to blynk
  BlynkEdgent.begin();

  /**KOMGRICH**/
  //WiFi.begin("FIBOWIFI", "Fibo@2538");
  WiFi.begin("iPhone", "0914329855");
  // if (!WiFi.config(DoitboardIP, gateway, subnet, primaryDNS, secondaryDNS)) {
  //   Serial.println("STA Failed to configure");
  // }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mb.server();
  sl.client();
  mb.addHreg(Holding_Register,0,3); //Reg 100 = Kp,Reg 101 = Ki,Reg 102 = Kd
  sl.Hreg(100,90);
  sl.Hreg(101,40);
  sl.Hreg(102,0);
  /**KOMGRICH**/ 

  /*
  //connect to wifi network-----------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  //connecting to a mqtt broker
  espClient.setCACert(root_ca);
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
          Serial.println("Public emqx mqtt broker connected");
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  }
  // publish and subscribe
  client.publish(topic, "Hi EMQX I'm ESP32 ^^");
  client.subscribe(topic);
//-----------------------------------------------------------------
*/
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  //pwm
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(GPIOPIN1, PWM_CHANNEL);// frequency left wheel
  ledcAttachPin(GPIOPIN2, PWM_CHANNEL);// frequency right wheel


  // for Checking the position from Stepper Motor
    attachInterrupt(digitalPinToInterrupt(GPIOPIN1), ISR_PWMA, RISING );
    attachInterrupt(digitalPinToInterrupt(GPIOPIN2), ISR_PWMB, RISING );
    /**********************************************************************/
    YawPitchRoll.pitch = 0;
    YawPitchRoll.roll=0;
    Speed = 0;
    MpuInit();
//core 0 initiate
xTaskCreatePinnedToCore(
             Task1code, /* Task function. */
             "Task1",   /* name of task. */
             10000,     /* Stack size of task */
             NULL,      /* parameter of the task */
             1,         /* priority of the task */
             &Task1,    /* Task handle to keep track of created task */
             0);        /* pin task to core 0 */
 
}
void callback(char *topic, byte *payload, unsigned int length) {
 Serial.print("Message arrived in topic: ");
 Serial.println(topic);
 Serial.print("Message:");
 for (int i = 0; i < length; i++) {
     Serial.print((char) payload[i]);
 }
 Serial.println();
 Serial.println("-----------------------");
}


void Task1code( void * parameter) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;) {
      
      vTaskDelay(10);
  }
}

//---------------------------------------------------------------------/
void loop() {
  //BlynkEdgent.run();
  mb.task();
  sl.task();  
  
  Now = micros();
  if (first){
    first=false;
    Last_time = Now;
    integrated_error = 0;
    prevError = 0.0;
    PositionAB = 0.0;
    HoldPosition = PositionAB;
    SetAngle =0;
    skipPos =0;
    
  }
    // --------------------- Sensor aquisition  -------------------------
  YawPitchRoll = ReadMpuRunRobot() ; // wait for MPU interrupt or extra packet(s) available
  // --------------------------------------------------------------#
if (!Running) {
    if ( ( abs(YawPitchRoll.roll) < 15.0 )  && ( millis() > ( ProgStartTime +  StartDelay)))  {
      Running = true; // after Delay time set Running true
      Serial.print("start running soon");
    }
}
int PositionAtemp;
int PositionBtemp;
if (Running) {

      //because conflicting declaration 'volatile int PositionA'
      PositionBtemp = PositionB ;
      PositionAtemp = PositionA ;
      Run( YawPitchRoll, PositionAtemp , PositionBtemp,  JStick ); //   Robot.Run

      if (AutoTuning)  PIDAutoTuning(); // auto Tuning via Twiddle

    }
   
 
}
