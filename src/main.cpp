#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
// #include <Ticker.h>

#define PIN        D5 
#define NUMPIXELS 20
#define BRIGHTNESS_AXIS 0
#define BRIGHTNESS_TRIGGER_AXIS x
#define BRIGHTNESS_TRIGGER_THRESHOLD 20000
#define BRIGHTNESS_CONTROL_TIME 2
#define INTERRUPT_PIN 15  // use pin 2 on Arduino Uno & most boards


//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

enum Mode { ILLUMINATE, ANIMATE, OFF };
bool initMode = true;
Mode operationMode;

struct LedHsvColor{
  uint16_t hue;
  uint8_t sat;
  uint8_t val;
};

const LedHsvColor darkOrange = {6189, 255, 255};
const LedHsvColor darkYellow = {9284, 255, 255};
const LedHsvColor lightYellow = {10923, 104, 255};
const LedHsvColor white = {0, 0, 255};

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool trackRelativeYpr = true;
bool controlBrightness = false;
float prevYpr[3];
float relativeYpr[3];
float referenceYpr[3];

float val = 127;
LedHsvColor pixelColors[NUMPIXELS];

void showPixels(){
  pixels.clear();
   for(int i = 0; i < NUMPIXELS; i++){
      LedHsvColor c = pixelColors[i];
      Serial.print(i);
      Serial.print("\t");
      Serial.println(c.hue);
      pixels.setPixelColor(i, pixels.ColorHSV(c.hue, c.sat, c.val));
    }
  pixels.show();
}

void setDefaultPixelColors(){
  pixelColors[0]=darkOrange;
  pixelColors[1]=darkYellow;
  pixelColors[2]=lightYellow;
  pixelColors[3]=white;
  pixelColors[4]=darkOrange;
  pixelColors[5]=darkYellow;
  pixelColors[6]=lightYellow;
  pixelColors[7]=white;
  pixelColors[8]=darkOrange;
  pixelColors[9]=darkYellow;
  pixelColors[10]=lightYellow;
  pixelColors[11]=lightYellow;
  pixelColors[12]=lightYellow;
  pixelColors[13]=lightYellow;
  pixelColors[14]=lightYellow;
  pixelColors[15]=lightYellow;
  pixelColors[16]=lightYellow;
  pixelColors[17]=lightYellow;
  pixelColors[18]=lightYellow;
  pixelColors[19]=lightYellow;
}

uint16_t fastSigmoid(uint16_t center, uint16_t range, uint16_t x){
  int result = (range + center/2) / (1 + exp(-0.002*(x-center)) ) + center/2;
  // int result = ((x - center) / (1 + abs(x - center)))*range + center;
  result  = std::min(center + range, result);
  result  = std::max(center - range, result);

  Serial.print(center);
  Serial.print("\t");
  Serial.print(range);
  Serial.print("\t");
  Serial.print(x);
  Serial.print("\t");
  Serial.println(result);
  return result;

}

void setAnimatedDefaultPixelColors(){
  int8_t hueRand = rand() % 101 - 50;     

  pixelColors[0].hue = fastSigmoid(darkOrange.hue, 2570, pixelColors[0].hue + hueRand);
  hueRand = rand() % 101 - 50;     
  pixelColors[1].hue = fastSigmoid(darkYellow.hue, 2570, pixelColors[1].hue + hueRand);

  hueRand = rand() % 101 - 50;     
  pixelColors[2].hue = fastSigmoid(lightYellow.hue, 2570, pixelColors[2].hue + hueRand);
}

void applyGlobalValue(){
  for(int i =0; i<NUMPIXELS; i++){
    pixelColors[i].val = val;
  }
}

void initializeMpu(){  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void ICACHE_RAM_ATTR disableBrightnessControl(){
    controlBrightness = false;
    Serial.println("Brightness control disabled");
}

void processYpr(){
  prevYpr[0] = ypr[0] + M_PI;
  prevYpr[1] = ypr[1] + M_PI;
  prevYpr[2] = ypr[2] + M_PI;

  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  if(trackRelativeYpr){
    // Positive roll over
    for(int i = 0; i<3; i++){
      if(ypr[i] + M_PI < M_PI * 0.5 && prevYpr[i] > M_PI * 1.5){
        relativeYpr[i] = relativeYpr[i] + ypr[i] + M_PI + (2 * M_PI - prevYpr[i]);
      }
      // Negative roll over
      else if(prevYpr[i] < M_PI * 0.5 && ypr[i] + M_PI > M_PI * 1.5){
        relativeYpr[i] = relativeYpr[i] + prevYpr[i]  + (2 * M_PI - (ypr[i] + M_PI));
      }
      // No roll over
      else{
         relativeYpr[i] = relativeYpr[i] + ((ypr[i] + M_PI) - prevYpr[i]);
      }
    }
  }

  if(controlBrightness){
    float cVal =  (ypr[BRIGHTNESS_AXIS] + M_PI) * 180 / M_PI;
    float cPrevValue = prevYpr[BRIGHTNESS_AXIS] * 180 / M_PI;

    // Positive roll over
    if(cVal < 90 && cPrevValue > 270){
      val = val + (cVal + (360 - cPrevValue));
    }
    // Negative roll over
    else if(cPrevValue < 90 && cVal > 270){
      val = val + (cPrevValue  + (360 - cVal));
    }
    // No roll over
    else{
      val = val + (cVal - cPrevValue);
    }

    val = std::max(0.0f, val);
    val = std::min(255.0f, val);
    applyGlobalValue();
  }
}

void processAccel(){
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  if(!controlBrightness && aaWorld.BRIGHTNESS_TRIGGER_AXIS > BRIGHTNESS_TRIGGER_THRESHOLD){
    controlBrightness = true;
    Serial.println("Brightness control enabled");
    timer1_write(312500 + BRIGHTNESS_CONTROL_TIME);
  }

  // Serial.print("areal\t");
  // Serial.print(aaReal.x);
  // Serial.print("\t");
  // Serial.print(aaReal.y);
  // Serial.print("\t");
  // Serial.print(aaReal.z);
  // Serial.print("\taworld\t");
  // Serial.print(aaWorld.x);
  // Serial.print("\t");
  // Serial.print(aaWorld.y);
  // Serial.print("\t");
  // Serial.println(aaWorld.z);
}

void processMpuData(){
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);

    processYpr();
    processAccel();
  }
}

void resetRelativeYpr(){
  relativeYpr[0] = 0;
  relativeYpr[1] = 0;
  relativeYpr[2] = 0;
}

void resetReferenceYpr(){
  referenceYpr[0] = ypr[0] + M_PI;
  referenceYpr[1] = ypr[1] + M_PI;
  referenceYpr[2] = ypr[2] + M_PI;
}

void updatePixels(){
  if(operationMode == OFF){

  }else if(operationMode == ILLUMINATE){
    if(initMode){
      setDefaultPixelColors();
      initMode = false;
    }
  }else if(operationMode == ANIMATE){
    if(initMode){
      setDefaultPixelColors();
      initMode = false;
    }else{
      setAnimatedDefaultPixelColors();
    }
  }
  showPixels();
}


void setup() {
  Serial.begin(115200);

  operationMode = ANIMATE;
  initMode = true;
  resetRelativeYpr();
  timer1_attachInterrupt(disableBrightnessControl);
  timer1_enable(TIM_DIV256, TIM_EDGE, TIM_SINGLE);
  initializeMpu();
  processMpuData();
  resetReferenceYpr();
  
  pixels.begin();
  pixels.show();  
  // for(int i = 0; i < NUMPIXELS; i++){
  //   pixels.setPixelColor(i, 255, 0, 255);
  // }
  // pixels.show();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  processMpuData();
  updatePixels();
}