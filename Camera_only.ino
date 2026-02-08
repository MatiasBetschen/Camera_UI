#include <Wire.h>
#include <SPI.h>
#include <ArduCAM.h>
#include "memorysaver.h"
#include <Adafruit_MLX90614.h>
#include <SparkFunLSM6DS3.h>
#include <SD.h>


#if !(defined OV2640_MINI_2MP)
#error Please enable OV2640_MINI_2MP in memorysaver.h
#endif
#define VBATPIN A2
uint32_t imageCounter = 0;
char imageFilename[20];

// --------------------
// PIN DEFINITIONS
// --------------------
#define CAM_CS_PIN 10
#define LED_Pin 12
// --------------------
// CONSTANTS
// --------------------
#define LINE_TIME_US 30.0
#define MAX_EXPOSURE_LINES 0xFFFFF
// --------------------
// IMPACT FILTERING
// --------------------
#define LPF_ALPHA            0.90f   // gravity LPF
#define IMPACT_ENERGY_THRESH 1.6f    // g
#define IMPACT_WINDOW_MS     20      // integration window
#define IMPACT_COOLDOWN_MS   1500

// --------------------
// OBJECTS
// --------------------
ArduCAM myCAM(OV2640, CAM_CS_PIN);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
LSM6DS3 imu(I2C_MODE, 0x6B);

// --------------------
// IMU VARIABLES
// --------------------
float gBiasX = 0, gBiasY = 0, gBiasZ = 0;
unsigned long lastIMUTime = 0;
bool deployed = false;
// Integrated angles (deg)
float angleX = 0, angleY = 0, angleZ = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 200; // 5Hz
// --------------------
// AUTO CAPTURE
// --------------------
bool jpegSending = false;

bool cameraBusy = false;
bool autoCaptureEnabled = false;

#define IMPACT_G_THRESH 1.5  // Adjust based on drop height (2-4g typical)
#define IMPACT_COOLDOWN_MS 150 // Prevent multiple triggers
float lastAccelMag = 0;
unsigned long lastImpactTime = 0;
bool impactPending = false;
float gLPX = 0, gLPY = 0, gLPZ = 0;
float impactEnergy = 0;
unsigned long impactWindowStart = 0;
// --------------------
// VELOCITY-BASED IMPACT
// --------------------
#define LPF_ALPHA            0.90f   // gravity low-pass
#define VEL_DAMPING          0.98f   // kills drift
#define STATIONARY_VEL_MPS   0.05f   // ~5 cm/s
#define STATIONARY_TIME_MS  120     // must stay still this long
#define IMPACT_COOLDOWN_MS  1500
// Gravity estimate
float gLPX = 0, gLPY = 0, gLPZ = 0;

// Velocity estimate (m/s)
float velX = 0, velY = 0, velZ = 0;

unsigned long lastVelTime = 0;
unsigned long stationaryStart = 0;
unsigned long lastImpactTime = 0;

bool impactPending = false;


// --------------------
// SETUP
// --------------------
void setup() {
  uint8_t vid, pid;
  pinMode(LED_Pin, OUTPUT);
  digitalWrite(LED_Pin, LOW);
  Wire.begin();
  Serial.begin(115200);
  delay(2000);

  SPI.begin();
  pinMode(CAM_CS_PIN, OUTPUT);
  digitalWrite(CAM_CS_PIN, HIGH);

  // Reset ArduCAM
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  // SPI test
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  if (myCAM.read_reg(ARDUCHIP_TEST1) != 0x55) {
    Serial.println("SPI ERROR");
    while (1);
  }

  // Detect OV2640
  myCAM.wrSensorReg8_8(0xFF, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);

  if (vid != 0x26) {
    Serial.println("OV2640 NOT FOUND");
    while (1);
  }

  Serial.println("OV2640 detected");

  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
  myCAM.clear_fifo_flag();

  if (!mlx.begin()) {
    Serial.println("MLX90614 error");
    while (1);
  }

  if (imu.begin() != 0) {
    Serial.println("IMU init failed");
    while (1);
  }

  Serial.println("Calibrating gyro...");
  calibrateGyro();

  lastIMUTime = millis();


  Serial.println("Ready");
}

// --------------------
// LOOP
// --------------------
void loop() {
 // Send deferred impact message when safe
  if (impactPending && !jpegSending) {
    Serial.println("IMPACT");
    impactPending = false;

    // Optional: stop auto capture immediately
    autoCaptureEnabled = false;
    deployed = false;
  }

  if (deployed && autoCaptureEnabled && !cameraBusy) {
    cameraBusy = true;
    captureJPEG();       // blocking, but safe
    cameraBusy = false;
  }


  // 1) Send telemetry at fixed rate
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS && deployed) {
    lastTelemetryTime = millis();

    float temp = mlx.readObjectTempC();

    // Read IMU data
    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();
    float gx = imu.readFloatGyroX() - gBiasX;
    float gy = imu.readFloatGyroY() - gBiasY;
    float gz = imu.readFloatGyroZ() - gBiasZ;
    detectImpact(ax, ay, az);
    // Send in your required format
    Serial.print("TMP:");
    Serial.println(temp, 2);

    Serial.print("IMU:");
    Serial.print(ax, 2); Serial.print(",");
    Serial.print(ay, 2); Serial.print(",");
    Serial.print(az, 2); Serial.print(",");
    Serial.print(gx, 2); Serial.print(",");
    Serial.print(gy, 2); Serial.print(",");
    Serial.println(gz, 2);

    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back 
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    float voltage = measuredvbat;
    float percent = (voltage - 3.0) / (4.2 - 3.0) * 100.0;
    // clamp to 0–100%
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    Serial.print("V:");
    Serial.println(percent,1);
  }

  // 2) Handle commands from PC (non-blocking)
  if (Serial.available()) {
    uint8_t cmd = Serial.read();

    switch (cmd) {
      case 0x00: myCAM.OV2640_set_JPEG_size(OV2640_160x120); break;
      case 0x01: myCAM.OV2640_set_JPEG_size(OV2640_176x144); break;
      case 0x02: myCAM.OV2640_set_JPEG_size(OV2640_320x240); break;
      case 0x03: myCAM.OV2640_set_JPEG_size(OV2640_352x288); break;
      case 0x04: myCAM.OV2640_set_JPEG_size(OV2640_640x480); break;
      case 0x05: myCAM.OV2640_set_JPEG_size(OV2640_800x600); break;
      case 0x06: myCAM.OV2640_set_JPEG_size(OV2640_1024x768); break;
      case 0x07: myCAM.OV2640_set_JPEG_size(OV2640_1280x1024); break;
      case 0x08: myCAM.OV2640_set_JPEG_size(OV2640_1600x1200); break;
      case 0x99:{
        deployed = true;
        autoCaptureEnabled = true;
        cameraBusy = false;   // force first capture
        velX = velY = velZ = 0;
        stationaryStart = 0;
        lastVelTime = millis();
        break;
      }

      case 0x98:{
        deployed = false;
        autoCaptureEnabled = false;
        break;
      }
    }
  }
}

void calibrateGyro() {
  gBiasX = gBiasY = gBiasZ = 0;

  for (int i = 0; i < 1000; i++) {
    gBiasX += imu.readFloatGyroX();
    gBiasY += imu.readFloatGyroY();
    gBiasZ += imu.readFloatGyroZ();
    delay(2);
  }

  gBiasX /= 1000.0;
  gBiasY /= 1000.0;
  gBiasZ /= 1000.0;
}

void getGyroRates(float &gx, float &gy, float &gz, float &dt) {
  unsigned long now = millis();
  dt = (now - lastIMUTime) / 1000.0;
  lastIMUTime = now;

  gx = imu.readFloatGyroX() - gBiasX;
  gy = imu.readFloatGyroY() - gBiasY;
  gz = imu.readFloatGyroZ() - gBiasZ;
}

void updateGyroAngles() {
  float gx, gy, gz, dt;
  getGyroRates(gx, gy, gz, dt);

  if (dt <= 0 || dt > 0.5) return; // safety

  angleX += gx * dt;
  angleY += gy * dt;
  angleZ += gz * dt;
}

void captureJPEG() {
  if (!deployed) return;

  jpegSending = true;
  digitalWrite(LED_Pin, HIGH);   // LED ON only here

  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();

  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  digitalWrite(LED_Pin, LOW);  
  uint32_t length = myCAM.read_fifo_length();
  if (!length || length >= MAX_FIFO_SIZE) {
    myCAM.clear_fifo_flag();
    digitalWrite(LED_Pin, LOW);
    jpegSending = false;
    return;
  }

  digitalWrite(CAM_CS_PIN, LOW);
  myCAM.set_fifo_burst();

  bool started = false;
  uint8_t prev = 0, cur;

  while (length--) {
    cur = SPI.transfer(0x00);

    if (!started) {
      if (prev == 0xFF && cur == 0xD8) {
        started = true;
        Serial.write(prev);
        Serial.write(cur);
      }
    } else {
      Serial.write(cur);
      if (prev == 0xFF && cur == 0xD9) break;
    }

    prev = cur;
    delayMicroseconds(15);
  }

  digitalWrite(CAM_CS_PIN, HIGH);
  myCAM.clear_fifo_flag();
  jpegSending = false;
}

void detectImpact(float ax, float ay, float az) {
  unsigned long now = millis();

  if (lastVelTime == 0) {
    lastVelTime = now;
    return;
  }

  float dt = (now - lastVelTime) / 1000.0f;
  lastVelTime = now;

  if (dt <= 0 || dt > 0.2f) return;

  // --- Gravity estimation (LPF) ---
  gLPX = LPF_ALPHA * gLPX + (1.0f - LPF_ALPHA) * ax;
  gLPY = LPF_ALPHA * gLPY + (1.0f - LPF_ALPHA) * ay;
  gLPZ = LPF_ALPHA * gLPZ + (1.0f - LPF_ALPHA) * az;

  // --- Linear acceleration (remove gravity) ---
  float lax = (ax - gLPX) * 9.81f;
  float lay = (ay - gLPY) * 9.81f;
  float laz = (az - gLPZ) * 9.81f;

  // --- Integrate to velocity ---
  velX += lax * dt;
  velY += lay * dt;
  velZ += laz * dt;

  // --- Drift damping ---
  velX *= VEL_DAMPING;
  velY *= VEL_DAMPING;
  velZ *= VEL_DAMPING;

  float velMag = sqrt(velX*velX + velY*velY + velZ*velZ);

  // --- Stationary detection ---
  if (velMag < STATIONARY_VEL_MPS) {
    if (stationaryStart == 0)
      stationaryStart = now;

    if ((now - stationaryStart) > STATIONARY_TIME_MS &&
        (now - lastImpactTime) > IMPACT_COOLDOWN_MS) {

      lastImpactTime = now;
      impactPending = true;

      // Reset velocity to avoid re-trigger
      velX = velY = velZ = 0;
      stationaryStart = 0;
    }
  } else {
    stationaryStart = 0;
  }
}


