#include <Wire.h>
#include <SPI.h>
#include <ArduCAM.h>
#include "memorysaver.h"

#if !(defined OV2640_MINI_2MP)
#error Please enable OV2640_MINI_2MP in memorysaver.h
#endif

// --------------------
// PIN DEFINITIONS
// --------------------
#define CAM_CS_PIN 10

// --------------------
// CONSTANTS
// --------------------
#define LINE_TIME_US 30.0   // Approx. OV2640 line period (µs)
#define MAX_EXPOSURE_LINES 0xFFFFF  // 20-bit max

// --------------------
// OBJECTS
// --------------------
ArduCAM myCAM(OV2640, CAM_CS_PIN);

// --------------------
// SETUP
// --------------------
void setup() {
  uint8_t vid, pid;

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
  myCAM.OV2640_set_JPEG_size(OV2640_320x240);
  myCAM.clear_fifo_flag();

  Serial.println("Ready");
}

// --------------------
// LOOP
// --------------------
void loop() {
  if (!Serial.available()) return;

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

    case 0x10:
      captureJPEG();
      break;

    case 0x21: { // Set exposure time in ms (hex)
      while (Serial.available() < 2);
      uint16_t ms = (Serial.read() << 8) | Serial.read();
      setExposureMs(ms);
      break;
    }
    case 0x22: {
      while (!Serial.available());
      uint8_t gain = Serial.read();
      setGain(gain);
      break;
    }

  }
}

// --------------------
// DISABLE AUTO EXPOSURE + GAIN
// --------------------
void disableAutoExposureAndGain() {
  myCAM.wrSensorReg8_8(0xFF, 0x01);
  uint8_t com8;
  myCAM.rdSensorReg8_8(0x13, &com8);
  com8 &= ~0x05;   // Clear AEC (bit0) and AGC (bit2)
  myCAM.wrSensorReg8_8(0x13, com8);
}

// --------------------
// SET EXPOSURE IN MILLISECONDS
// --------------------
void setExposureMs(uint16_t ms) {
  disableAutoExposureAndGain();

  float exposure_us = ms * 1000.0;
  uint32_t exposure_lines = exposure_us / LINE_TIME_US;

  if (exposure_lines > MAX_EXPOSURE_LINES)
    exposure_lines = MAX_EXPOSURE_LINES;

  myCAM.wrSensorReg8_8(0xFF, 0x01);

  uint8_t high = (exposure_lines >> 12) & 0x0F;
  uint8_t mid  = (exposure_lines >> 4)  & 0xFF;
  uint8_t low  = (exposure_lines & 0x0F) << 4;

  myCAM.wrSensorReg8_8(0x04, high);
  myCAM.wrSensorReg8_8(0x10, mid);
  myCAM.wrSensorReg8_8(0x45, low);

  Serial.print("Exposure set: ");
  Serial.print(ms);
  Serial.println(" ms");
}

void setGain(uint8_t gain) {
  disableAutoExposureAndGain();

  myCAM.wrSensorReg8_8(0xFF, 0x01);
  myCAM.wrSensorReg8_8(0x00, gain); // OV2640 gain register

  Serial.print("Gain set: 0x");
  Serial.println(gain, HEX);
}


// --------------------
// CAPTURE JPEG
// --------------------
void captureJPEG() {
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();

  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));

  uint32_t length = myCAM.read_fifo_length();
  if (!length || length >= MAX_FIFO_SIZE) {
    myCAM.clear_fifo_flag();
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
}
