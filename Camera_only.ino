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
#define CAM_CS_PIN   10

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

  Serial.println("System starting...");

  // SPI
  SPI.begin();

  // Chip select
  pinMode(CAM_CS_PIN, OUTPUT);
  digitalWrite(CAM_CS_PIN, HIGH);

  // --------------------
  // ArduCAM INIT
  // --------------------
  Serial.println("Initializing ArduCAM...");

  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  // SPI test
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  if (myCAM.read_reg(ARDUCHIP_TEST1) != 0x55) {
    Serial.println("ArduCAM SPI ERROR");
    while (1);
  }

  // Detect OV2640
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);

  if (vid != 0x26 || (pid != 0x41 && pid != 0x42)) {
    Serial.println("OV2640 NOT FOUND");
    while (1);
  }

  Serial.println("OV2640 detected");

  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_320x240);
  myCAM.clear_fifo_flag();
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

    case 0x20: // Enable night mode
      enableNightMode();
      break;

    case 0x21: // Set exposure
      if (Serial.available()) {
        uint8_t exposureValue = Serial.read();
        setExposure(exposureValue);
      }
      break;

    case 0x22: // Set gain
      if (Serial.available()) {
        uint8_t gainValue = Serial.read();
        setGain(gainValue);
      }
      break;
  }
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
  if (length == 0 || length >= MAX_FIFO_SIZE) {
    myCAM.clear_fifo_flag();
    return;
  }

  digitalWrite(CAM_CS_PIN, LOW);   // enable camera

  myCAM.set_fifo_burst();

  bool jpegStarted = false;
  uint8_t prev = 0, cur;

  while (length--) {
    cur = SPI.transfer(0x00);

    if (!jpegStarted) {
      if (prev == 0xFF && cur == 0xD8) {
        jpegStarted = true;
        Serial.write(0xFF);
        Serial.write(0xD8);
      }
    } else {
      Serial.write(cur);
      if (prev == 0xFF && cur == 0xD9) break;
    }

    prev = cur;
    delayMicroseconds(15); // REQUIRED for SAMD USB
  }

  digitalWrite(CAM_CS_PIN, HIGH);
  myCAM.clear_fifo_flag();
}

// --------------------
// ENABLE NIGHT MODE
// --------------------
void enableNightMode() {
  myCAM.wrSensorReg8_8(0xFF, 0x01); // Select sensor register bank
  myCAM.wrSensorReg8_8(0x3B, 0x0A); // Enable night mode
  Serial.println("Night mode enabled");
}

// --------------------
// SET EXPOSURE
// --------------------
void setExposure(uint8_t value) {
  myCAM.wrSensorReg8_8(0xFF, 0x01); // Select sensor register bank
  myCAM.wrSensorReg8_8(0x10, value); // Set exposure to the provided value
  Serial.print("Exposure set to: 0x");
  Serial.println(value, HEX);
}

// --------------------
// SET GAIN
// --------------------
void setGain(uint8_t value) {
  myCAM.wrSensorReg8_8(0xFF, 0x01); // Select sensor register bank
  myCAM.wrSensorReg8_8(0x00, value); // Set gain to the provided value
  Serial.print("Gain set to: 0x");
  Serial.println(value, HEX);
}