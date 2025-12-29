#include <Wire.h>
#include <SPI.h>
#include <ArduCAM.h>
#include "memorysaver.h"

#if !(defined OV2640_MINI_2MP)
#error Please enable OV2640_MINI_2MP in memorysaver.h
#endif

#define CS_PIN 9

ArduCAM myCAM(OV2640, CS_PIN);

void setup() {
  uint8_t vid, pid;

  Wire.begin();
  Serial.begin(115200);
  delay(2000);

  Serial.println("ArduCAM Mini 2MP Ready");

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();

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

void loop() {
  if (!Serial.available()) return;

  uint8_t cmd = Serial.read();

  // --------------------
  // RESOLUTION COMMANDS
  // --------------------
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

    // --------------------
    // CAPTURE COMMAND
    // --------------------
    case 0x10:
      captureJPEG();
      break;
  }
}

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

  myCAM.CS_LOW();
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
      if (prev == 0xFF && cur == 0xD9) {
        break;
      }
    }

    prev = cur;
    delayMicroseconds(15);  // VERY IMPORTANT for SAMD USB
  }

  myCAM.CS_HIGH();
  myCAM.clear_fifo_flag();
}

