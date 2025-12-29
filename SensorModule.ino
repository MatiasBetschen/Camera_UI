#include <Wire.h>
#include <SPI.h>
#include <ArduCAM.h>

// -------- SAMD USB SERIAL FIX --------
#define Serial SerialUSB

// -------- CAMERA SELECTION --------
#if !(defined OV2640_MINI_2MP)
  #error Please enable OV2640_MINI_2MP in memorysaver.h
#endif

// -------- PIN CONFIG --------
const int CS = 9;   // CHANGE if your board uses a different CS

ArduCAM myCAM(OV2640, CS);

bool is_header = false;
uint8_t start_capture = 0;

void setup() {
  // USB Serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  delay(1000);

  Serial.println("ACK CMD ArduCAM Start! END");

  // I2C + SPI
  Wire.begin();
  SPI.begin();

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  // Reset ArduCAM
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  // Check SPI
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  uint8_t temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55) {
    Serial.println("ACK CMD SPI interface Error! END");
    while (1);
  }
  Serial.println("ACK CMD SPI interface OK. END");

  // Detect OV2640
  uint8_t vid, pid;
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);

  if (vid != 0x26 || (pid != 0x41 && pid != 0x42)) {
    Serial.println("ACK CMD Can't find OV2640 module! END");
    while (1);
  }
  Serial.println("ACK CMD OV2640 detected. END");

  // Camera init
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_640x480);
  delay(1000);

  myCAM.clear_fifo_flag();

  Serial.println("ACK CMD Ready. Send 0x10 to capture. END");
}

void loop() {
  if (Serial.available()) {
    uint8_t cmd = Serial.read();

    // -------- SINGLE IMAGE CAPTURE --------
    if (cmd == 0x10) {
      start_capture = 1;
      Serial.println("ACK CMD CAM start single shoot. END");
    }
  }

  if (start_capture) {
    start_capture = 0;

    myCAM.flush_fifo();
    myCAM.clear_fifo_flag();
    myCAM.start_capture();

    while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
      delay(1);
    }

    Serial.println("ACK CMD CAM Capture Done. END");
    read_fifo_burst();
    myCAM.clear_fifo_flag();
  }
}

// -------- READ JPEG FROM FIFO --------
void read_fifo_burst() {
  uint32_t length = myCAM.read_fifo_length();
  if (length == 0 || length >= MAX_FIFO_SIZE) {
    Serial.println("ACK CMD Invalid image size. END");
    return;
  }

  myCAM.CS_LOW();
  myCAM.set_fifo_burst();

  uint8_t temp = 0, temp_last = 0;
  bool is_header = false;

  while (length--) {
    temp_last = temp;
    temp = SPI.transfer(0x00);

    if (is_header) {
      Serial.write(temp);
    } else if (temp == 0xD8 && temp_last == 0xFF) {
      is_header = true;
      Serial.write(temp_last);
      Serial.write(temp);
    }

    if (temp == 0xD9 && temp_last == 0xFF) {
      break;
    }
  }

  myCAM.CS_HIGH();
}
