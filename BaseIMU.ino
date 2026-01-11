#include <Wire.h>
#include <SparkFunLSM6DS3.h>
#include <LoRa.h> // Include LoRa library


#define LORA_CS_PIN  6
#define LORA_RST_PIN 5
#define LORA_DIO0    3

LSM6DS3 imu(I2C_MODE, 0x6B);

float gBiasX, gBiasY, gBiasZ;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  if (imu.begin() != 0) {
    Serial.println("IMU init failed");
    while (1);
  }

  Serial.println("Calibrating gyro...");
  calibrateGyro();
  Serial.println("Done");

   LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0);
  // Initialize LoRa
  if (!LoRa.begin(433E6)) { // Set frequency to 433 MHz
    Serial.println("LoRa init failed");
    while (1);
  }
  Serial.println("LoRa init succeeded");
}

void calibrateGyro() {
  gBiasX = gBiasY = gBiasZ = 0;

  for (int i = 0; i < 1000; i++) {
    gBiasX += imu.readFloatGyroX();
    gBiasY += imu.readFloatGyroY();
    gBiasZ += imu.readFloatGyroZ();
    delay(2);
  }

  gBiasX /= 1000;
  gBiasY /= 1000;
  gBiasZ /= 1000;
}

void loop() {
  float gx = imu.readFloatGyroX() - gBiasX;
  float gy = imu.readFloatGyroY() - gBiasY;
  float gz = imu.readFloatGyroZ() - gBiasZ;

  // Send gyro data over LoRa
  LoRa.beginPacket();
  LoRa.print("Gyro [deg/s]: ");
  LoRa.print(gx, 3); LoRa.print(", ");
  LoRa.print(gy, 3); LoRa.print(", ");
  LoRa.println(gz, 3);
  LoRa.endPacket();

  // Debug output
  Serial.print("Gyro [deg/s]: ");
  Serial.print(gx, 3); Serial.print(", ");
  Serial.print(gy, 3); Serial.print(", ");
  Serial.println(gz, 3);

  delay(50);
}
