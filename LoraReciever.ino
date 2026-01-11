#include <LoRa.h> // Include LoRa library

#define LORA_CS_PIN  6
#define LORA_RST_PIN 5
#define LORA_DIO0    3

void setup() {
  Serial.begin(115200);
  delay(1000);

  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0);
  // Initialize LoRa
  if (!LoRa.begin(433E6)) { // Set frequency to 433 MHz
    Serial.println("LoRa init failed");
    while (1);
  }
  Serial.println("LoRa Receiver initialized");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Received a packet
    Serial.print("Received packet: ");

    while (LoRa.available()) {
      String received = LoRa.readString();
      Serial.print(received);
    }

    Serial.println();
  }
}
