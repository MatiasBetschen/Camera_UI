#include <SPI.h>
#include <LoRa.h>

#define LORA_CS_PIN  9
#define LORA_RST_PIN 7
#define LORA_DIO0    6

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver");
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Init Sucessfull");
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