#include <SPI.h>
#include <LoRa.h>



#define LoRa_CS 5
#define LoRa_MOSI 23
#define LoRa_MISO 19
#define LoRa_SCK 18

void setup() {
  Serial.begin(9600);
  delay(10);
  Serial.println();
  while (!Serial);

  Serial.println("LoRa Receiver");
  //SPI.begin(LoRa_SCK,LoRa_MISO,LoRa_MOSI, LoRa_CS);
  SPI.begin();
  //Serial.println("LoRa Receiver 2");
  
  // Configurar pines de LoRa
  
  //Serial.println("LoRa Receiver 3");
  //LoRa.setSyncWord(1);
  LoRa.setPins(LoRa_CS);
  if (!LoRa.begin(433E6)) {
    //Serial.println("LoRa Receiver 4");
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}