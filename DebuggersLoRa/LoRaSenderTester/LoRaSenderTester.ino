#include <SPI.h>
#include <LoRa.h>
//SPI LoRa
#define LoRa_CS 10 
#define LoRa_MOSI 11 
#define LoRa_MISO 12
#define LoRa_SCK 13 


int counter = 0;

void setup() {
  Serial.begin(115200);
  delay(10);
  while (!Serial);

  Serial.println("LoRa Sender");

  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_CS);
  LoRa.setPins(LoRa_CS);
  //LoRa.setSyncWord(1);
  
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("Hello number ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(3000);//Probar también con un mensaje parecido al que vamos a enviar desde el cohete
}
