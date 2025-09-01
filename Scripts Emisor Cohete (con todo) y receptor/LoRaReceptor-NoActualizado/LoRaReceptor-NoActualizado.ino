#include <SPI.h>
#include <LoRa.h>
//#include <SPIFFS.h>

// Definir pines
#define LoRa_CS 5
#define LoRa_MOSI 20
#define LoRa_MISO 21
#define LoRa_SCK 47


//#define LoRa_DIO0 2  

/*
void guardarDatosEnSPIFFS(String datos) {
  //Serial.println("💾 Guardando datos en SPIFFS...");

  File archivo = SPIFFS.open("/datosLoRa.txt", FILE_APPEND);
  if (!archivo) {
    //Serial.println("❌ Error al abrir archivo SPIFFS!");
    return;
  }

  archivo.println(datos);
  archivo.close();
  
  //Serial.println("✅ Datos guardados correctamente en memoria interna!");
}
*/


void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("Test");
  
  Serial.println("Inicializando ESP32-S3");
  // Inicializar SPIFFS
  /*
  if (!SPIFFS.begin(true)) {  
    Serial.println("❌ Error al iniciar SPIFFS");
    while (true);
  }
  Serial.println("✅ SPIFFS iniciado correctamente!");
  */
  
  //SPI.begin(LoRa_SCK,LoRa_MISO,LoRa_MOSI, LoRa_CS);
  SPI.begin();
  // Configurar pines de LoRa
  LoRa.setPins(LoRa_CS);
  
  // Inicializar LoRa en 433 MHz
  
  if (!LoRa.begin(433E6)) { 
    Serial.println("❌ Error al iniciar LoRa");
    while (1);
  }
  
  // Ajustar parámetros para mejorar recepción  
  /*
  LoRa.setSpreadingFactor(7);  
  LoRa.setSignalBandwidth(125E3);  
  LoRa.setCodingRate4(5);
  */  

  Serial.println("✅ LoRa iniciado correctamente! Esperando datos...");
  
}


void loop() {
  int packetSize = LoRa.parsePacket();
  
  if (packetSize != 0) {
    Serial.println("📡 Paquete recibido!");
    int chars = 0;
    String mensaje = "";
    
    while (LoRa.available() && chars < packetSize) {
      mensaje += (char)LoRa.read();
      chars++;
    }
    
    Serial.print("Mensaje recibido: ");
    Serial.println(mensaje);
    
    // Guardar mensaje en SPIFFS
    //guardarDatosEnSPIFFS(mensaje);

    // Enviar confirmación de recepción al emisor
    /*Serial.println("✅ Enviando confirmación al emisor...");
    LoRa.beginPacket();
    LoRa.print("OK");
    LoRa.endPacket();
    */
  } 
  else {
    Serial.println("⏳ No hay paquetes disponibles. Intentando recibir...");
  }

  delay(100);
}


