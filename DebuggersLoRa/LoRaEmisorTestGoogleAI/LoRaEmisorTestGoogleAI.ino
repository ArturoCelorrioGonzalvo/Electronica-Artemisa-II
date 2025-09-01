#include <SPI.h>
#include <RadioLib.h>

// --- PINES DE CONTROL DEL MÓDULO LORA ---
#define LORA_CS_PIN      10  // Pin para Chip Select (CS) o Slave Select (NSS)
#define LORA_IRQ_PIN     8   // Pin para la interrupción (DIO0). ¡Debe ser un pin con capacidad de interrupción!
#define LORA_RST_PIN     7   // Pin para el Reset.

// --- PINES PERSONALIZADOS PARA EL BUS SPI ---
#define LORA_SCK_PIN     13  // Serial Clock
#define LORA_MISO_PIN    12  // Master In Slave Out
#define LORA_MOSI_PIN    11  // Master Out Slave In

// Inicializa el objeto de RadioLib con los pines de CONTROL
// SX1276 (CS, IRQ/DIO0, RST, GPIO/DIO1)
SX1276 radio = new Module(LORA_CS_PIN, LORA_IRQ_PIN, LORA_RST_PIN);

// Estructura de datos de telemetría
struct TelemetryData {
  float altitude;
  float velocity;
  int gps_sats;
  byte flight_state;
};

TelemetryData packet;
int counter = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Emisor LoRa para Cohete (con pines SPI personalizados)");

  // --- PASO 1: INICIALIZAR EL BUS SPI ---
  // Se le dice al sistema qué pines físicos usar para SCK, MISO y MOSI.
  // El último parámetro (CS) se suele ignorar aquí porque la librería lo gestiona.
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN);

  // --- PASO 2: INICIALIZAR EL MÓDULO LORA ---
  // RadioLib ahora usará el bus SPI que acabamos de configurar.
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Fallo al iniciar, código de error: "));
    Serial.println(state);
    while (true);
  }

  // Configura los parámetros para largo alcance
  radio.setSpreadingFactor(11);
  radio.setBandwidth(125.0);
  radio.setCodingRate(8);
  radio.setSyncWord(0xAB);
  radio.setOutputPower(17);
  
  Serial.println("Emisor listo.");
}

void loop() {
  packet.altitude = 150.0;
  packet.velocity = 50.0;
  packet.gps_sats = 8;
  packet.flight_state = 3;

  //Serial.println(F("Enviando paquete... "));
  
  int state = radio.transmit((byte*)&packet, sizeof(packet));

  /*
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("¡Éxito!"));
  } else {
    Serial.print(F("Fallo al enviar, código de error: "));
    Serial.println(state);
  }
  */
  
  //counter++;
  //delay(1500);
}