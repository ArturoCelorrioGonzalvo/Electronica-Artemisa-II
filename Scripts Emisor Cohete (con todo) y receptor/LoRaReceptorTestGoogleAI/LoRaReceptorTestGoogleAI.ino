#include <SPI.h>
#include <RadioLib.h>

// --- PINES DE CONTROL DEL MÓDULO LORA ---
// Asegúrate de que coincidan con tu cableado
#define LoRa_CS      10  // Pin para Chip Select (CS). Puedes usar el que quieras.
#define LoRa_DI0     16   // Pin para la interrupción (DIO0).
#define LoRa_RST     15   // Pin para el Reset.

// Objeto de RadioLib
SX1276 radio = new Module(LoRa_CS, LoRa_DI0, LoRa_RST);

// Estructura de datos (idéntica a la del emisor)
struct TelemetryData {
 // uint8_t testCero;
  float hdop;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  double latitude;
  double longitude;
  float speed_kmph;
  float altitude_gps;
  float pressure_hpa;
  float altitude_bar;
  float temperature;
  float acc_x;
  float acc_y;
  float acc_z;
  uint8_t tamanoPaquete;
};

TelemetryData packet;

// --- LA MAGIA DE LAS INTERRUPCIONES ---
// Esta es la "bandera" que la interrupción levantará.
// "volatile" le dice al compilador que esta variable puede cambiar en cualquier momento.
volatile bool packetReceived = false;

// Esta es la Rutina de Servicio de Interrupción (ISR).
// Debe ser lo más rápida posible. ¡No uses Serial.print() aquí dentro!
// IRAM_ATTR es crucial en ESP32 para que la función se guarde en RAM y sea más rápida.
void IRAM_ATTR onReceive() {
  // Un paquete ha llegado, levantamos la bandera.
  packetReceived = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Receptor LoRa con Interrupciones - Esperando paquetes...");

  // Inicializar SPI con pines por defecto del ESP32-S3
  SPI.begin();

  // Inicializar LoRa
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Fallo al iniciar, código de error: "));
    Serial.println(state);
    while (true);
  }

  // Configurar parámetros (idénticos al emisor)
  radio.setSpreadingFactor(9);
  radio.setBandwidth(125.0);
  radio.setCodingRate(6);
  radio.setSyncWord(0xAB);
  
  // --- CONFIGURACIÓN DE LA INTERRUPCIÓN ---
  // 1. Configurar el pin DI0 como entrada
  pinMode(LoRa_DI0, INPUT);
  // 2. "Enganchar" nuestra función 'onReceive' a la interrupción del pin.
  // Se disparará cuando el pin suba de LOW a HIGH (RISING).
  attachInterrupt(digitalPinToInterrupt(LoRa_DI0), onReceive, RISING);

  // Ponemos el LoRa en modo de recepción. Ahora esperará a que llegue un paquete
  // y activará la interrupción por su cuenta.
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Fallo al iniciar la recepción, código de error: "));
    Serial.println(state);
    while (true);
  }
}

void loop() {
  // El loop principal ahora solo hace una cosa:
  // comprobar si la bandera 'packetReceived' ha sido levantada por la interrupción.
  if (packetReceived) {
    // Si la bandera está levantada, significa que tenemos un paquete.
    
    // Leemos los datos del paquete.
    int state = radio.readData((byte*)&packet, sizeof(packet));

    if (state == RADIOLIB_ERR_NONE) {
      // Paquete leído con éxito. Lo imprimimos.
      Serial.println();
      Serial.println(F("--- Paquete Recibido (vía Interrupción) ---"));
      Serial.print(F("  HDOP: ")); Serial.println(packet.hdop);
      Serial.print(F("  Hora: ")); Serial.print(packet.hour); Serial.print(":"); Serial.print(packet.minute ); Serial.print(":"); Serial.println(packet.second);
      Serial.print(F("  Latitud: ")); Serial.println(packet.latitude);
      Serial.print(F("  Longitud: ")); Serial.println(packet.longitude);
      Serial.print(F("  Velocidad: ")); Serial.println(packet.speed_kmph);
      Serial.print(F("  Altura del GPS: ")); Serial.println(packet.altitude_gps);
      Serial.print(F("  Presión (hPa): ")); Serial.println(packet.pressure_hpa);
      Serial.print(F("  Altura del barómetro: ")); Serial.println(packet.altitude_bar);
      Serial.print(F("  Temperatura: ")); Serial.println(packet.temperature);
      Serial.print(F("  Aceleración eje X: ")); Serial.println(packet.acc_x);
      Serial.print(F("  Aceleración eje Y: ")); Serial.println(packet.acc_y);
      Serial.print(F("  Aceleración eje Z: ")); Serial.println(packet.acc_z);
      //Serial.print(F("  Tamaño paquete: ")); Serial.println(sizeof(packet));
      Serial.print(F("  RSSI: ")); Serial.print(radio.getRSSI()); Serial.println(F(" dBm"));
      Serial.print(F("  SNR: ")); Serial.print(radio.getSNR()); Serial.println(F(" dB"));
      Serial.println(F("---------------------------------------------"));
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println(F("Error de CRC, paquete corrupto descartado."));
    }

    // --- PASOS CRUCIALES DE REINICIO ---
    // 1. Bajamos la bandera para no procesar este paquete de nuevo.
    packetReceived = false;

    // 2. Volvemos a poner el LoRa en modo de recepción para esperar el SIGUIENTE paquete.
    radio.startReceive();
  }
  // Si no hay interrupción, el loop no hace absolutamente nada.
  // Esto es muchísimo más eficiente.
}