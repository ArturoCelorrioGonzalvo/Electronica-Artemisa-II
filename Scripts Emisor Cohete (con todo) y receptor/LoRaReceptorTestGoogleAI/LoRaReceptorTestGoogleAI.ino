#include <SPI.h>
#include <RadioLib.h>

// --- PINES DE CONTROL DEL MÓDULO LORA ---
#define LoRa_CS      10
#define LoRa_DI0     16
#define LoRa_RST     15

// Objeto de RadioLib
SX1278 radio = new Module(LoRa_CS, LoRa_DI0, LoRa_RST);

// --- NUEVO: Enum de los estados de vuelo ---
// ¡CRÍTICO! Esta enumeración DEBE SER IDÉNTICA a la del emisor.
// --- MÁQUINA DE ESTADOS ACTUALIZADA ---
enum FlightState {
  DEBUG,
  TEST,
  ON_PAD,
  ASCENDING,
  APOGEE_DEPLOYMENT,
  MAIN_DEPLOYMENT,
  LANDED
};

// --- NUEVO: Struct optimizado para LoRa ---
// Esta estructura debe ser 100% idéntica a la que usa el emisor para enviar.
// Usamos __attribute__((packed)) como buena práctica para evitar problemas de alineación de memoria.
struct __attribute__((packed)) LoRaPacket {
  uint8_t flight_state;
  float altitude_bar;
  float speed_kmph;
  float roll;
  float pitch;
  float yaw;
  float acc_x; 
  float acc_y;
  float acc_z;
  float latitude;   
  float longitude; 
};

// --- Variable global actualizada al nuevo tipo ---
LoRaPacket loraPacket;

// --- INTERRUPCIÓN (sin cambios) ---
volatile bool packetReceived = false;
void IRAM_ATTR onReceive() {
  packetReceived = true;
}

// --- NUEVA FUNCIÓN AUXILIAR ---
// Convierte el número del estado de vuelo a un String legible.
const char* getStateString(uint8_t state) {
  switch (state) {
    case DEBUG:             return "DEBUG";
    case TEST:              return "TEST (Low Power)";
    case ON_PAD:            return "ON PAD (Ready)";
    case ASCENDING:         return "ASCENDING";
    case APOGEE_DEPLOYMENT: return "DROGUE DEPLOY";
    case MAIN_DEPLOYMENT:   return "MAIN DEPLOY";
    case LANDED:            return "LANDED (Recovery)";
    default:                return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Receptor LoRa (Modo Telemetría Optimizada) - Esperando paquetes...");

  SPI.begin();

  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Fallo al iniciar, código de error: "));
    Serial.println(state);
    while (true);
  }
  Serial.println("Inicio correcto");
  // --- PARÁMETROS IDÉNTICOS AL EMISOR (sin cambios) ---
  radio.setSpreadingFactor(9);
  radio.setBandwidth(125.0);
  radio.setCodingRate(6);
  radio.setSyncWord(0xAB);
  radio.setOutputPower(17);
  
  // --- CONFIGURACIÓN DE INTERRUPCIÓN (sin cambios) ---
  pinMode(LoRa_DI0, INPUT);
  attachInterrupt(digitalPinToInterrupt(LoRa_DI0), onReceive, RISING);

  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Fallo al iniciar la recepción, código de error: "));
    Serial.println(state);
    while (true);
  }
}

void loop() {
  if (packetReceived) {
    // Bajamos la bandera inmediatamente
    packetReceived = false;
    
    // --- LECTURA ACTUALIZADA ---
    // Leemos los datos en nuestro nuevo struct 'loraPacket'
    int state = radio.readData((byte*)&loraPacket, sizeof(loraPacket));
    Serial.println(state);
    Serial.println(sizeof(loraPacket));
    if (state == RADIOLIB_ERR_NONE) {
      // --- SECCIÓN DE IMPRESIÓN REESCRITA ---
      Serial.println();
      Serial.println(F("--- Paquete de Telemetría Recibido ---"));
      
      // Imprimimos los campos del nuevo LoRaPacket
      Serial.print(F("  Estado de Vuelo:     ")); Serial.println(getStateString(loraPacket.flight_state));
      Serial.print(F("  Altitud Barométrica: ")); Serial.print(loraPacket.altitude_bar, 2); Serial.println(F(" m"));
      Serial.print(F("  Velocidad GPS:       ")); Serial.print(loraPacket.speed_kmph, 2); Serial.println(F(" km/h"));
      Serial.print(F("  Roll:                ")); Serial.print(loraPacket.roll, 2); Serial.println(F(" deg"));
      Serial.print(F("  Pitch:               ")); Serial.print(loraPacket.pitch, 2); Serial.println(F(" deg"));
      Serial.print(F("  Yaw:                 ")); Serial.print(loraPacket.yaw, 2); Serial.println(F(" deg"));
      Serial.print(F("  Acc X:               ")); Serial.print(loraPacket.acc_x, 2); Serial.println(F(" m/s^2"));
      Serial.print(F("  Acc Y:               ")); Serial.print(loraPacket.acc_y, 2); Serial.println(F(" m/s^2"));
      Serial.print(F("  Acc Z:               ")); Serial.print(loraPacket.acc_z, 2); Serial.println(F(" m/s^2"));
      Serial.print(F("  Latitud:             ")); Serial.print(loraPacket.latitude, 2); Serial.println(F(" deg"));
      Serial.print(F("  Longitud:            ")); Serial.print(loraPacket.longitude, 2); Serial.println(F(" deg"));

      // La información de la señal sigue siendo muy útil
      Serial.print(F("  RSSI:                ")); Serial.print(radio.getRSSI()); Serial.println(F(" dBm"));
      Serial.print(F("  SNR:                 ")); Serial.print(radio.getSNR()); Serial.println(F(" dB"));
      Serial.println(F("--------------------------------------"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println(F("Error de CRC, paquete corrupto descartado."));
    }

    // Volvemos a poner el LoRa en modo de recepción
    radio.startReceive();
  }
}