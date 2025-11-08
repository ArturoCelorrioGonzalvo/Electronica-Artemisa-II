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
    case TEST:              return "TEST";
    case ON_PAD:            return "ON PAD";
    case ASCENDING:         return "ASCENDING";
    case APOGEE_DEPLOYMENT: return "DROGUE DEPLOY";
    case MAIN_DEPLOYMENT:   return "MAIN DEPLOY";
    case LANDED:            return "LANDED";
    default:                return "DEBUG";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  

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
  Serial.println("Receptor LoRa Activado");
}

void loop() {
  if (packetReceived) {
    // Bajamos la bandera inmediatamente
    packetReceived = false;
    
    // --- LECTURA ACTUALIZADA ---
    // Leemos los datos en nuestro nuevo struct 'loraPacket'
    int state = radio.readData((byte*)&loraPacket, sizeof(loraPacket));

    if (state == RADIOLIB_ERR_NONE) {
    // --- NUEVO FORMATO DE SALIDA PARA PYTHON ---
    // Enviamos una sola línea con todos los datos, fácil de parsear.
    // Ejemplo: STATE:3,ALT:450.21,SPD:320.50,ROLL:5.2,PITCH:-2.1,YAW:120.3,LAT:40.1234,LON:-3.4321
    
    Serial.print("STATE:");  Serial.print(loraPacket.flight_state);
    Serial.print(",ALT:");   Serial.print(loraPacket.altitude_bar, 2);
    Serial.print(",SPD:");   Serial.print(loraPacket.speed_kmph, 2);
    Serial.print(",ACC_X:"); Serial.print(loraPacket.acc_x, 2);
    Serial.print(",ROLL:");  Serial.print(loraPacket.roll, 2);
    Serial.print(",PITCH:"); Serial.print(loraPacket.pitch, 2);
    Serial.print(",YAW:");   Serial.print(loraPacket.yaw, 2);
    Serial.print(",LAT:");   Serial.print(loraPacket.latitude, 6);
    Serial.print(",LON:");   Serial.print(loraPacket.longitude, 6);
    Serial.println(); // Importante: termina la línea para que Python sepa que el paquete ha terminado

  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("CRC_ERROR"); // Enviamos un mensaje de error simple
  }

  // Volvemos a poner el LoRa en modo de recepción
  radio.startReceive();
  }
}