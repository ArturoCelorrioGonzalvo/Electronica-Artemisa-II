#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <RadioLib.h>
#include <L3G.h>
#include <LSM303.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

// --- Definiciones Globales ---
#define P0 1013.25
#define borrarSD true
#define DEBUG_MODE true
#define TEST_MODE false
#define debugLoRa false

// --- Pines ---
#define SDA_PIN 5
#define SCL_PIN 4

#define MOSI 11
#define SCK 12
#define MISO 10
#define SD_CS 13

#define LoRa_CS 9
#define LoRa_RST RADIOLIB_NC
#define LoRa_DI0 8

#define PARACAIDAS 3
#define DROGUE 2

#define BUZZER 6

// --- Macros de Depuración ---
#if DEBUG_MODE
  #define D_PRINT(...)    Serial.print(__VA_ARGS__)
  #define D_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define D_PRINT(...)
  #define D_PRINTLN(...)
#endif

// --- Estructuras de Datos ---
struct TelemetryData {
  int fix;
  float hdop;
  uint8_t hour, minute, second;
  double latitude, longitude;
  float speed_kmph, altitude_gps;
  float pressure_hpa, altitude_bar, temperature;
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  float roll, pitch, yaw;
};

struct __attribute__((packed)) LoRaPacket {
  uint8_t flight_state;
  float altitude_bar, speed_kmph;
  float roll, pitch, yaw;
  float acc_x, acc_y, acc_z;
  float latitude, longitude;
};

// --- Máquina de Estados ---
enum FlightState {
  DEBUG, ON_PAD, ASCENDING, APOGEE_DEPLOYMENT, MAIN_DEPLOYMENT, LANDED
};
FlightState currentState = ON_PAD;

// --- Instancias de Sensores y Periféricos ---
L3G gyroscope;
LSM303 imu_lsm303;
TelemetryData flightDataPacket;
Adafruit_Mahony filter;
TinyGPSPlus gps;
Adafruit_BMP280 bmp280;
SX1278 radio = new Module(LoRa_CS, LoRa_DI0, LoRa_RST);

// --- Variables de Lógica de Vuelo ---
volatile bool loraTxDone = true;
const float LAUNCH_ACCELERATION_THRESHOLD = 35.0; // m/s^2
const float MAIN_DEPLOYMENT_ALTITUDE = 150.0;     // metros
float initialAltitude = 0.0;
float restingAccelerationY;
const int APOGEE_DETECTION_WINDOW = 30;
float altitudeReadings[APOGEE_DETECTION_WINDOW];
int readingIndex = 0;
bool altitudeBufferFull = false;

// --- Variables y Constantes para el Buzzer de Recuperación ---
const int BEEP_FREQUENCY = 2730;          // Frecuencia del pitido en Hz
const long BEEP_DURATION = 200;           // Duración del pitido en ms
const long BEEP_PAUSE = 150;              // Pausa entre pitidos en ms
const long SERIES_PAUSE = 2500;           // Pausa larga entre series de pitidos
unsigned long previousBuzzerMillis = 0;   // Para temporizar el patrón del buzzer
int buzzerState = 0;                      // Máquina de estados para el patrón del buzzer (0 a 5)

// --- Temporizador para el Loop de Alta Frecuencia ---
const long highFreqInterval = 25; // 25ms = 40Hz
long previousHighFreqRead = 0;

const char* getStateString(uint8_t state) {
  switch (state) {
    case DEBUG:             return "DEBUG";
    case ON_PAD:            return "ON PAD (Ready)";
    case ASCENDING:         return "ASCENDING";
    case APOGEE_DEPLOYMENT: return "DROGUE DEPLOY";
    case MAIN_DEPLOYMENT:   return "MAIN DEPLOY";
    case LANDED:            return "LANDED (Recovery)";
    default:                return "DEBUG";
  }
}

void IRAM_ATTR onLoRaDio0Interrupt() {
  loraTxDone = true;
}

void guardarDatosEnSD(const TelemetryData& packet) {
  digitalWrite(LoRa_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  File archivo = SD.open("/datos.csv", FILE_APPEND);
  if (!archivo) {
    D_PRINTLN("❌ Error al escribir en microSD!");
    return;
  }
  archivo.print(packet.hdop, 2); archivo.print(";");
  archivo.print(packet.hour); archivo.print(":");
  archivo.print(packet.minute); archivo.print(":");
  archivo.print(packet.second); archivo.print(";");
  archivo.print(packet.latitude, 6); archivo.print(";");
  archivo.print(packet.longitude, 6); archivo.print(";");
  archivo.print(packet.speed_kmph, 2); archivo.print(";");
  archivo.print(packet.altitude_gps, 2); archivo.print(";");
  archivo.print(packet.pressure_hpa, 2); archivo.print(";");
  archivo.print(packet.altitude_bar, 2); archivo.print(";");
  archivo.print(packet.temperature, 2); archivo.print(";");
  archivo.print(packet.acc_x, 4); archivo.print(";");
  archivo.print(packet.acc_y, 4); archivo.print(";");
  archivo.print(packet.acc_z, 4); archivo.print(";");
  archivo.print(packet.gyro_x, 4); archivo.print(";");
  archivo.print(packet.gyro_y, 4); archivo.print(";");
  archivo.print(packet.gyro_z, 4); archivo.print(";");
  archivo.print(packet.mag_x, 2); archivo.print(";");
  archivo.print(packet.mag_y, 2); archivo.print(";");
  archivo.print(packet.mag_z, 2); archivo.print(";");
  archivo.print(packet.roll, 2); archivo.print(";");
  archivo.print(packet.pitch, 2); archivo.print(";");
  archivo.println(packet.yaw, 2);
  archivo.close();
}

/**
 * @brief Comprueba si el archivo de datos existe en la SD. Si no,
 * lo crea y escribe la primera fila con las cabeceras para el CSV.
 */
void inicializarArchivoSD() {
  // Comprobamos si el archivo ya existe
  if (!SD.exists("/datos.csv")) {
    D_PRINTLN("El archivo /datos.csv no existe. Creando y escribiendo cabeceras...");
    
    // Abrimos el archivo en modo escritura (lo crea si no existe)
    File archivo = SD.open("/datos.csv", FILE_WRITE);
    if (archivo) {
      // Escribimos la fila de cabeceras. Asegúrate de que coincida con guardarDatosEnSD()
      archivo.println("HDOP;Timestamp (UTC+1);Latitude (deg);Longitude (deg);Speed (km/h);Altitude_GPS (m);Pressure (hPa);Altitude_Baro (m);Temperature (C);Accel_X (m/s^2);Accel_Y (m/s^2);Accel_Z (m/s^2);Gyro_X (raw);Gyro_Y (raw);Gyro_Z (raw);Mag_X (raw);Mag_Y (raw);Mag_Z (raw);Roll (deg);Pitch (deg);Yaw (deg)");
      archivo.close();
      D_PRINTLN("✔ Cabeceras escritas correctamente.");
    } else {
      D_PRINTLN("❌ Error al crear el archivo /datos.csv");
    }
  } else {
    D_PRINTLN("✔ El archivo /datos.csv ya existe. Se añadirán nuevos datos.");
  }
}

void formatSDCard() {
  #if borrarSD
    D_PRINTLN("🗑 Eliminando archivos en la microSD...");
    File root = SD.open("/");
    File entry;
    while(entry = root.openNextFile()) {
      String entryName = entry.name();
      entry.close();
      if (!entryName.equals("/System Volume Information")) { SD.remove(entryName); }
    }
    D_PRINTLN("✔ Formateo completado.");

  #endif
}

void enviarPaqueteLoRa(const void* data, size_t size) {
  digitalWrite(SD_CS, HIGH);
  digitalWrite(LoRa_CS, LOW);
  if (size > 255) return;
  int state = radio.startTransmit((uint8_t*)data, size);
  if (state != RADIOLIB_ERR_NONE) {
    D_PRINT("❌ Fallo al iniciar envío LoRa, código: "); D_PRINTLN(state);
  }
  //loraTxDone = true;
}

void enviarDatosPorLoRa(const char* datos) {
  D_PRINT("Enviando datos de debug por LoRa: "); D_PRINTLN(datos);
  enviarPaqueteLoRa(datos, strlen(datos) + 1);
}

static void smartDelay(long ms) {
    long start = millis();
    do {
        while (Serial2.available()) {
            gps.encode(Serial2.read());
            if(millis() - start < ms){
              break;
            }
        }
    } while (millis() - start < ms);
}

void handleSetupFailure(const bool (&fallo)[6]) {
  const char* errorMessages[] = { "Acelerómetro", "Giróscopo", "Magnetómetro", "Barómetro", "microSD", "LoRa" };
  D_PRINTLN("\n--- FALLO CRÍTICO EN EL ARRANQUE ---");
    while(true){
      for(int i = 0; i < 6; i++) {
      if (fallo[i]) { D_PRINT("❌ Error al iniciar "); D_PRINTLN(errorMessages[i]); }
      }
      D_PRINTLN("El sistema continuará. Revisa el código");
      delay(5000);
    }
}

void runStateMachine() {
  switch (currentState) {
    case DEBUG:
      break;
    case ON_PAD:
      restingAccelerationY = abs(flightDataPacket.acc_y);
      if (restingAccelerationY < 10.0) {
      D_PRINT("Calibración en rampa OK. Accel y: "); D_PRINTLN(restingAccelerationY);
      }
      if (abs(flightDataPacket.acc_y - restingAccelerationY) > LAUNCH_ACCELERATION_THRESHOLD) {
        D_PRINTLN("¡LANZAMIENTO! -> ASCENDING");
        currentState = ASCENDING;
      }
      break;
    case ASCENDING:
      altitudeReadings[readingIndex] = flightDataPacket.altitude_bar;
      readingIndex = (readingIndex + 1) % APOGEE_DETECTION_WINDOW;
      if (!altitudeBufferFull && readingIndex == 0) altitudeBufferFull = true;
      if (altitudeBufferFull) {
        bool descending = true;
        for (int i = 0; i < APOGEE_DETECTION_WINDOW; i++) {
          if (flightDataPacket.altitude_bar >= altitudeReadings[i]) { 
            descending = false; 
            break; 
          }
        }
        if (descending) {
          D_PRINTLN("¡APOGEO! -> APOGEE_DEPLOYMENT");
          currentState = APOGEE_DEPLOYMENT;
        }
      }
      break;
    case APOGEE_DEPLOYMENT:
      D_PRINTLN("Activando paracaídas piloto (DROGUE)...");
      digitalWrite(DROGUE, HIGH);
      delay(10000);
      digitalWrite(DROGUE, LOW);
      currentState = MAIN_DEPLOYMENT;
      break;
    case MAIN_DEPLOYMENT:
      if (flightDataPacket.altitude_bar <= MAIN_DEPLOYMENT_ALTITUDE) {
        D_PRINTLN("ALTITUD OK -> Desplegando paracaidas principal.");
        digitalWrite(PARACAIDAS, HIGH);
        delay(5000);
        digitalWrite(PARACAIDAS, LOW);
        currentState = LANDED;
      }
      break;
    case LANDED:
      // --- Lógica del Buzzer de Recuperación (no bloqueante) ---
      unsigned long currentMillis = millis();

      // Esta máquina de estados interna controla el patrón de pitidos.
      // Se ejecuta repetidamente pero solo actúa cuando ha pasado el tiempo necesario.
      switch (buzzerState) {
        case 0: // BEEP 1
          if (currentMillis - previousBuzzerMillis >= SERIES_PAUSE) {
            tone(BUZZER, BEEP_FREQUENCY); // Inicia el primer pitido
            previousBuzzerMillis = currentMillis;
            buzzerState = 1;
          }
          break;

        case 1: // Pausa 1
          if (currentMillis - previousBuzzerMillis >= BEEP_DURATION) {
            noTone(BUZZER); // Apaga el buzzer
            previousBuzzerMillis = currentMillis;
            buzzerState = 2;
          }
          break;

        case 2: // BEEP 2
          if (currentMillis - previousBuzzerMillis >= BEEP_PAUSE) {
            tone(BUZZER, BEEP_FREQUENCY); // Inicia el segundo pitido
            previousBuzzerMillis = currentMillis;
            buzzerState = 3;
          }
          break;

        case 3: // Pausa 2
          if (currentMillis - previousBuzzerMillis >= BEEP_DURATION) {
            noTone(BUZZER); // Apaga el buzzer
            previousBuzzerMillis = currentMillis;
            buzzerState = 4;
          }
          break;

        case 4: // BEEP 3
          if (currentMillis - previousBuzzerMillis >= BEEP_PAUSE) {
            tone(BUZZER, BEEP_FREQUENCY); // Inicia el tercer pitido
            previousBuzzerMillis = currentMillis;
            buzzerState = 5;
          }
          break;

        case 5: // Pausa larga antes de repetir
          if (currentMillis - previousBuzzerMillis >= BEEP_DURATION) {
            noTone(BUZZER); // Apaga el buzzer
            previousBuzzerMillis = currentMillis;
            buzzerState = 0; // Vuelve al estado inicial para esperar la pausa larga
          }
          break;
      }
      break;
    default: 
      D_PRINTLN("");
      break;
  }
}

void checkSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Elimina espacios en blanco o retornos de carro

    if (command.startsWith("SET_STATE=")) {
      String newStateStr = command.substring(10); // Extrae el nombre del estado
      D_PRINT("Comando serie recibido para cambiar estado a: "); D_PRINTLN(newStateStr);

      if (newStateStr.equalsIgnoreCase("PAD")) {
        currentState = ON_PAD;
        D_PRINTLN("Estado cambiado a ON_PAD");
      } else if (newStateStr.equalsIgnoreCase("ASCENDING")) {
        currentState = ASCENDING;
        D_PRINTLN("Estado cambiado a ASCENDING");
      } else if (newStateStr.equalsIgnoreCase("APOGEE")) {
        currentState = APOGEE_DEPLOYMENT;
        D_PRINTLN("Estado cambiado a APOGEE_DEPLOYMENT");
      } else if (newStateStr.equalsIgnoreCase("MAIN")) {
        currentState = MAIN_DEPLOYMENT;
        D_PRINTLN("Estado cambiado a MAIN_DEPLOYMENT");
      } else if (newStateStr.equalsIgnoreCase("LANDED")) {
        currentState = LANDED;
        D_PRINTLN("Estado cambiado a LANDED");
      } else {
        D_PRINTLN("Error: Estado no reconocido.");
      }
    }
  }
}

void handleTelemetry() {
  if (loraTxDone) {
    #if debugLoRa
      loraTxDone = false;
      enviarDatosPorLoRa("Prueba LoRa");
    #else
      loraTxDone = false;
      LoRaPacket loraPacket;
      loraPacket.flight_state = (uint8_t)currentState;
      loraPacket.altitude_bar = flightDataPacket.altitude_bar;
      loraPacket.speed_kmph = flightDataPacket.speed_kmph;
      loraPacket.roll = flightDataPacket.roll;
      loraPacket.pitch = flightDataPacket.pitch;
      loraPacket.yaw = flightDataPacket.yaw;
      loraPacket.acc_x = flightDataPacket.acc_x;
      loraPacket.acc_y = flightDataPacket.acc_y;
      loraPacket.acc_z = flightDataPacket.acc_z;
      loraPacket.latitude = flightDataPacket.latitude;
      loraPacket.longitude = flightDataPacket.longitude;
      int size = sizeof(loraPacket);
      //D_PRINTLN(size);
      enviarPaqueteLoRa(&loraPacket, size);
      //D_PRINTLN("Se ha enviado un paquete");
      //D_PRINT("Acc x:");
      //D_PRINTLN(flightDataPacket.acc_x);
      //D_PRINT("Acc y:");
      //D_PRINTLN(flightDataPacket.acc_y);
      //D_PRINT("Acc z:");
      //D_PRINTLN(flightDataPacket.acc_z);
      //delay(1000);
    #endif
  }
}

/**
 * @brief Inicializa todo el hardware: pines, buses, sensores y periféricos.
 * Configura los sensores para adquisición de alta frecuencia y actualiza el
 * array 'fallo' para reportar cualquier error durante el proceso.
 * @param fallo Array booleano para rastrear el estado de la inicialización.
 */
void initializeSystems(bool (&fallo)[6]) {
    pinMode(PARACAIDAS, OUTPUT); digitalWrite(PARACAIDAS, LOW);
    pinMode(DROGUE, OUTPUT); digitalWrite(DROGUE, LOW);
    pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);
    pinMode(LoRa_CS, OUTPUT); digitalWrite(LoRa_CS, HIGH);
    D_PRINTLN("Configuring Pins & Buses...");
    Wire.begin(SDA_PIN, SCL_PIN);
    SPI.begin(SCK, MISO, MOSI);
    
    //delay(1000);
    D_PRINTLN("Iniciando IMU");
    //D_PRINTLN("Initializing Sensors...");
    //delay(5000);
    
    if (!imu_lsm303.init(LSM303::device_DLHC, LSM303::sa0_auto)) {
    //if (!imu_lsm303.init()) {   
      D_PRINTLN("FALLO EN ACCEL Y MAG");
      fallo[0] = true; 
      fallo[2] = true; 
    }
    else {
        D_PRINTLN("Aquí el Accel y el Mag se han iniciado");
        imu_lsm303.enableDefault();
        D_PRINTLN("Aquí se usa el .enableDefault");
        imu_lsm303.writeAccReg(LSM303::CTRL_REG4_A, 0x38);
        //imu_lsm303.writeAccReg(0x21, 0x20); // Configura Accel a ±16g 0x20 es 32, que es +-16g
        //D_PRINTLN("Aquí escribimos en el registro 0x21");
    }
    
    //delay(3000);
    D_PRINTLN("Iniciando giróscopo");

    if (!gyroscope.init(L3G::device_4200D)) { 
      fallo[1] = true;
    }
    else {      
        gyroscope.enableDefault();
        D_PRINTLN("Giroscopo bien");
        // --- CORRECCIÓN PARA EL GIROSCOPIO ---
        // Escribimos directamente en el registro de control 4 (CTRL_REG4).
        // El valor 0x20 establece la escala de 2000 dps (grados por segundo).
        gyroscope.writeReg(L3G::CTRL_REG4, 0x20);
    }
    
    
    //delay(1000);

    D_PRINTLN("Iniciando barómetro");
    if (!bmp280.begin()) {
        fallo[3] = true;
    } else {
        // Configura el barómetro para reducir ruido (hacer "media")
        bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Modo de operación */
                         Adafruit_BMP280::SAMPLING_X4,      /* Sobremuestreo de Temp. */ //Podríamos llegar a x16, pero no nos importa demasiado
                         Adafruit_BMP280::SAMPLING_X16,     /* Sobremuestreo de Presión */
                         Adafruit_BMP280::FILTER_X16,       /* Coeficiente del Filtro */
                         Adafruit_BMP280::STANDBY_MS_1);    /* Tiempo de espera */
    }
    
    //delay(1000);

    
    D_PRINTLN("Iniciando SD");
    if (!SD.begin(SD_CS, SPI, 4000000, "/sd", 5, true)) { fallo[4] = true; }

    //delay(1000);

    D_PRINTLN("Iniciando LoRa");
    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) { 
      fallo[5] = true;
      D_PRINTLN(state);
    }
    else {
        radio.setSpreadingFactor(9);
        radio.setBandwidth(125.0);
        radio.setCodingRate(6);
        radio.setSyncWord(0xAB);
        radio.setOutputPower(17);//Ya tenemos la potencia máxima en EU, no se puede más de 17dBm
        pinMode(LoRa_DI0, INPUT);
        attachInterrupt(digitalPinToInterrupt(LoRa_DI0), onLoRaDio0Interrupt, RISING);
    }

    //delay(1000);
}

void readHighFrequencySensors() {
    flightDataPacket.pressure_hpa = bmp280.readPressure() / 100.0;
    flightDataPacket.altitude_bar = bmp280.readAltitude(P0); 
    flightDataPacket.temperature = bmp280.readTemperature();
    
    imu_lsm303.read();
    gyroscope.read();

    const float G_TO_MS2 = 9.80665;
    const float ACCEL_SENSITIVITY_G_PER_LSB = 0.000732;

    // Simplemente multiplicamos el valor crudo por la sensibilidad.
    flightDataPacket.acc_x = (float)imu_lsm303.a.x * ACCEL_SENSITIVITY_G_PER_LSB * G_TO_MS2;
    flightDataPacket.acc_y = (float)imu_lsm303.a.y * ACCEL_SENSITIVITY_G_PER_LSB * G_TO_MS2;
    flightDataPacket.acc_z = (float)imu_lsm303.a.z * ACCEL_SENSITIVITY_G_PER_LSB * G_TO_MS2;
    
    flightDataPacket.mag_x = imu_lsm303.m.x;
    flightDataPacket.mag_y = imu_lsm303.m.y;
    flightDataPacket.mag_z = imu_lsm303.m.z;
    
    flightDataPacket.gyro_x = gyroscope.g.x;
    flightDataPacket.gyro_y = gyroscope.g.y;
    flightDataPacket.gyro_z = gyroscope.g.z;
}

void setup() {
  Serial.begin(115220);
  Serial2.begin(9600, SERIAL_8N1, 43, 44);
  delay(2000);
  D_PRINTLN("\n--- INICIANDO AVIÓNICA ARTEMISA II ---");

  bool fallo[6] = {false};
  initializeSystems(fallo);
  
  bool anyFailure = false;
  for (int i = 0; i < 6; i++) { 
    if (fallo[i]) {
      anyFailure = true; 
      D_PRINTLN("HA HABIDO ALGÚN ERROR");
    }
    //D_PRINTLN("Buscando Errores");
  }
  if (anyFailure) handleSetupFailure(fallo);


  
  filter.begin((int)(1000.0/(float)highFreqInterval));
  initialAltitude = bmp280.readAltitude(P0);
  D_PRINT("Altitud inicial: "); D_PRINT(initialAltitude); D_PRINTLN(" m");
  
  for (int i = 0; i < APOGEE_DETECTION_WINDOW; i++) { altitudeReadings[i] = -1000.0; }
  formatSDCard();
  inicializarArchivoSD();
  D_PRINTLN("--- CONFIGURACIÓN FINALIZADA ---");


}

void loop() {
    #if(TEST_MODE)
        checkSerialCommand();
    #endif
    long currentTime = millis();
    if (currentTime - previousHighFreqRead >= highFreqInterval) {
      
        previousHighFreqRead = currentTime;

        // --- INICIO DEL CICLO ---
        readHighFrequencySensors();

        const float GYRO_SENSITIVITY_TO_DPS = 0.070; // Para L3G a ±2000dps
        const float DPS_TO_RADS = 0.0174533;
        float gyro_x_rads = flightDataPacket.gyro_x * GYRO_SENSITIVITY_TO_DPS * DPS_TO_RADS;
        float gyro_y_rads = flightDataPacket.gyro_y * GYRO_SENSITIVITY_TO_DPS * DPS_TO_RADS;
        float gyro_z_rads = flightDataPacket.gyro_z * GYRO_SENSITIVITY_TO_DPS * DPS_TO_RADS;

        filter.update(gyro_x_rads, gyro_y_rads, gyro_z_rads,
                      flightDataPacket.acc_x, flightDataPacket.acc_y, flightDataPacket.acc_z,
                      flightDataPacket.mag_x, flightDataPacket.mag_y, flightDataPacket.mag_z);
        
        flightDataPacket.roll = filter.getRoll();
        flightDataPacket.pitch = filter.getPitch();
        flightDataPacket.yaw = filter.getYaw();
        
        // Actualizar datos del GPS en el struct principal
        flightDataPacket.fix = gps.location.age();
        flightDataPacket.latitude = gps.location.lat();
        flightDataPacket.longitude = gps.location.lng();
        flightDataPacket.speed_kmph = gps.speed.kmph();
        flightDataPacket.hdop = gps.hdop.hdop();
        flightDataPacket.hour = (gps.time.hour() + 1) % 24;
        flightDataPacket.minute = gps.time.minute();
        flightDataPacket.second = gps.time.second();
        flightDataPacket.altitude_gps = gps.altitude.meters();

        D_PRINT(flightDataPacket.hour);
        D_PRINT(":");
        D_PRINT(flightDataPacket.minute);
        D_PRINT(":");
        D_PRINT(flightDataPacket.second);
        D_PRINT(";Lat");
        D_PRINT(flightDataPacket.latitude);
        D_PRINT(";Long");
        D_PRINT(flightDataPacket.longitude);

        //runStateMachine();
       //D_PRINTLN(getStateString(loraPacket.flight_state));

      
        //Quito la escritura en la SD porque tengo la lenta
        //guardarDatosEnSD(flightDataPacket);
        handleTelemetry();
    }
    //Serial.println(millis()-currentTime); //Serial.print("  Delay: "); Serial.println(currentTime + highFreqInterval - millis());
    int delayDeseado = currentTime + highFreqInterval - millis();
    delayDeseado = (delayDeseado < 1) ? 1:delayDeseado;
    smartDelay(delayDeseado);
    //runStateMachine();
    //D_PRINTLN(currentState);
    //delay(1000);
}