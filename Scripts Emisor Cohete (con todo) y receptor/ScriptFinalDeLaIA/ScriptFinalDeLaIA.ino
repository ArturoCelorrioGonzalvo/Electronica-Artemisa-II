#include <SD.h>
#include <Wire.h>
#include <BMP280.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
//#include <LoRa.h> Uso la librería RadioLib
#include <RadioLib.h>
#include <Adafruit_LSM303_U.h>
//#include <Adafruit_L3GD20_U.h>//Librería del giróscopo
#include <L3G.h>
#include <Adafruit_Sensor_Calibration.h> // Puede que la necesites para el magnetómetro
#include <Adafruit_AHRS.h>             // <-- NUEVA LIBRERÍA DE FUSIÓN

//Sensores de SPI
//Lora, MicroSD

//Sensores I2C
//IMU, Barómetro

//GPS va por Rx/Tx

//Definiciones para debugging
#define debugGPS false
#define debugBar false
#define debugIMU false
#define verMensaje true

#define IMUConectada true

#define borrarSD true

#define debugResto debugGPS || debugBar || debugIMU
#define debugLoRa false
#define usarLoRa true

// Definir pines
#define P0  1013.25//presión a nivel del mar


#define SDA_PIN 5  
#define SCL_PIN 4   

//Pines SPI

//Generales
#define MOSI 11
#define SCK 13
#define MISO 12

//SD
#define SD_CS 3

//LoRa
#define LoRa_CS 10 
#define LoRa_RST 7
#define LoRa_DI0 8



/*
Pines barómetro de derecha a izquierda mirando desde el sensor
Vcc 3.3V
GND GND
SCL pin 4
SDA pin 5
CSB NC
SD0 GND
*/

/*
Pines tarjeta microSD de derecha a izquierda mirando desde la tarjeta 
Vcc 3.3V
CS pin 3 Antes era el 6
MOSI pin 11
CLK pin 13
MISO pin 12
GND GND 
*/

// --- LLAVE MAESTRA PARA DEBUG ---
// Ponlo en 'true' para activar los mensajes por Serial durante las pruebas.
// Ponlo en 'false' para compilar el código de vuelo final sin NINGÚN mensaje.
#define DEBUG_MODE true

// --- MACROS DE DEPURACIÓN ---
// Si DEBUG_MODE es true, estas macros usarán Serial.print.
// Si es false, el compilador las eliminará por completo.
#if DEBUG_MODE
  #define D_PRINT(...)    Serial.print(__VA_ARGS__)
  #define D_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define D_PRINT(...)
  #define D_PRINTLN(...)
#endif

// --- DEFINICIONES PARA EL DESPLIEGUE ---
#define DEPLOYMENT_PIN 2 // Este pin hay que elegirlo para el despliegue del paracaídas, de momento uso el pin 2 que lo tengo libre

//Configuración del acelerómetro
#if IMUConectada
  Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
  L3G gyroscope;
  Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
#endif

struct TelemetryData {
  
  int fix;
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
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  float roll, pitch, yaw;
};
//Cambiar a UTF-8?

// Creamos una instancia global de la estructura para llenarla en el loop
TelemetryData flightDataPacket;

Adafruit_Mahony filter;

//Meter acelerómetro
TinyGPSPlus gps;

// Configuración del Barómetro
BMP280 bmp280;


// --- NUEVO: INTERRUPCIÓN PARA EL EMISOR ---
// Esta bandera volátil nos dirá cuándo ha terminado la transmisión anterior.
// Se inicializa en 'true' para permitir el envío del primer paquete.
volatile bool transmissionFinished = true;

// Esta es la Rutina de Servicio de Interrupción (ISR).
// Se ejecuta automáticamente cuando el LoRa termina de enviar un paquete.
// Debe ser muy rápida. IRAM_ATTR es una optimización para ESP32.
void IRAM_ATTR onTxDone() {
  // La transmisión ha terminado, levantamos la bandera.
  transmissionFinished = true;
}

// Definimos la ESTRUCTURA para los datos de telemetría reales.
// Debe ser idéntica en el emisor y el receptor.


// Creamos la instancia del objeto de radio usando tus pines definidos
// La firma es (CS, IRQ/DIO0, RST)
SX1278 radio = new Module(LoRa_CS, LoRa_DI0, LoRa_RST);
//SX1278 radio = Module(LoRa_CS, LoRa_DI0, LoRa_RST);

char buffer[200];

//Variable para la medida inicial de la altura

float initialAltitude = 0.0;

// Máquina de estados con el modo TESTING
enum FlightState {
  TESTING,          // Para pruebas en tierra
  ON_PAD,           // Para el estado de espera en la rampa
  ASCENDING,
  APOGEE_DETECTED,
  RECOVERY
};

// --- CONFIGURACIÓN DEL MODO DE ARRANQUE ---
// Cambia este valor a ON_PAD para el vuelo real.
FlightState currentState = TESTING; 

// Umbrales y constantes
const float LAUNCH_ACCELERATION_THRESHOLD = 30.0; // m/s^2 (aprox. 3G)
const int APOGEE_DETECTION_WINDOW = 5;

// Buffer para detección de apogeo
float altitudeReadings[APOGEE_DETECTION_WINDOW];
int readingIndex = 0;
bool altitudeBufferFull = false;

// Temporizador para el modo de prueba
unsigned long lastTestPrint = 0;



// VERSIÓN 2: Para el struct de datos de vuelo
void guardarDatosEnSD(const TelemetryData& packet) {
  digitalWrite(LoRa_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  
  File archivo = SD.open("/datos.csv", FILE_APPEND);
  if (!archivo) {
    Serial.println("❌ Error al escribir en microSD!");
    return;
  }

  // --- CONSTRUCCIÓN DE LA LÍNEA DEL CSV, CAMPO POR CAMPO ---
  archivo.print(packet.hdop, 2); archivo.print(",");
  archivo.print(packet.hour); archivo.print(":");
  archivo.print(packet.minute); archivo.print(":");
  archivo.print(packet.second); archivo.print(",");
  archivo.print(packet.latitude, 6); archivo.print(",");
  archivo.print(packet.longitude, 6); archivo.print(",");
  archivo.print(packet.speed_kmph, 2); archivo.print(",");
  archivo.print(packet.altitude_gps, 2); archivo.print(",");
  archivo.print(packet.pressure_hpa, 2); archivo.print(",");
  archivo.print(packet.altitude_bar, 2); archivo.print(",");
  archivo.print(packet.temperature, 2); archivo.print(",");
  archivo.print(packet.acc_x, 4); archivo.print(",");
  archivo.print(packet.acc_y, 4); archivo.print(",");
  archivo.print(packet.acc_z, 4); archivo.print(",");
  archivo.print(packet.gyro_x, 4); archivo.print(",");
  archivo.print(packet.gyro_y, 4); archivo.print(",");
  archivo.print(packet.gyro_z, 4); archivo.print(",");
  archivo.print(packet.mag_x, 2); archivo.print(",");
  archivo.print(packet.mag_y, 2); archivo.print(",");
  archivo.print(packet.mag_z, 2); archivo.print(",");
  archivo.print(packet.roll, 2); archivo.print(",");
  archivo.print(packet.pitch, 2); archivo.print(",");
  archivo.println(packet.yaw, 2); 

  archivo.close();
}

// Versión bloqueante
/*
void enviarPaqueteLoRa(const void* data, size_t size) {
  // Gestionamos los pines Chip Select
  digitalWrite(SD_CS, HIGH);
  digitalWrite(LoRa_CS, LOW);

  // Advertencia si los datos son demasiado grandes para un solo paquete
  if (size > 255) {
    Serial.println("⚠️ ADVERTENCIA: Datos exceden tamaño máximo de LoRa (255 bytes).");
    return; // No se envía nada para evitar errores
  }

  // Usamos el método transmit de RadioLib
  int state = radio.transmit((uint8_t*)data, size);

  if (state == RADIOLIB_ERR_NONE) {
    // El paquete se empezó a enviar sin problemas
    Serial.println("✅ Paquete LoRa enviado.");
  } else {
    // Hubo un error
    Serial.print("❌ Fallo al enviar LoRa, código de error: ");
    Serial.println(state);
  }
}
*/

//Versión no bloqueante
void enviarPaqueteLoRa(const void* data, size_t size) {
  digitalWrite(SD_CS, HIGH);
  digitalWrite(LoRa_CS, LOW);

  if (size > 255) {
    Serial.println("⚠️ ADVERTENCIA: Datos exceden tamaño máximo de LoRa (255 bytes).");
    transmissionFinished = true; // Liberamos la bandera para permitir un nuevo intento.
    return;
  }

  // --- CAMBIO CLAVE: de transmit() a startTransmit() ---
  // Esta función devuelve el control al programa INMEDIATAMENTE.
  int state = radio.startTransmit((uint8_t*)data, size);

  if (state == RADIOLIB_ERR_NONE) {
    // El envío ha comenzado en segundo plano.
    Serial.println("✅ Transmisión LoRa iniciada...");
  } else {
    // Si hubo un error al iniciar, no habrá interrupción.
    // Debemos liberar la bandera manualmente para poder reintentar.
    Serial.print("❌ Fallo al iniciar envío LoRa, código de error: ");
    Serial.println(state);
    transmissionFinished = true;
  }
}

/* Versión antigua, fragmentaba un string de datos, ahora usamos casi siempre un struct de datos
void enviarDatosPorLoRa(String datos) {
  digitalWrite(SD_CS, HIGH); //Desactivamos SD
  digitalWrite(LoRa_CS, LOW); //Activamos LoRa

  int maxBytes = 250;  // Límite recomendado para evitar errores
  int totalFragmentos = (datos.length() / maxBytes) + 1;  // Calcular cantidad de fragmentos
  
  Serial.print("Datos: ");

  for (int i = 0; i < datos.length(); i += maxBytes) {  
    String fragmento = datos.substring(i, i + maxBytes);
    Serial.print(fragmento);
    LoRa.beginPacket();
    LoRa.print(fragmento);
    LoRa.endPacket();
  }
  Serial.println("");
  //Serial.println("✅ Todos los fragmentos enviados! Esperando confirmación del receptor...");

  // Esperar confirmación "OK" del receptor antes de enviar otro dato
  
  unsigned long tiempoInicio = millis();
  while (millis() - tiempoInicio < 1000) {  // Máximo 1s de espera
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String confirmacion = "";
      while (LoRa.available()) {
        confirmacion += (char)LoRa.read();
      }
      if (confirmacion == "OK") {
        Serial.println("✅ Confirmación recibida! Listo para el siguiente dato.");
        return;
      }
    }
    delay(10);
  }

  Serial.println("⚠ Tiempo de espera agotado. Reintentando...");
  
}
*/

// Mantenemos tu función original para los casos de debug.
// Ahora simplemente llama a la nueva función.
// La lógica de fragmentación se ha eliminado porque es compleja y un struct no la necesita. Para los strings de debug, asumimos que caben en un paquete.
void enviarDatosPorLoRa(String datos) {
  Serial.print("Enviando datos de debug por LoRa: ");
  Serial.println(datos);
  
  // Enviamos el contenido del String como un array de bytes.
  // datos.length() + 1 para incluir el carácter nulo de fin de cadena.
  enviarPaqueteLoRa(datos.c_str(), datos.length() + 1);
}

//Función para el GPS
static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (Serial2.available()) {
            gps.encode(Serial2.read());
        }
    } while (millis() - start < ms);
}

// --- NUEVAS FUNCIONES DE INICIALIZACIÓN ---

void setupPins() {
  D_PRINTLN("Configurando pines...");
  #if DEBUG_MODE
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
  #endif
  
  pinMode(DEPLOYMENT_PIN, OUTPUT);
  digitalWrite(DEPLOYMENT_PIN, LOW);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH); 
  #if usarLoRa 
    pinMode(LoRa_CS, OUTPUT); 
    digitalWrite(LoRa_CS, HIGH);
  #endif
}

void setupBuses() {
  D_PRINTLN("Inicializando buses de comunicación (I2C y SPI)...");
  Wire.begin(SDA_PIN, SCL_PIN);
  SPI.begin(SCK, MISO, MOSI);
}

// Pasamos el array 'fallo' por referencia para que esta función pueda modificarlo
void setupSensors(bool (&fallo)[6]) {
  #if IMUConectada
    D_PRINTLN("Test 1: Acelerómetro...");
    if(!accel.begin()){ fallo[0] = true; }
  
    D_PRINTLN("Test 2: Giróscopo...");
    if (!gyroscope.init()) {
      fallo[1] = true;
    } else {
      gyroscope.enableDefault();
    }
  
    D_PRINTLN("Test 3: Magnetómetro...");
    if(!mag.begin()){ fallo[2] = true; }
  #endif

  D_PRINTLN("Test 4: Barómetro...");
  if (bmp280.begin() != 0) {
    fallo[3] = true;
  }
}

void setupPeripherals(bool (&fallo)[6]) {
  D_PRINTLN("Test 5: Tarjeta SD...");
  if (!SD.begin(SD_CS)) {
    fallo[4] = true;
  }

  #if usarLoRa
      D_PRINTLN("Test 6: LoRa...");
      int state = radio.begin();
      if (state != RADIOLIB_ERR_NONE) { 
        fallo[5] = true;
      } else {
        radio.setSpreadingFactor(9);
        radio.setBandwidth(125.0);
        radio.setCodingRate(6);
        radio.setSyncWord(0xAB);
        radio.setOutputPower(17);
        pinMode(LoRa_DI0, INPUT);
        attachInterrupt(digitalPinToInterrupt(LoRa_DI0), onTxDone, RISING);
      }
  #endif
}

void handleSetupFailure(const bool (&fallo)[6]) {
  const char* errorMessages[] = {
    "❌ Error al iniciar Acelerómetro",
    "❌ Error al iniciar Giróscopo",
    "❌ Error al iniciar Magnetómetro",
    "❌ Error al iniciar Barómetro",
    "❌ Error al inicializar microSD",
    "❌ Error al iniciar LoRa"
  };

  // Este bucle se ejecutará para siempre si hay algún fallo
  while(true) {
    D_PRINTLN("\n--- FALLO CRÍTICO EN EL ARRANQUE ---");
    for(int i = 0; i < 6; i++) {
      if (fallo[i]) {
        D_PRINTLN(errorMessages[i]);
      }
    }
    D_PRINTLN("El sistema se detendrá. Reinicia la placa.");
    delay(5000);
  }
}

void formatSDCard() {
  #if borrarSD
    D_PRINTLN("🗑 Eliminando archivos en la microSD...");
    File root = SD.open("/");
    File entry = root.openNextFile();
    while(entry) {
      String entryName = entry.name();
      entry.close();
      if (!entryName.equals("/System Volume Information")) {
        if (SD.remove(entryName.c_str())) {
          D_PRINT("  ✅ Archivo eliminado: ");
          D_PRINTLN(entryName);
        } else {
          D_PRINT("  ❌ No se pudo eliminar: ");
          D_PRINTLN(entryName);
        }
      }
      entry = root.openNextFile();
    }
    D_PRINTLN("✔ Proceso de formateo completado.");
  #endif
}

// --- NUEVAS FUNCIONES DEL CICLO PRINCIPAL (VERSIÓN UNIFICADA CON STRUCT) ---

void updateSensorData() {
  // Esta función SIEMPRE lee los sensores y actualiza el struct global 'flightDataPacket'.
  // Los valores por defecto se asignan si un sensor está deshabilitado.

  smartDelay(10);

  // --- GPS ---
  flightDataPacket.fix = gps.location.age();
  flightDataPacket.latitude = gps.location.isValid() ? gps.location.lat() : -1.0;
  flightDataPacket.longitude = gps.location.isValid() ? gps.location.lng() : -1.0;
  flightDataPacket.speed_kmph = gps.speed.kmph();
  flightDataPacket.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : -1.0;
  flightDataPacket.hour = (gps.time.hour() + 2) % 24;
  flightDataPacket.minute = gps.time.minute();
  flightDataPacket.second = gps.time.second();
  flightDataPacket.altitude_gps = gps.altitude.isValid() ? gps.altitude.meters() : -1.0;

  // --- Barómetro ---
  flightDataPacket.temperature = bmp280.getTemperature();
  uint32_t presion = bmp280.getPressure();
  flightDataPacket.altitude_bar = bmp280.calAltitude(presion, P0) - initialAltitude;//revisar altitud inicial (-3km en referencia)
  //flightDataPacket.altitude_bar = bmp280.calAltitude(presion, P0);
  flightDataPacket.pressure_hpa = (presion != 0.0 ? presion / 100.0 : -1.0);

  // --- IMU (Acelerómetro, Giroscopio, Magnetómetro) ---
  #if IMUConectada
    sensors_event_t accel_event, mag_event;
    accel.getEvent(&accel_event);
    gyroscope.read();
    mag.getEvent(&mag_event);
  
    // Guardar datos crudos en el struct
    flightDataPacket.acc_x = accel_event.acceleration.x;
    flightDataPacket.acc_y = accel_event.acceleration.y;
    flightDataPacket.acc_z = accel_event.acceleration.z;
    flightDataPacket.gyro_x = gyroscope.g.x * 0.0001527; // Conversión a rad/s
    flightDataPacket.gyro_y = gyroscope.g.y * 0.0001527;
    flightDataPacket.gyro_z = gyroscope.g.z * 0.0001527;
    flightDataPacket.mag_x = mag_event.magnetic.x;
    flightDataPacket.mag_y = mag_event.magnetic.y;
    flightDataPacket.mag_z = mag_event.magnetic.z;
    
    // Alimentar el filtro y obtener los ángulos de orientación
    filter.update(flightDataPacket.gyro_x, flightDataPacket.gyro_y, flightDataPacket.gyro_z, 
                  flightDataPacket.acc_x, flightDataPacket.acc_y, flightDataPacket.acc_z, 
                  flightDataPacket.mag_x, flightDataPacket.mag_y, flightDataPacket.mag_z);
    flightDataPacket.roll = filter.getRoll();
    flightDataPacket.pitch = filter.getPitch();
    flightDataPacket.yaw = filter.getYaw();
  #else
    // Si la IMU no está conectada, llenar el struct con valores nulos/de error
    flightDataPacket.acc_x = flightDataPacket.acc_y = flightDataPacket.acc_z = -1;
    flightDataPacket.gyro_x = flightDataPacket.gyro_y = flightDataPacket.gyro_z = -1;
    flightDataPacket.mag_x = flightDataPacket.mag_y = flightDataPacket.mag_z = -1;
    flightDataPacket.roll = flightDataPacket.pitch = flightDataPacket.yaw = -1;
  #endif
}

void runStateMachine() {
  // Esta función contiene toda la lógica de vuelo.
  // (La he dejado comentada como en tu código original, puedes activarla cuando quieras)
  /*
  float totalAcceleration = sqrt(pow(flightDataPacket.acc_x, 2) + pow(flightDataPacket.acc_y, 2) + pow(flightDataPacket.acc_z, 2));

  switch (currentState) {
    case TESTING:
      // ... tu lógica de TESTING ...
      break;
    case ON_PAD:
      // ... tu lógica de ON_PAD ...
      break;
    // ... etc ...
  }
  */
}

void handleTelemetry() {
  // Esta función se encarga de la telemetría LoRa, que se ejecuta con menos frecuencia.
  if (transmissionFinished) {
    #if usarLoRa
      #if debugLoRa
        transmissionFinished = false;
        enviarDatosPorLoRa("Esto es una prueba del LoRa"); // Esta función ahora es solo para debug
      #else
        transmissionFinished = false;
        // Enviamos el struct de telemetría directamente
        enviarPaqueteLoRa(&flightDataPacket, sizeof(flightDataPacket));
      #endif
    #endif
  }
}

void setup() {
  Serial.begin(115220);
  Serial2.begin(9600, SERIAL_8N1, 44, 43); 
  delay(2000); // Dar tiempo a que todo se estabilice

  D_PRINTLN("\n--- INICIANDO AVIÓNICA ARTEMISA II ---");

  setupPins();
  setupBuses();

  bool fallo[6] = {false, false, false, false, false, false};
  setupSensors(fallo);
  setupPeripherals(fallo);
  
  // Comprobar si alguna inicialización falló
  bool anyFailure = false;
  for(int i = 0; i < 6; i++) {
    if (fallo[i]) {
      anyFailure = true;
      break;
    }
  }

  if (anyFailure) {
    handleSetupFailure(fallo); // Si algo falló, entramos en un bucle infinito de error
  }

  D_PRINTLN("✅ Todos los componentes se han iniciado correctamente.");

  // Inicializar filtro AHRS y altitud de referencia
  filter.begin(1000);
  initialAltitude = bmp280.calAltitude(bmp280.getPressure(), P0);
  D_PRINT("Altitud inicial en la rampa: "); D_PRINT(initialAltitude); D_PRINTLN(" m");

  // Inicializar buffer de altitudes para detección de apogeo
  for (int i = 0; i < APOGEE_DETECTION_WINDOW; i++) {
    altitudeReadings[i] = -1000.0;
  }
  
  formatSDCard(); // Borrar la SD si está configurado

  D_PRINTLN("--- CONFIGURACIÓN FINALIZADA ---");
  D_PRINT("Estado inicial: "); D_PRINTLN(currentState);
}

void loop() {
  // 1. Leer todos los sensores y actualizar el struct 'flightDataPacket' con los datos más recientes.
  updateSensorData();

  // 2. Ejecutar la lógica de la máquina de estados de vuelo.
  runStateMachine();

  // 3. Guardar el struct 'flightDataPacket' en la tarjeta SD en cada ciclo.
  //    La función 'guardarDatosEnSD' ya sabe cómo convertir el struct a CSV.
  guardarDatosEnSD(flightDataPacket);

  // 4. Enviar el struct 'flightDataPacket' por LoRa si la radio está disponible.
  handleTelemetry();

  // 5. (Opcional) Imprimir el contenido del struct si 'verMensaje' está activado.
    #if verMensaje
    // Imprimimos los valores directamente desde el struct para asegurarnos
    // de que lo que vemos es lo que se guarda/envía en este ciclo.
    D_PRINTLN("\n--- Contenido del Struct en este Ciclo ---");

    // --- GPS ---
    D_PRINT("  GPS Time (UTC): ");
    if (flightDataPacket.hour < 10) D_PRINT('0');
    D_PRINT((unsigned int)flightDataPacket.hour);
    D_PRINT(':');
    if (flightDataPacket.minute < 10) D_PRINT('0');
    D_PRINT((unsigned int)flightDataPacket.minute);
    D_PRINT(':');
    if (flightDataPacket.second < 10) D_PRINT('0');
    D_PRINTLN((unsigned int)flightDataPacket.second);

    D_PRINT("  GPS Coords (Lat, Lon): ");
    D_PRINT(flightDataPacket.latitude, 6);
    D_PRINT(", ");
    D_PRINTLN(flightDataPacket.longitude, 6);
    
    D_PRINT("  GPS Speed:          "); D_PRINT(flightDataPacket.speed_kmph, 2); D_PRINTLN(" km/h");
    D_PRINT("  GPS Altitude:       "); D_PRINT(flightDataPacket.altitude_gps, 2); D_PRINTLN(" m");
    D_PRINT("  GPS HDOP:           "); D_PRINTLN(flightDataPacket.hdop, 2);

    // --- Barómetro ---
    D_PRINT("  Baro Altitude (Rel):"); D_PRINT(flightDataPacket.altitude_bar, 2); D_PRINTLN(" m");
    D_PRINT("  Baro Pressure:      "); D_PRINT(flightDataPacket.pressure_hpa, 2); D_PRINTLN(" hPa");
    D_PRINT("  Baro Temperature:   "); D_PRINT(flightDataPacket.temperature, 2); D_PRINTLN(" C");

    // --- IMU Raw Data ---
    D_PRINTLN("  --- IMU Cruda ---");
    D_PRINT("  Acceleration (X,Y,Z):   ");
    D_PRINT(flightDataPacket.acc_x, 3); D_PRINT(", ");
    D_PRINT(flightDataPacket.acc_y, 3); D_PRINT(", ");
    D_PRINT(flightDataPacket.acc_z, 3); D_PRINTLN(" m/s^2");

    D_PRINT("  Gyro Velocity (X,Y,Z):");
    D_PRINT(flightDataPacket.gyro_x, 3); D_PRINT(", ");
    D_PRINT(flightDataPacket.gyro_y, 3); D_PRINT(", ");
    D_PRINT(flightDataPacket.gyro_z, 3); D_PRINTLN(" rad/s");

    D_PRINT("  Magnetometer (X,Y,Z): ");
    D_PRINT(flightDataPacket.mag_x, 2); D_PRINT(", ");
    D_PRINT(flightDataPacket.mag_y, 2); D_PRINT(", ");
    D_PRINT(flightDataPacket.mag_z, 2); D_PRINTLN(" uT");

    // --- IMU Filtered Data (Orientación) ---
    D_PRINTLN("  --- Orientacion ---");
    D_PRINT("  Roll:  "); D_PRINT(flightDataPacket.roll, 2); D_PRINTLN(" deg");
    D_PRINT("  Pitch: "); D_PRINT(flightDataPacket.pitch, 2); D_PRINTLN(" deg");
    D_PRINT("  Yaw:   "); D_PRINT(flightDataPacket.yaw, 2); D_PRINTLN(" deg");
    
  #endif
}