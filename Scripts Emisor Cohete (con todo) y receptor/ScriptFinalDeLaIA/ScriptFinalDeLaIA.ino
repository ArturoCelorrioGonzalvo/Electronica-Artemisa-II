#include <SD.h>
#include <Wire.h>
#include <BMP280.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
//#include <LoRa.h> Uso la librería RadioLib
#include <RadioLib.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>//Librería del giróscopo
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
#define verMensaje false

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
  Adafruit_L3GD20_Unified gyroscope = Adafruit_L3GD20_Unified(20);
  Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
#endif

struct TelemetryData {
  
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

void guardarDatosEnSD(String datos) {
  digitalWrite(LoRa_CS, HIGH);//Desactivamos LoRa
  digitalWrite(SD_CS, LOW); //Activamos SD
  
  File archivo = SD.open("/datos.csv", FILE_APPEND);
  if (!archivo) {
    Serial.println("❌ Error al escribir en microSD!");
    return;
  }
  archivo.println(datos);
  archivo.close();
  //Serial.println("✅ Datos guardados en microSD!");
}

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

void setup() {
  bool fallo[6] = {false,false,false,false,false,false};
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 44, 43); 
  filter.begin(1000);
  delay(1000);

  #if DEBUG_MODE
    pinMode(LED_BUILTIN, OUTPUT);      // Configura el pin del LED integrado como salida
    digitalWrite(LED_BUILTIN, LOW);       // Asegura que el LED empiece apagado
  #endif
  

  pinMode(DEPLOYMENT_PIN, OUTPUT);
  digitalWrite(DEPLOYMENT_PIN, LOW); // ¡CRÍTICO! Asegurarse de que empieza APAGADO

  // Inicializar el buffer de altitudes
  for (int i = 0; i < APOGEE_DETECTION_WINDOW; i++) {
    altitudeReadings[i] = -1000.0;
  }

  D_PRINTLN("Aviónica lista.");
  D_PRINT("Estado inicial: ");
  D_PRINTLN(currentState); // Imprimirá el número del estado (0 para TESTING)
  
  pinMode(SD_CS, OUTPUT);
  #if usarLoRa 
    pinMode(LoRa_CS, OUTPUT); 
  #endif

  //Los pines CS son activos en bajo
  digitalWrite(SD_CS, HIGH); 
  #if usarLoRa
    digitalWrite(LoRa_CS, HIGH);
  #endif
  
  
  // Inicializar Wire para el BMP280 y el acelerómetro
  Wire.begin(SDA_PIN, SCL_PIN);
  //Wire.begin();
  SPI.begin(SCK, MISO, MOSI);
  
  //Incializar Acelerómetro
  #if IMUConectada
    Serial.println("Test 1: Acelerómetro");
    if(!accel.begin()){
    fallo[0]=true;
    }
    //displaySensorDetails();
  #endif
  
  delay(1000);

  #if IMUConectada
    Serial.println("Test 2: Giróscopo");
    if(!gyroscope.begin(, 0x69)){
      fallo[1] = true; // Usamos el nuevo espacio en el array
    }
  #endif

  delay(1000);
  
  #if IMUConectada
    Serial.println("Test 3: Magnetómetro");
    if(!mag.begin()){
      fallo[2] = true; // Usamos el nuevo espacio en el array
    }
  #endif

  delay(1000);
  
  Serial.println("Test 4: Barómetro");
  // Inicializar barómetro BMP280
  int status = bmp280.begin();
  if (status!=0) {
    
    fallo[3] = true;
    
  }
  delay(1000);

  initialAltitude = bmp280.calAltitude(bmp280.getPressure(), P0);
  D_PRINT("Altitud inicial en la rampa: "); D_PRINTLN(initialAltitude);
  
  //Serial.println("✅ BMP280 iniciado!");

  Serial.println("Test 5: SD");
  // Inicializar SD
  digitalWrite(SD_CS, LOW); 
  #if usarLoRa
    digitalWrite(LoRa_CS, HIGH);
  #endif
  
  
  if (!SD.begin(SD_CS)) {
    
    fallo[4] = true;
    
  }
  //Serial.println("✅ MicroSD lista!");
  delay(1000);

  // Inicializar LoRa

  #if usarLoRa
      Serial.println("Test 6: LoRa");
      digitalWrite(SD_CS, HIGH); 
      digitalWrite(LoRa_CS, LOW);

      // Inicializamos el bus SPI. Es mejor no pasar el pin CS aquí,
      // ya que RadioLib gestiona su estado (HIGH/LOW) por sí mismo.
      
    
      // Inicializamos el módulo de radio
      int state = radio.begin();
      if (state != RADIOLIB_ERR_NONE) { 
        // Si la inicialización falla, marcamos el error
        fallo[5] = true;
      } else {
        // Si la inicialización fue exitosa, configuramos los parámetros de largo alcance
        radio.setSpreadingFactor(9);//3~4km
        radio.setBandwidth(125.0);//
        radio.setCodingRate(6);
        radio.setSyncWord(0xAB); // Palabra de sincronización personalizada
        radio.setOutputPower(17); // Potencia de transmisión (ej. 17 dBm)
         // --- NUEVO: CONFIGURAR LA INTERRUPCIÓN ---
        // Le decimos a RadioLib que llame a nuestra función 'onTxDone'
        // cuando el evento de Transmisión Completada ocurra en el pin DI0.
        attachInterrupt(digitalPinToInterrupt(LoRa_DI0), onTxDone, RISING);
        Serial.println("✅ LoRa listo!");
      }
  #else
      Serial.println("No hay LoRa conectado");
      fallo[5] = false;
  #endif
  delay(1000);

  while(fallo[0]||fallo[1]||fallo[2]||fallo[3]||fallo[4]||fallo[5]){
    if(fallo[0]) {
      Serial.println("❌ Error al iniciar Acelerómetro");
    }
    if(fallo[1]){
      Serial.println("❌ Error al iniciar Giróscopo");
    }
    if (fallo[2]){
      Serial.println("❌ Error al iniciar Magnetómetro");
    }
    if(fallo[3]){
      Serial.println("❌ Error al iniciar Barómetro");
    }
    if(fallo[4]){
      Serial.println("❌ Error al inicializar microSD");
    }
    if (fallo[5]){
      Serial.println("❌ Error al iniciar LoRa");
    }
    
    delay(3000);
  }

  #if borrarSD 
    digitalWrite(SD_CS, LOW); 
    #if usarLoRa
      digitalWrite(LoRa_CS, HIGH);
    #endif

    Serial.println("🗑 Eliminando archivos en la microSD...");
    File root = SD.open("/");

    File archivo = root.openNextFile();
    while (archivo) {
      //Serial.print("📂 Procesando: ");
      //Serial.println(archivo.name());

      archivo.close(); // Cerrar antes de eliminar
      String nombreArchivo=archivo.name();

      

      if (!nombreArchivo.equals("/System Volume Information") && SD.remove(nombreArchivo)) {
        Serial.print("✅ Archivo eliminado: ");
        Serial.println(nombreArchivo);
      }

      archivo = root.openNextFile(); // Obtener el siguiente archivo
    }

    Serial.println("✔ Proceso completado, continuando con el resto del código");
    
  #endif
  delay(1000);
}

void loop() {
  //leer milis en variable += periodo
  //Serial.println("📍 Obteniendo datos...");
  
  // Esperar datos del GPS antes de procesar
  smartDelay(1);



  float latitud = gps.location.isValid() ? gps.location.lat() : -1.0;
  float longitud = gps.location.isValid() ? gps.location.lng() : -1.0;
  float velocidad = gps.speed.kmph();
  float HDOP = gps.hdop.isValid() ? gps.hdop.hdop() : -1.0;
  int hora = (gps.time.hour()+2)%24;
  int minuto = gps.time.minute();
  int segundo = gps.time.second();
  float altitudGPS = gps.altitude.isValid() ? gps.altitude.meters() : -1.0;
  

  
  
  // Leer datos del barómetro
  //Wire.begin(SDA_PIN,SCL_PIN);
  float temperatura = bmp280.getTemperature();
  uint32_t presion = bmp280.getPressure();
  //float altitudBar = bmp280.calAltitude(presion, P0);
  float altitudBar = bmp280.calAltitude(presion, P0) - initialAltitude;
  /* Display the results (acceleration is measured in m/s^2) */
  
  #if IMUConectada
  sensors_event_t accel_event;
    accel.getEvent(&accel_event);
    
    sensors_event_t gyro_event;
    gyroscope.getEvent(&gyro_event);

    sensors_event_t mag_event; // <-- El objeto que faltaba
    mag.getEvent(&mag_event);
  
    // --- EXTRACCIÓN DE DATOS CRUDOS ---
    float acc_x = accel_event.acceleration.x;
    float acc_y = accel_event.acceleration.y;
    float acc_z = accel_event.acceleration.z;
    
    // Las velocidades angulares deben estar en radianes/segundo (la librería ya lo hace)
    float gyro_x = gyro_event.gyro.x;
    float gyro_y = gyro_event.gyro.y;
    float gyro_z = gyro_event.gyro.z;

    // Los datos del magnetómetro en micro-Tesla (uT) (la librería ya lo hace)
    float mag_x = mag_event.magnetic.x;
    float mag_y = mag_event.magnetic.y;
    float mag_z = mag_event.magnetic.z;
    
    // --- FUSIÓN DE SENSORES ---
    // Alimentamos el filtro con los 9 ejes de datos
    filter.update(gyro_x, gyro_y, gyro_z, 
                  acc_x, acc_y, acc_z, 
                  mag_x, mag_y, mag_z);

    // --- OBTENCIÓN DE LOS ÁNGULOS FINALES ---
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw = filter.getYaw();
  #else
    float acc_x = -1, acc_y = -1, acc_z = -1;
    float gyro_x = -1, gyro_y = -1, gyro_z = -1;
    float roll = -1, pitch = -1, yaw = -1;
  #endif
  
  
  //Máquina de estados, de momento deshabilitada
  /*
  switch (currentState) {
    case TESTING:
      // Este estado usa 'Serial.print' directamente porque es una interfaz de usuario,
      // no un mensaje de debug. Siempre debe funcionar.
      if (millis() - lastTestPrint > 1000) {
        Serial.println("\n--- MODO TEST ---");
        Serial.print("  Altitud Relativa: "); Serial.print(altitudBar); Serial.println(" m");
        Serial.print("  Aceleración Total: "); Serial.print(totalAcceleration); Serial.println(" m/s^2");
        Serial.println("  Enviar 'f' para probar disparo (fire).");
        lastTestPrint = millis();
      }

      if (Serial.available() > 0 && Serial.read() == 'f') {
        Serial.println("\n!!! COMANDO DE DISPARO DE PRUEBA RECIBIDO !!!");
        digitalWrite(DEPLOYMENT_PIN, HIGH);
        delay(2000);
        digitalWrite(DEPLOYMENT_PIN, LOW);
        Serial.println("... Secuencia de prueba finalizada.");
      }
      break;

    case ON_PAD:
      if (totalAcceleration > LAUNCH_ACCELERATION_THRESHOLD) {
        Serial.println("¡LANZAMIENTO DETECTADO! Estado: ASCENDING");
        currentState = ASCENDING;
      }
      break;

    case ASCENDING:
      altitudeReadings[readingIndex] = altitudBar;
      readingIndex = (readingIndex + 1) % APOGEE_DETECTION_WINDOW;
      if (readingIndex == 0) altitudeBufferFull = true;

      if (altitudeBufferFull) {
        bool descending = true;
        for (int i = 0; i < APOGEE_DETECTION_WINDOW; i++) {
          if (altitudBar >= altitudeReadings[i]) {
            descending = false;
            break;
          }
        }
        if (descending) {
          Serial.println("¡APOGEO DETECTADO! Desplegando paracaídas.");
          
          #if DEBUG_MODE
            // Encender el LED como confirmación visual SOLO en modo debug
            digitalWrite(LED_BUILTIN, HIGH); 
          #endif

          digitalWrite(DEPLOYMENT_PIN, HIGH);
          currentState = APOGEE_DETECTED;
        }
      }
      break;

    case APOGEE_DETECTED:
      delay(2000);
      digitalWrite(DEPLOYMENT_PIN, LOW);
      Serial.println("Secuencia de despliegue finalizada. Estado: RECOVERY");
      currentState = RECOVERY;
      break;

    case RECOVERY:
      // El cohete está descendiendo. No hacemos más acciones de control,
      // pero la telemetría y el guardado en SD siguen funcionando.
      break;
  }
  */



  char buffer[200];
  String datos = "";

  
  
  //Casos de debug

  #if debugGPS
    latitud = gps.location.lat();
    longitud = gps.location.lng();
    velocidad = gps.speed.kmph();
    HDOP = gps.hdop.hdop();
    hora = (gps.time.hour()+2)%24;
    minuto = gps.time.minute();
    segundo = gps.time.second();
    altitudGPS = gps.altitude.meters();
    datos += "Latitud: "+ String(latitud,8) + "º; Longitud: " + String(longitud,8) + " º; Velocidad: " + String(velocidad) + " km/h; HDOP: " + String(HDOP) + 
                "; Hora: " + String(hora) + ":" + String(minuto) + ":" + String(segundo) +"; Altitud: " + String(altitudGPS) + "m";
    
    #if debugBar
      datos += "; Presión: " + String(presion/100) + " hPa; Temperatura: " + String(temperatura) +" ºC; Altitud calculada: " + String(altitudBar) + "m";
      
      #if (debugIMU && IMUConectada) //Añadir giróscopo y magnetómetro (desde el filtro?)
        datos += "; Aceleración en X: " + String(acc_x) + "m/s^2; Aceleración en Y: " + String(acc_y) + " m/s^2; Aceleración en Z: " + String(acc_z) + 
                "m/s^2; Velocidad angular en X: " + String(gyro_x) + "rad/s; Velocidad angular en Y: " + String(gyro_y) + "rad/s; Velocidad angular en Z: " 
                + String(gyro_z) + "rad/s; Campo magnético en X: " + String(mag_x) + "uT; Campo magnético en Y: " + String(mag_y) 
                + " uT; Campo magnético en Z: " + String(mag_z) + "uT; Roll: " + String(roll) + "º; Pitch: " + String(pitch) + "º; Yaw: " + String(yaw) +"º";
      #endif
    #endif
  #else 
    #if debugBar
      datos += "; Presión: " + String(presion/100) + " hPa; Temperatura: " + String(temperatura) +" ºC; Altitud calculada: " + String(altitudBar) + "m";
      
      #if (debugIMU && IMUConectada) //Añadir giróscopo y magnetómetro (desde el filtro?)
        datos += "; Aceleración en X: " + String(acc_x) + "m/s^2; Aceleración en Y: " + String(acc_y) + " m/s^2; Aceleración en Z: " + String(acc_z) + 
                "m/s^2; Velocidad angular en X: " + String(gyro_x) + "rad/s; Velocidad angular en Y: " + String(gyro_y) + "rad/s; Velocidad angular en Z: " 
                + String(gyro_z) + "rad/s; Campo magnético en X: " + String(mag_x) + "uT; Campo magnético en Y: " + String(mag_y) 
                + " uT; Campo magnético en Z: " + String(mag_z) + "uT; Roll: " + String(roll) + "º; Pitch: " + String(pitch) + "º; Yaw: " + String(yaw) +"º";
      #endif
    #else 
      #if (debugIMU && IMUConectada) //Añadir giróscopo y magnetómetro (desde el filtro?)
        datos += "; Aceleración en X: " + String(acc_x) + "m/s^2; Aceleración en Y: " + String(acc_y) + " m/s^2; Aceleración en Z: " + String(acc_z) + 
                "m/s^2; Velocidad angular en X: " + String(gyro_x) + "rad/s; Velocidad angular en Y: " + String(gyro_y) + "rad/s; Velocidad angular en Z: " 
                + String(gyro_z) + "rad/s; Campo magnético en X: " + String(mag_x) + "uT; Campo magnético en Y: " + String(mag_y) 
                + " uT; Campo magnético en Z: " + String(mag_z) + "uT; Roll: " + String(roll) + "º; Pitch: " + String(pitch) + "º; Yaw: " + String(yaw) +"º";
      #else
        #if IMUConectada //Añadir giróscopo y magnetómetro (desde el filtro?)
        //Orden de los campos:
        //HDOP, HORA, MINUTO, SEGUNDO, LATITUD, LONGITUD, VELOCIDAD_GPS, ALTITUD_GPS, PRESIÓN, ALTITUD_BAR, TEMPERATURA, ACC_X, ACC_Y, ACC_Z, 
        //(sigue) VEL_ANG_X, VEL_ANG_Y, VEL_ANG_Z, CAMPO_MAG_X, CAMPO_MAG_Y, CAMPO_MAG_Z, ROLL, PITCH YAW
          sprintf(buffer,"%f,%d,%d,%d,%.8f,%.8f,%.2f,%.1f,%.2f,%.2f,%.2f,%.8f,%.8f,%.8f,%.8f,%.8f,:%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f;",HDOP,hora,minuto,segundo,latitud,longitud,velocidad,altitudGPS,(presion!=0.0? presion/100.0 : -1), (altitudBar>0.0? altitudBar : -1.0),(temperatura!=0.0 ? temperatura : -1.0), acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, roll, pitch, yaw); 
          datos = String(buffer);
        #else
        //Orden de los campos:
        //HDOP, HORA, MINUTO, SEGUNDO, LATITUD, LONGITUD, VELOCIDAD_GPS, ALTITUD_GPS, PRESIÓN, ALTITUD_BAR, TEMPERATURA
          sprintf(buffer,"%f,%d,%d,%d,%.8f,%.8f,%.2f,%.1f,%.2f,%.2f,%.2f,",HDOP,hora,minuto,segundo,latitud,longitud,velocidad,altitudGPS,(presion!=0.0? presion/100.0 : -1), (altitudBar>0.0? altitudBar : -1.0),(temperatura!=0.0 ? temperatura : -1.0)); 
          datos = String(buffer);
        #endif
      #endif
    #endif
  #endif
  

  #if verMensaje
    Serial.println("💾 Datos a guardar: ");
    Serial.println(datos);
  #endif

  //Divido en 2, si quiero hacer debug guardo con el string, sino con el struct
  #if !debugResto
      flightDataPacket.hdop = HDOP;
      flightDataPacket.hour = hora;
      flightDataPacket.minute = minuto;
      flightDataPacket.second = segundo;
      flightDataPacket.latitude = latitud;
      flightDataPacket.longitude = longitud;
      flightDataPacket.speed_kmph = velocidad;
      flightDataPacket.altitude_gps = altitudGPS;
      flightDataPacket.pressure_hpa = (presion != 0.0 ? presion / 100.0 : -1);
      flightDataPacket.altitude_bar = (altitudBar > 0.0 ? altitudBar : -1.0);
      flightDataPacket.temperature = (temperatura != 0.0 ? temperatura : -1.0);
      flightDataPacket.acc_x = acc_x;
      flightDataPacket.acc_y = acc_y;
      flightDataPacket.acc_z = acc_z;
      flightDataPacket.gyro_x = gyro_x;
      flightDataPacket.gyro_y = gyro_y;
      flightDataPacket.gyro_z = gyro_z;
      flightDataPacket.mag_x = mag_x;
      flightDataPacket.mag_y = mag_y;
      flightDataPacket.mag_z = mag_z;
      flightDataPacket.roll = roll;
      flightDataPacket.pitch = pitch;
      flightDataPacket.yaw = yaw;
      guardarDatosEnSD(flightDataPacket);
  #else
      guardarDatosEnSD(datos);
  #endif
  
  // --- SECCIÓN DE ENVÍO POR LORA (LÓGICA MODIFICADA) ---
  
  

  // Solo entramos en la lógica de envío si la radio está libre.
  if (transmissionFinished) {
      #if (usarLoRa && !debugLoRa)
        
        transmissionFinished = false;
        //Ya está todo metido en el struct de cuando se ha guardado en la SD
      
        enviarPaqueteLoRa(&flightDataPacket, sizeof(flightDataPacket));
      #endif // Fin de la comprobación usarLoRa
    
    

      #if (usarLoRa && debugLoRa)
          // MODO DEBUG LORA:
          transmissionFinished = false;
          enviarDatosPorLoRa("Esto es una prueba del LoRa");
      #endif //Chequeo del modo debug
    

    
  } 
}