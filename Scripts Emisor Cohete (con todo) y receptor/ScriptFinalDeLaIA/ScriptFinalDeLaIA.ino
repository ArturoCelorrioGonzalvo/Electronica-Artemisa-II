#include <SD.h>
#include <Wire.h>
#include <BMP280.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
//#include <LoRa.h> Uso la librería RadioLib
#include <RadioLib.h>
#include <Adafruit_LSM303_U.h>

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

//Configuración del acelerómetro
#if IMUConectada
  Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
#endif

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
struct TelemetryData {
  //uint8_t testCero;
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
//Cambiar a UTF-8?

// Creamos una instancia global de la estructura para llenarla en el loop
TelemetryData flightDataPacket;

// Creamos la instancia del objeto de radio usando tus pines definidos
// La firma es (CS, IRQ/DIO0, RST)
SX1278 radio = new Module(LoRa_CS, LoRa_DI0, LoRa_RST);


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
  bool fallo[4] = {false,false,false,false};
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 44, 43); 
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

  // ... resto de tu código de setup (inicialización de sensores, LoRa, SD) ...

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
    Serial.println("Test 1");
    if(!accel.begin()){
    //There was a problem detecting the ADXL345 ... check your connections 
    fallo[0]=true;
    }
    //displaySensorDetails();
  #endif
  
  delay(1000);
  
  
  Serial.println("Test 2");
  // Inicializar barómetro BMP280
  int status = bmp280.begin();
  if (status!=0) {
    
    fallo[1] = true;
    
  }
  delay(1000);

  initialAltitude = bmp280.calAltitude(bmp280.getPressure(), P0);
  D_PRINT("Altitud inicial en la rampa: "); D_PRINTLN(initialAltitude);
  
  //Serial.println("✅ BMP280 iniciado!");

  Serial.println("Test 3");
  // Inicializar SD
  digitalWrite(SD_CS, LOW); 
  #if usarLoRa
    digitalWrite(LoRa_CS, HIGH);
  #endif
  
  
  if (!SD.begin(SD_CS)) {
    
    fallo[2] = true;
    
  }
  //Serial.println("✅ MicroSD lista!");
  delay(1000);

  // Inicializar LoRa

  #if usarLoRa
      Serial.println("Test 4");
      digitalWrite(SD_CS, HIGH); 
      digitalWrite(LoRa_CS, LOW);

      // Inicializamos el bus SPI. Es mejor no pasar el pin CS aquí,
      // ya que RadioLib gestiona su estado (HIGH/LOW) por sí mismo.
      
    
      // Inicializamos el módulo de radio
      int state = radio.begin();
      if (state != RADIOLIB_ERR_NONE) { 
        // Si la inicialización falla, marcamos el error
        fallo[3] = true;
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
      fallo[3] = false;
  #endif
  delay(1000);

  while(fallo[0]||fallo[1]||fallo[2]||fallo[3]){
    if(fallo[0]) {
      Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    }
    if(fallo[1]){
      Serial.println("❌ Error al iniciar BMP280");
    }
    if(fallo[2]){
      Serial.println("❌ Error al inicializar microSD");
    }
    if (fallo[3]){
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

  #if IMUConectada
    sensors_event_t event;
    accel.getEvent(&event);
  #endif
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
    float acc_x = event.acceleration.x;
    float acc_y = event.acceleration.y;
    float acc_z = event.acceleration.z;
  #else
    float acc_x = -1;
    float acc_y = -1;
    float acc_z = -1;
  #endif
  
  float totalAcceleration = sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);

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



  char buffer[200];
  String datos = "";

  // Guardar en microSD
  

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
      
      #if (debugIMU && IMUConectada)
        datos += "; Aceleración en X: " + String(acc_x) + "m/s^2; Aceleración en Y: " + String(acc_y) + " m/s^2; Aceleración en Z: " + String(acc_z);
      #endif
    #endif
  #else 
    #if debugBar
      datos += "; Presión: " + String(presion/100) + " hPa; Temperatura: " + String(temperatura) +" ºC; Altitud calculada: " + String(altitudBar) + "m";
      
      #if (debugIMU && IMUConectada) 
        datos += "; Aceleración en X: " + String(acc_x) + "m/s^2; Aceleración en Y: " + String(acc_y) + " m/s^2; Aceleración en Z: " + String(acc_z);
      #endif
    #else 
      #if (debugIMU && IMUConectada)
        datos += "Aceleración en X: " + String(acc_x) + "m/s^2; Aceleración en Y: " + String(acc_y) + " m/s^2; Aceleración en Z: " + String(acc_z);
      #else
        #if IMUConectada
          sprintf(buffer,"HDOP:%f;%d:%d:%d;%.8f;%.8f;%.2f;%.1f;%.2f;%.2f;%.2f;x:%.8f;y:%.8f;z:%.8f",HDOP,hora,minuto,segundo,latitud,longitud,velocidad,altitudGPS,(presion!=0.0? presion/100.0 : -1), (altitudBar>0.0? altitudBar : -1.0),(temperatura!=0.0 ? temperatura : -1.0), acc_x, acc_y, acc_z); 
          datos = String(buffer);
        #else
          sprintf(buffer,"HDOP:%f;%d:%d:%d;%.8f;%.8f;%.2f;%.1f;%.2f;%.2f;%.2f;",HDOP,hora,minuto,segundo,latitud,longitud,velocidad,altitudGPS,(presion!=0.0? presion/100.0 : -1), (altitudBar>0.0? altitudBar : -1.0),(temperatura!=0.0 ? temperatura : -1.0)); 
          datos = String(buffer);
        #endif
      #endif
    #endif
  #endif
  //Cambiar de orden para que lo primero que salga sea lo del GPS

  #if verMensaje
    Serial.println("💾 Datos a guardar: ");
    Serial.println(datos);
  #endif
  
  // --- SECCIÓN DE ENVÍO POR LORA (LÓGICA MODIFICADA) ---
  
  // Solo entramos en la lógica de envío si la radio está libre.
  if (transmissionFinished) {
  
    #if debugResto
      delay(0);
    #else
      // MODO DE VUELO NORMAL:
      // La escritura en la SD siempre se hace en este modo, independientemente del LoRa.
      guardarDatosEnSD(datos); 
      
      // AHORA, COMPROBAMOS SI VAMOS A USAR LORA EN GENERAL
      #if (usarLoRa && !debugLoRa)
        // Dentro de este bloque, sabemos que el hardware LoRa está activo.
        // Ahora podemos decidir si es para debug o para telemetría normal.

        #if debugLoRa
          // MODO DEBUG LORA:
          transmissionFinished = false;
          enviarDatosPorLoRa("Esto es una prueba del LoRa");
        #else
          // MODO DE VUELO NORMAL CON TELEMETRÍA:
          transmissionFinished = false;

          // 1. Llenamos la estructura (sin cambios)
          flightDataPacket.hdop = HDOP;
          flightDataPacket.hour = hora;
          flightDataPacket.minute = minuto;
          flightDataPacket.second = segundo;
          flightDataPacket.latitude = latitud;
          //flightDataPacket.testCero = 0;
          flightDataPacket.longitude = longitud;
          flightDataPacket.speed_kmph = velocidad;
          flightDataPacket.altitude_gps = altitudGPS;
          flightDataPacket.pressure_hpa = (presion != 0.0 ? presion / 100.0 : -1);
          flightDataPacket.altitude_bar = (altitudBar > 0.0 ? altitudBar : -1.0);
          flightDataPacket.temperature = (temperatura != 0.0 ? temperatura : -1.0);
          flightDataPacket.acc_x = acc_x;
          flightDataPacket.acc_y = acc_y;
          flightDataPacket.acc_z = acc_z;

          
          
          // 2. Iniciamos el envío NO BLOQUEANTE
          enviarPaqueteLoRa(&flightDataPacket, sizeof(flightDataPacket));
          
        #endif // Fin de la comprobación debugLoRa
      #endif // Fin de la comprobación usarLoRa
    #endif // Fin de la comprobación debugResto
  } 
}





