#include <SD.h>
#include <Wire.h>
#include <BMP280.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
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



//Configuración del acelerómetro
#if IMUConectada
  Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
#endif

// Configuración del GPS
//SoftwareSerial ss(3, 4);  //Ya no hace falta, está conectado directamente a RX y TX del ESP
//RX es el GPIO43 y TX es el GPIO44

//trabajar a 10Hz

//Usar @ifdef para compilar dependiendo de variables
//tiempo fix en el String de datos del GPS

//Meter acelerómetro
TinyGPSPlus gps;

// Configuración del Barómetro
BMP280 bmp280;

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
CS pin 6
MOSI pin 11
CLK pin 13
MISO pin 12
GND GND 
*/

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
  /*
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
  */
}

//Cambiar los mensajes porque no podré usar el serial porque no me puedo conectar a la placa

void setup() {
  bool fallo[4] = {false,false,false,false};
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 44, 43); 
  delay(1000);
  
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
    //while (true);
  }
  delay(1000);
  
  //Serial.println("✅ BMP280 iniciado!");

  Serial.println("Test 3");
  // Inicializar SD
  digitalWrite(SD_CS, LOW); 
  #if usarLoRa
    digitalWrite(LoRa_CS, HIGH);
  #endif
  SPI.begin(SCK, MISO, MOSI, SD_CS);
  
  if (!SD.begin(SD_CS)) {
    
    fallo[2] = true;
    //while (true);
  }
  //Serial.println("✅ MicroSD lista!");
  delay(1000);

  // Inicializar LoRa

  #if usarLoRa
    Serial.println("Test 4");
    digitalWrite(SD_CS, HIGH); 
    digitalWrite(LoRa_CS, LOW);

    SPI.begin(SCK, MISO, MOSI, LoRa_CS);
    LoRa.setPins(LoRa_CS, LoRa_RST, LoRa_DI0);
  
    if (LoRa.begin(433E6)!=1) { 
      fallo[3]=true;
    }
    //Serial.println("✅ LoRa listo!");
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
  smartDelay(10);//100ms = 10Hz

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
  float altitudBar = bmp280.calAltitude(presion, P0);
  /* Display the results (acceleration is measured in m/s^2) */
  #if IMUConectada
    float acc_x = event.acceleration.x;
    float acc_y = event.acceleration.y;
    float acc_z = event.acceleration.z;
  #endif
  // Crear el mensaje
  /*String datos = String(HDOP) + ";" + String(hora)+":"+String(minuto)+":"+String(segundo)+";"+
                 String(latitud,8) + ";" + String(longitud,8) + ";" + 
                 String(velocidad,2) + ";" + String(altitudGPS,1) + ";" + 
                 String((presion/100.0!=0.0? presion/100.0 : -1),2) + ";" + String((altitudBar>0.0? altitudBar : -1.0),1)+";"+
                 String((temperatura!=0.0 ? temperatura : -1.0),2) + ";x:" + String(acc_x,8) + ";y:" + String(acc_y,8) +
                 ";z:" + String(acc_z,8);
  */
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
  
  // Enviar por LoRa
  #if debugResto
    delay(0);
  #else
    #if debugLoRa
      enviarDatosPorLoRa("Esto es una prueba del LoRa");
    #else
      guardarDatosEnSD(datos);
      #if usarLoRa
        enviarDatosPorLoRa(datos);
      #endif
    #endif
  #endif
}

// Función optimizada para procesar datos del GPS
static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (Serial2.available()) {
            gps.encode(Serial2.read());
        }
    } while (millis() - start < ms);
}


