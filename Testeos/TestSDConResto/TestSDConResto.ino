#include <WiFi.h>
#include <SD.h>
#include <Wire.h>
#include <BMP280.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define SD_CS 10  
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK 12
#define P0 1013.25 
#define SDA_PIN 7 
#define SCL_PIN 8 

BMP280 bmp280;
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
static const int maxDatos=10; //Hacer 500 cuando funcione

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

const char* ssid = "MIWIFI_MXvM";    
const char* password = "Rr7mr33s";  

uint8_t status = -1;
AsyncWebServer server(80);

int numDatos = 0;
bool servidorActivo = false;
bool esperandoComando = false;

void setup() {
  Serial.setDebugOutput(true);
Serial.println("Depuración activada...");
    Serial.begin(115200);
    Serial.println("ESP32 iniciado...");
    ss.begin(GPSBaud);
    delay(10);

    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("Comunicación I2C inicializada.");
    status = bmp280.begin();
    
    if (status != 0) {
        Serial.println("Error al inicializar BMP280.");
        return;
    }

    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

    Serial.println("Intentando inicializar la tarjeta SD...");
    if (!SD.begin(SD_CS)) {
        Serial.println("Error al inicializar la tarjeta SD.");
        return;
    }
    Serial.println("Tarjeta SD montada correctamente.");
}

void loop() {
    if (esperandoComando) {
        verificarPuertoSerie();
        //vTaskDelay(portMAX_DELAY);
        return;  
    }

    float temperature = bmp280.getTemperature();
    float pressure = bmp280.getPressure(); 
    float altitude = bmp280.calAltitude(pressure, P0); 

    File barometroFile = SD.open("/barometro.txt", FILE_APPEND);
    File gpsFile = SD.open("/gps.txt", FILE_APPEND);

    if (barometroFile && status == 0) {
        barometroFile.printf("Temperatura: %.2f °C\n", temperature);
        barometroFile.printf("Presión: %.2f hPa\n", pressure / 100);
        barometroFile.printf("Altura estimada: %.2f m\n", altitude);
        barometroFile.close();
    }

    smartDelay(100);

    if (gpsFile) {
        gpsFile.printf("%d %.2f %.6f %.6f %02d/%02d/%04d %02d:%02d:%02d %.2f %.2f %.2f %.2f %lu %lu %lu\n",
            gps.satellites.value(),
            gps.hdop.hdop(),
            gps.location.lat(), gps.location.lng(),
            gps.date.day(), gps.date.month(), gps.date.year(),
            gps.time.hour(), gps.time.minute(), gps.time.second(),
            gps.altitude.meters(),
            gps.course.deg(),
            gps.speed.kmph(),
            gps.distanceBetween(gps.location.lat(), gps.location.lng(), 41.6839, -0.8873),
            gps.charsProcessed(),
            gps.sentencesWithFix(),
            gps.failedChecksum());
        gpsFile.close();
    }

    numDatos++;
    Serial.printf("Datos recogidos: %d\n", numDatos);

    if (numDatos >= maxDatos) {
        Serial.println("Se han alcanzado 500 muestras. Esperando el comando 'iniciar'...");
        esperandoComando = true;
    }
}

void verificarPuertoSerie() {
    if (Serial.available()) {
        String comando = Serial.readStringUntil('\n');
        comando.trim();

        if (comando.equalsIgnoreCase("iniciar")) {
            iniciarServidor();
        }
    }
}

void iniciarServidor() {
    Serial.println("Comando 'iniciar' recibido. Activando servidor web...");
    servidorActivo = true;
    
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    Serial.println("Intentando conectar a WiFi...");
    
    unsigned long tiempoInicio = millis();
    bool conectado = false;
    int resultado = -1;
    
    while (millis() - tiempoInicio < 15000 && resultado!=3) { 
      resultado = WiFi.waitForConnectResult();
        Serial.print(".");
        delay(500);
    }

    

    Serial.println("Servidor Web Iniciado.");
    Serial.print("Dirección IP: ");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Servidor activo. Usa /download para acceder a archivos.");
    });

    server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("file")) {
            String fileName = request->getParam("file")->value();
            File file = SD.open("/" + fileName, FILE_READ);
            if (file) {
                request->send(SD, "/" + fileName, "application/octet-stream", true);
                file.close();
            } else {
                request->send(404, "text/plain", "Archivo no encontrado.");
            }
        } else {
            request->send(400, "text/plain", "Nombre de archivo no especificado.");
        }
    });
    
    server.begin();
}

static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (ss.available()) {
            gps.encode(ss.read());
        }
    } while (millis() - start < ms);
}