#include <WiFi.h>
#include <SD.h>
#include <SPI.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define SD_CS 3  // Pin CS de la tarjeta SD
#define SD_MOSI 11
#define SD_MISO 12
#define SD_SCK 13

const char* ssid = "telekino";    // MIWIFI_MXvM Galaxy
const char* password = "LeonardoTQ1852";  //  Rr7mr33s 00000000

AsyncWebServer server(80);

void setup() {
    Serial.begin(115200);
    Serial.println("El ESP32 ha arrancado correctamente.");

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Conectando a WiFi...");
    }

    Serial.println("Conectado a WiFi");
    

    // Inicializar SPI con los pines personalizados
    Serial.println("Inicializando SPI...");
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    
    Serial.print("📡 Dirección IP del ESP32: ");
    Serial.println(WiFi.localIP());

    // Intentar inicializar la tarjeta SD
    Serial.println("Intentando inicializar la tarjeta SD...");
    if (!SD.begin(SD_CS)) {
        Serial.println("Error al inicializar la tarjeta SD. Reiniciando en 5 segundos...");
        delay(5000);
        //ESP.restart();  // Reinicia el ESP32 si la SD no se monta correctamente
    }

    Serial.println("Tarjeta SD montada correctamente.");

    // Listar archivos en la SD para diagnóstico
    File root = SD.open("/");
    while (File file = root.openNextFile()) {
        String fileName = file.name();
        Serial.print("Archivo encontrado: ");
        Serial.println(fileName);

        // Intentar abrir cada archivo encontrado con ruta absoluta
        Serial.print("Intentando abrir archivo: /");
        Serial.println(fileName);
        File testFile = SD.open("/" + fileName, FILE_READ);
        if (testFile) {
            Serial.println("✅ Archivo abierto correctamente.");
            testFile.close();
        } else {
            Serial.println("❌ Error: No se pudo abrir el archivo.");
        }

        file.close();
    }

    // Ruta principal para listar archivos
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        String page = "<html><body><h2>Archivos en la tarjeta SD</h2><ul>";
        File root = SD.open("/");
        File file = root.openNextFile();

        while (file) {
            String fileName = file.name();
            Serial.print("Agregando archivo a la lista: ");
            Serial.println(fileName);
            
            page += "<li><a href='/download?file=";
            page += fileName;
            page += "'>";
            page += fileName;
            page += "</a></li>";
            file.close();
            file = root.openNextFile();
        }

        page += "</ul></body></html>";
        request->send(200, "text/html", page);
    });

    // Ruta para descargar archivos
    server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("file")) {
            String fileName = request->getParam("file")->value();
            Serial.print("Intentando abrir archivo: /");
            Serial.println(fileName);

            File file = SD.open("/" + fileName, FILE_READ);
            if (file) {
                request->send(SD, "/" + fileName, "application/octet-stream", true);
                file.close();
            } else {
                Serial.println("Error: Archivo no encontrado en la SD.");
                request->send(404, "text/plain", "Archivo no encontrado.");
            }
        } else {
            Serial.println("Error: Nombre de archivo no especificado.");
            request->send(400, "text/plain", "Nombre de archivo no especificado.");
        }
    });

    server.begin();
    Serial.println("Servidor Web Iniciado");
}

void loop() {
    // No se requiere código en el loop, el servidor web maneja todo
}