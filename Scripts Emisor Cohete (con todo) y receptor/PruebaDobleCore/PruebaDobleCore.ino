/**
 * @file PruebaDobleCore.ino
 * @brief Script de prueba para demostrar el funcionamiento de dos núcleos en un ESP32.
 * - El Núcleo 1 (loop principal de Arduino) ejecuta una tarea rápida y constante.
 * - El Núcleo 0 ejecuta una tarea lenta y bloqueante.
 * El objetivo es observar que la tarea lenta del Núcleo 0 NO afecta
 * a la tarea rápida del Núcleo 1.
 */

// --- Inclusión de la librería para FreeRTOS (ya viene con el ESP32) ---
#include <Arduino.h>

// --- Definiciones y Variables Globales para FreeRTOS ---
QueueHandle_t dataQueue;    // "Buzón" para pasar datos entre los núcleos.
SemaphoreHandle_t spiMutex; // "Llave" para proteger recursos compartidos (simulado).

// Estructura de datos simple para pasar entre núcleos
struct DataPacket {
  int counter;
  unsigned long timestamp;
};

// --- Tarea Lenta (Ingeniero de Vuelo) ---
/**
 * @brief Esta función se ejecutará en un bucle infinito en el Núcleo 0.
 * Simula una operación lenta, como escribir en una tarjeta SD.
 * @param pvParameters Parámetros de la tarea (no se usan aquí).
 */
void slowTask(void *pvParameters) {
  DataPacket receivedPacket; // Variable para guardar el paquete recibido

  Serial.printf("INFO: Tarea lenta iniciada en el Núcleo %d\n", xPortGetCoreID());

  while (true) {
    // La tarea se "duerme" aquí, sin consumir CPU, hasta que algo llega a la cola.
    if (xQueueReceive(dataQueue, &receivedPacket, portMAX_DELAY) == pdPASS) {
      unsigned long  tiempo = millis();

      // --- SIMULACIÓN DE TRABAJO LENTO ---
      Serial.printf("[%lu ms] Core %d (Lento): Paquete #%d recibido. Empezando escritura lenta...\n",
                    millis() - tiempo, xPortGetCoreID(), receivedPacket.counter);
      
      // Bloqueamos el "recurso SPI" simulado
      xSemaphoreTake(spiMutex, portMAX_DELAY);
      
      // ¡ESTA ES LA SIMULACIÓN DE LA ESCRITURA LENTA EN LA SD!
      if(((int)millis())%1000 == 0){
        delay(124);
      }
      
      // Liberamos el recurso
      xSemaphoreGive(spiMutex);

      Serial.printf("[%lu ms] Core %d (Lento): Escritura finalizada.\n",
                    millis() - tiempo, xPortGetCoreID());
    }
  }
}

// --- Configuración Inicial (se ejecuta en el Núcleo 1) ---
void setup() {
  Serial.begin(115200);
  delay(1000); // Dar tiempo a que el monitor serie se conecte.

  Serial.printf("\n--- PRUEBA DE DOBLE NÚCLEO --- \n");
  Serial.printf("INFO: setup() se está ejecutando en el Núcleo %d\n", xPortGetCoreID());

  // 1. Crear la cola para comunicar los núcleos.
  // Puede almacenar 100 paquetes de tipo DataPacket.
  //Caso muy extremo, pero es que parece que se llena incluso con 20 paquetes
  dataQueue = xQueueCreate(20, sizeof(DataPacket));
  
  // 2. Crear el mutex para proteger recursos compartidos.
  spiMutex = xSemaphoreCreateMutex();

  if (dataQueue == NULL || spiMutex == NULL) {
    Serial.println("!!! ERROR: No se pudo crear la cola o el mutex. Deteniendo. !!!");
    while(1);
  }

  // 3. Crear y "clavar" (pin) la tarea lenta al Núcleo 0.
  xTaskCreatePinnedToCore(
      slowTask,         // Función que ejecutará la tarea
      "Slow_SD_Writer", // Nombre de la tarea (para depuración)
      2048,             // Tamaño de la pila (stack) en bytes
      NULL,             // Parámetros de la tarea (ninguno)
      1,                // Prioridad de la tarea (1 es baja)
      NULL,             // Handle de la tarea (no necesario)
      0);               // ¡El núcleo donde se ejecutará! (0)

  Serial.println("INFO: Tarea lenta asignada al Núcleo 0. El loop() se ejecutará en el Núcleo 1.");
  Serial.println("--------------------------------------");
}

// --- Bucle Principal (Piloto) ---
/**
 * @brief El loop() de Arduino se ejecuta por defecto en el Núcleo 1.
 * Simula una tarea rápida y constante, como leer sensores y volar el cohete.
 */
void loop() {
  static int packetCounter = 0; // Contador estático para los paquetes

  // Creamos el paquete de datos a enviar
  DataPacket packetToSend;
  packetToSend.counter = packetCounter++;
  packetToSend.timestamp = millis();

  // Enviamos el paquete a la cola. Esta operación es casi instantánea.
  xQueueSend(dataQueue, &packetToSend, portMAX_DELAY);
  
  // Imprimimos un mensaje CADA VEZ que el loop se ejecuta para demostrar que no está bloqueado.
  Serial.printf("[%lu ms] Core %d (Rápido): Paquete #%d enviado a la cola.\n",
                millis() - packetToSend.timestamp, xPortGetCoreID(), packetToSend.counter);
  
  // Simulamos el trabajo del Piloto (50 Hz)
  delay(20);
}