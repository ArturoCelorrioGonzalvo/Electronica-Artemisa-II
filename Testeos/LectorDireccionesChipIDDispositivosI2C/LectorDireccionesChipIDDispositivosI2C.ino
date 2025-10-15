/**
 * @file I2C_Scanner_and_ChipID_Verifier.ino
 * @brief Escanea el bus I2C para encontrar dispositivos y verifica sus registros
 * de identificación (WHO_AM_I) para confirmar que son los sensores correctos.
 * 
 * Direcciones y Chip IDs esperados para el proyecto Artemisa II:
 * -----------------------------------------------------------------------------
 * Sensor      | Chip(s)          | Dirección I2C | Registro ID | Valor Esperado
 * -----------------------------------------------------------------------------
 * Barómetro   | BMP280           | 0x76 (o 0x77) | 0xD0        | 0x58
 * Acel/Mag    | LSM303D          | 0x1E (o 0x1D) | 0x0F        | 0x49
 * Giroscopio  | L3GD20           | 0x69 (o 0x6B) | 0x0F        | 0xD3, 0xD4, o 0xD7
 * -----------------------------------------------------------------------------
 */

#include <Wire.h>

// --- DEFINE AQUÍ LOS PINES I2C DE TU PROYECTO ---
#define SDA_PIN 5
#define SCL_PIN 4

void setup() {
  Serial.begin(115200);
  // Esperamos a que el monitor serie se conecte.
  while (!Serial); 
  
  Serial.println("\n--- Escáner y Verificador de Chip ID I2C ---");
  Serial.println("Inicializando bus I2C en pines SDA: " + String(SDA_PIN) + ", SCL: " + String(SCL_PIN));

  // Inicializa el bus I2C con los pines especificados.
  Wire.begin(SDA_PIN, SCL_PIN);
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("\nEscaneando bus I2C...");
  nDevices = 0;
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) { // Un dispositivo ha respondido.
      nDevices++;
      Serial.print("-> Dispositivo encontrado en 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);

      // --- VERIFICACIÓN DE IDENTIDAD BASADA EN LA DIRECCIÓN ---
      byte chipID = 0;
      bool idRead = false;

      switch (address) {
        case 0x76:
        case 0x77: // Direcciones del Barómetro BMP280
          chipID = readRegister(address, 0xD0);
          idRead = true;
          Serial.print(" | ID: 0x"); Serial.print(chipID, HEX);
          if (chipID == 0x58) Serial.print(" (¡Coincide con BMP280!)");
          else if (chipID == 0x60) Serial.print(" (¡Coincide con BME280!)");
          break;

        case 0x1D:
        case 0x1E: // Direcciones del Acelerómetro/Magnetómetro LSM303D
          chipID = readRegister(address, 0x0F);
          idRead = true;
          Serial.print(" | ID: 0x"); Serial.print(chipID, HEX);
          if (chipID == 0x49) Serial.print(" (¡Coincide con LSM303D!)");
          break;

        case 0x69:
        case 0x6B: // Direcciones del Giroscopio L3G
          chipID = readRegister(address, 0x0F);
          idRead = true;
          Serial.print(" | ID: 0x"); Serial.print(chipID, HEX);
          if (chipID == 0xD4) {
            Serial.print(" (¡Coincide con L3GD20!)");
          }
          break;
          
        default:
          Serial.print(" | Dispositivo no reconocido en este proyecto.");
          break;
      }
      
      if (idRead && chipID == 0x00) {
        Serial.print(" -> Error de lectura del registro.");
      }
      Serial.println();
    }
  }

  if (nDevices == 0) {
    Serial.println("-> No se encontraron dispositivos I2C. Revisa el cableado.");
  } else {
    Serial.print("Escaneo finalizado. Encontrados ");
    Serial.print(nDevices);
    Serial.println(" dispositivos.");
  }

  delay(5000); // Espera 5 segundos antes de volver a escanear.
}

/**
 * @brief Función auxiliar para leer un registro de 8 bits de un dispositivo I2C.
 * @param deviceAddress La dirección I2C del dispositivo.
 * @param regAddress La dirección del registro a leer.
 * @return El valor del registro, o 0 si hay un error.
 */
byte readRegister(byte deviceAddress, byte regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  if (Wire.endTransmission() != 0) {
    return 0; // Error de comunicación
  }
  
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  
  return 0; // No se recibieron datos
}