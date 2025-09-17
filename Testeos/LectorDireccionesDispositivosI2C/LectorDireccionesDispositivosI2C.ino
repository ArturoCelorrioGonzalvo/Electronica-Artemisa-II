#include <Wire.h>

// Define aquí los pines I2C que estás usando
#define SDA_PIN 5
#define SCL_PIN 4

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  Serial.println("\n--- Buscador de Dispositivos I2C ---");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Buscando dispositivos en el bus I2C...");
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado en la dirección 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      
      // // Añadimos una comprobación para identificar los dispositivos conocidos
      // if (address == 0x1D || address == 0x1E) Serial.print("  <-- Probablemente LSM303 (Acel/Mag)");
      // if (address == 0x6A || address == 0x6B) Serial.print("  <-- Probablemente L3GD20 (Giroscopio)");
      // if (address == 0x76 || address == 0x77) Serial.print("  <-- Probablemente BMP280 (Barómetro)");
      // Serial.println();
      
      nDevices++;
    }
  }
  if (nDevices == 0) {
    Serial.println("No se encontraron dispositivos I2C. Revisa el cableado (SDA, SCL, VCC, GND).");
  } else {
    Serial.print("Búsqueda completada. Se encontraron ");
    Serial.print(nDevices);
    Serial.println(" dispositivos.");
  }
  Serial.println("----------------------------------------\n");
  delay(5000); 
}