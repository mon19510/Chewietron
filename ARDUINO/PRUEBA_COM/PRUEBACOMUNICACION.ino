#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Escaneando dispositivos I2C...");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado en la dirección 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Error en la dirección 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No se encontraron dispositivos I2C\n");
  else
    Serial.println("Escaneo completo\n");

  delay(5000);  // Escanear cada 5 segundos
}
