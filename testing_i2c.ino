#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define I2C_INPUT_SDA 16
#define I2C_INPUT_SCL 17
#define I2C_OUTPUT_SDA 21
#define I2C_OUTPUT_SCL 19

TwoWire Wire1 = TwoWire(1);  // Bus I2C kedua
LiquidCrystal_I2C lcd(0x27, 16, 2); // Ganti alamat sesuai hasil scan

// === Fungsi Scanner Universal ===
void I2C_ScannerWire(TwoWire &wire, const char *busName) {
  Serial.printf("Memindai perangkat I2C pada %s...\n", busName);
  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    if (wire.endTransmission() == 0) {
      Serial.printf("  Perangkat ditemukan di alamat 0x%02X\n", address);
      count++;
    }
  }
  if (count == 0) Serial.println("  Tidak ada perangkat ditemukan.");
  else Serial.printf("Total %d perangkat terdeteksi.\n", count);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Tes LCD via Wire1 ===");

  // Inisialisasi bus input
  Wire.begin(I2C_INPUT_SDA, I2C_INPUT_SCL);
  Serial.println("Wire0 siap (input)");
  I2C_ScannerWire(Wire, "Wire0");

  // Inisialisasi bus output
  Wire1.begin(I2C_OUTPUT_SDA, I2C_OUTPUT_SCL);
  Serial.println("Wire1 siap (output)");
  I2C_ScannerWire(Wire1, "Wire1");

  // Gunakan Wire1 untuk LCD
  lcd.setWire(&Wire1);
  lcd.begin(16, 2);
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("LCD OK via Wire1");
  lcd.setCursor(0, 1);
  lcd.print("SDA21 SCL19");
}

void loop() {
  lcd.setCursor(0, 1);
  lcd.print("Tes LCD Aktif  ");
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("                ");
  delay(1000);
}
