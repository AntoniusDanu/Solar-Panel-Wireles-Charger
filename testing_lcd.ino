#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // ganti 0x27 sesuai hasil scanner

void setup() {
  Wire.begin(16, 17); // SDA=16, SCL=17
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("=== LCD TEST ===");
  lcd.setCursor(0, 1);
  lcd.print("I2C OK & WORKING");
  lcd.setCursor(0, 2);
  lcd.print("Check 20x4 lines");
  lcd.setCursor(0, 3);
  lcd.print("Line 4 visible?");
}

void loop() {
  // nothing
}
