#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define I2C_INPUT_SDA  16
#define I2C_INPUT_SCL  17
#define I2C_OUTPUT_SDA 21
#define I2C_OUTPUT_SCL 19
#define I2C_FREQ       100000

#define INA219_ADDR_INPUT  0x40
#define INA219_ADDR_OUTPUT 0x40   // Jika kedua sensor sama, ganti alamat salah satu (misal 0x41)

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variabel global pembacaan
float vin_bus = 0, vin_curr = 0, vin_pwr = 0;
float vout_bus = 0, vout_curr = 0, vout_pwr = 0;

// Variabel kalibrasi
float INA_current_LSB_input = 0.0f;
float INA_power_LSB_input   = 0.0f;
float INA_current_LSB_output = 0.0f;
float INA_power_LSB_output   = 0.0f;

// ====================== Fungsi Kalibrasi ======================
void ina219_calibrate_auto(TwoWire &wire, uint8_t addr, float R_shunt_ohm, float max_expected_current_A,
                           float &out_current_LSB, float &out_power_LSB) {
  float current_LSB = max_expected_current_A / 32768.0f; // A/bit
  uint32_t cal = (uint32_t)(0.04096f / (current_LSB * R_shunt_ohm));
  if (cal > 0xFFFF) {
    current_LSB = 0.04096f / (0xFFFF * R_shunt_ohm);
    cal = 0xFFFF;
  }

  wire.beginTransmission(addr);
  wire.write(0x05);
  wire.write((cal >> 8) & 0xFF);
  wire.write(cal & 0xFF);
  wire.endTransmission();

  out_current_LSB = current_LSB;
  out_power_LSB   = 20.0f * current_LSB;
}

// ====================== Fungsi Baca Register ======================
int ina219_read_register(TwoWire &wire, uint8_t addr, uint8_t reg) {
  wire.beginTransmission(addr);
  wire.write(reg);
  if (wire.endTransmission(false) != 0) return -1;
  if (wire.requestFrom(addr, (uint8_t)2) != 2) return -1;
  uint8_t msb = wire.read();
  uint8_t lsb = wire.read();
  return (msb << 8) | lsb;
}

float ina219_get_bus_voltage(TwoWire &wire, uint8_t addr) {
  int reg = ina219_read_register(wire, addr, 0x02);
  if (reg < 0) return -1.0f;
  return ((reg >> 3) & 0x1FFF) * 0.004f; // Volt
}

float ina219_get_current(TwoWire &wire, uint8_t addr, float current_LSB) {
  int reg = ina219_read_register(wire, addr, 0x04);
  if (reg < 0) return -1.0f;
  int16_t val = (int16_t)reg;
  return val * current_LSB * 1000.0f; // mA
}

float ina219_get_power(TwoWire &wire, uint8_t addr, float power_LSB) {
  int reg = ina219_read_register(wire, addr, 0x03);
  if (reg < 0) return -1.0f;
  return reg * power_LSB * 1000.0f; // mW
}

// ====================== Task Pembacaan Sensor ======================
void readTask(void *pvParameters) {
  for (;;) {
    vin_bus  = ina219_get_bus_voltage(Wire, INA219_ADDR_INPUT);
    vin_curr = ina219_get_current(Wire, INA219_ADDR_INPUT, INA_current_LSB_input);
    vin_pwr  = ina219_get_power(Wire, INA219_ADDR_INPUT, INA_power_LSB_input);

    vout_bus  = ina219_get_bus_voltage(Wire1, INA219_ADDR_OUTPUT);
    vout_curr = ina219_get_current(Wire1, INA219_ADDR_OUTPUT, INA_current_LSB_output);
    vout_pwr  = ina219_get_power(Wire1, INA219_ADDR_OUTPUT, INA_power_LSB_output);

    Serial.printf("INPUT  : %.3fV %.3fmA %.3fmW\n", vin_bus, vin_curr, vin_pwr);
    Serial.printf("OUTPUT : %.3fV %.3fmA %.3fmW\n\n", vout_bus, vout_curr, vout_pwr);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Starting Dual I2C Bus: Wire + Wire1");

  // Inisialisasi dua bus I2C
  Wire.begin(I2C_INPUT_SDA, I2C_INPUT_SCL, I2C_FREQ);   // Bus 0: LCD + INA219 Input
  Wire1.begin(I2C_OUTPUT_SDA, I2C_OUTPUT_SCL, I2C_FREQ); // Bus 1: INA219 Output

  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();

  // Kalibrasi kedua INA219
  float R_shunt = 0.1f;
  ina219_calibrate_auto(Wire,  INA219_ADDR_INPUT,  R_shunt, 1.0f,
                        INA_current_LSB_input,  INA_power_LSB_input);
  ina219_calibrate_auto(Wire1, INA219_ADDR_OUTPUT, R_shunt, 1.0f,
                        INA_current_LSB_output, INA_power_LSB_output);

  // Jalankan task baca sensor
  xTaskCreatePinnedToCore(readTask, "INA219_ReadTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("In:%.2fV %.1fmA", vin_bus, vin_curr);
  lcd.setCursor(0, 1);
  lcd.printf("Out:%.2fV %.1fmA", vout_bus, vout_curr);
  delay(2000);
}
