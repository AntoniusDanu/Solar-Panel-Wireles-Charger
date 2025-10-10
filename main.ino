#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define I2C_MASTER_FREQ_HZ 100000
#define INA219_ADDR 0x40
#define I2C_INPUT_SDA   16
#define I2C_INPUT_SCL   17
#define I2C_OUTPUT_SDA  21
#define I2C_OUTPUT_SCL  19
#define I2C_LCD_SDA     22
#define I2C_LCD_SCL     23

TwoWire I2CInput  = TwoWire(0);
TwoWire I2COutput = TwoWire(1);
TwoWire I2CLcd    = TwoWire(2);  

LiquidCrystal_I2C lcd(0x27, 16, 2);

// variabel global (shared antara task dan loop)
float vin_bus, vin_curr, vin_pwr, vout_bus, vout_curr, vout_pwr;

// variabel LSB kalibrasi (global)
float INA_current_LSB_input = 0.0f;
float INA_power_LSB_input = 0.0f;
float INA_current_LSB_output = 0.0f;
float INA_power_LSB_output = 0.0f;

// ====================== Fungsi Kalibrasi INA219 ======================
void ina219_calibrate_auto(TwoWire &i2c, float R_shunt_ohm, float max_expected_current_A,
                           float &out_current_LSB, float &out_power_LSB) {
  // hitung LSB arus dan power
  float current_LSB = max_expected_current_A / 32768.0f; // A/bit
  uint32_t cal = (uint32_t)(0.04096f / (current_LSB * R_shunt_ohm));
  if (cal > 0xFFFF) {
    current_LSB = 0.04096f / (0xFFFF * R_shunt_ohm);
    cal = 0xFFFF;
  }
  // tulis ke register kalibrasi
  i2c.beginTransmission(INA219_ADDR);
  i2c.write(0x05);
  i2c.write((cal >> 8) & 0xFF);
  i2c.write(cal & 0xFF);
  i2c.endTransmission();

  // simpan hasil LSB
  out_current_LSB = current_LSB;
  out_power_LSB = 20.0f * current_LSB;
}
// =====================================================================

int ina219_read_register(TwoWire &i2c, uint8_t reg) {
  uint8_t data_rd[2];
  i2c.beginTransmission(INA219_ADDR);
  i2c.write(reg);
  if (i2c.endTransmission(false) != 0) return -1;
  if (i2c.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2) != 2) return -1;
  data_rd[0] = i2c.read();
  data_rd[1] = i2c.read();
  return (data_rd[0] << 8) | data_rd[1];
}

float ina219_get_shunt_voltage(TwoWire &i2c) {
  int reg = ina219_read_register(i2c, 0x01);
  if (reg < 0) return -1.0f;
  int16_t val = (int16_t)reg;
  return val * 0.00001f;
}

float ina219_get_bus_voltage(TwoWire &i2c) {
  int reg = ina219_read_register(i2c, 0x02);
  if (reg < 0) return -1.0f;
  return ((reg >> 3) & 0x1FFF) * 0.004f;
}

float ina219_get_power(TwoWire &i2c) {
  int reg = ina219_read_register(i2c, 0x03);
  if (reg < 0) return -1.0f;
  if (&i2c == &I2CInput)
    return reg * INA_power_LSB_input * 1000.0f; // mW
  else
    return reg * INA_power_LSB_output * 1000.0f; // mW
}

float ina219_get_current(TwoWire &i2c) {
  int reg = ina219_read_register(i2c, 0x04);
  if (reg < 0) return -1.0f;
  int16_t val = (int16_t)reg;
  if (&i2c == &I2CInput)
    return val * INA_current_LSB_input * 1000.0f; // mA
  else
    return val * INA_current_LSB_output * 1000.0f; // mA
}

// Task pembacaan INA219
void readTask(void *pvParameters) {
  for (;;) {
    vin_bus  = ina219_get_bus_voltage(I2CInput);
    vin_curr = ina219_get_current(I2CInput);
    vin_pwr  = ina219_get_power(I2CInput);

    vout_bus  = ina219_get_bus_voltage(I2COutput);
    vout_curr = ina219_get_current(I2COutput);
    vout_pwr  = ina219_get_power(I2COutput);

    Serial.printf("INPUT  : Bus=%.3f V | Current=%.3f mA | Power=%.3f mW\n",
                  vin_bus, vin_curr, vin_pwr);
    Serial.printf("OUTPUT : Bus=%.3f V | Current=%.3f mA | Power=%.3f mW\n",
                  vout_bus, vout_curr, vout_pwr);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting INA219 dual I2C example + LCD on separate bus");

  // Inisialisasi dua bus INA219
  I2CInput.begin(I2C_INPUT_SDA, I2C_INPUT_SCL, I2C_MASTER_FREQ_HZ);
  I2COutput.begin(I2C_OUTPUT_SDA, I2C_OUTPUT_SCL, I2C_MASTER_FREQ_HZ);

  // Kalibrasi otomatis tiap sensor
  float R_shunt = 0.1f; // ukur dengan multimeter
  ina219_calibrate_auto(I2CInput, R_shunt, 1.0f, INA_current_LSB_input, INA_power_LSB_input);
  ina219_calibrate_auto(I2COutput, R_shunt, 1.0f, INA_current_LSB_output, INA_power_LSB_output);

  // Inisialisasi bus terpisah untuk LCD
  I2CLcd.begin(I2C_LCD_SDA, I2C_LCD_SCL, I2C_MASTER_FREQ_HZ);
  lcd.init();
  lcd.backlight();

  // Jalankan task FreeRTOS untuk pembacaan sensor
  xTaskCreatePinnedToCore(readTask, "INA219_ReadTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Update tampilan LCD setiap 2 detik
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("In:%.2fV %.1fmA", vin_bus, vin_curr);
  lcd.setCursor(0, 1);
  lcd.printf("Out:%.2fV %.1fmA", vout_bus, vout_curr);
  delay(2000);
}
