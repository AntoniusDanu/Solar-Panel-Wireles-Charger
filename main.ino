/* sketch.ino
   Konversi dari ESP-IDF main.c ke Arduino IDE (.ino) dengan FreeRTOS task
*/

#include <Arduino.h>
#include <Wire.h>

// Tag untuk debug
const char *TAG = "INA219-2SENSORS";

// I2C settings
#define I2C_MASTER_FREQ_HZ 100000

// INA219 default address
#define INA219_ADDR 0x40

// I2C0 (Input sensor)
#define I2C_INPUT_SDA  16
#define I2C_INPUT_SCL  17

// I2C1 (Output sensor)
#define I2C_OUTPUT_SDA  21
#define I2C_OUTPUT_SCL  19

// TwoWire instances for two I2C buses (ESP32 Arduino supports multiple TwoWire)
TwoWire I2CInput = TwoWire(0);
TwoWire I2COutput = TwoWire(1);

// Forward declarations
int ina219_read_register(TwoWire &i2c, uint8_t reg);
float ina219_get_shunt_voltage(TwoWire &i2c);
float ina219_get_bus_voltage(TwoWire &i2c);
float ina219_get_power(TwoWire &i2c);
float ina219_get_current(TwoWire &i2c);

void readTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    float vin_bus  = ina219_get_bus_voltage(I2CInput);
    float vin_shnt = ina219_get_shunt_voltage(I2CInput);
    float vin_curr = ina219_get_current(I2CInput);
    float vin_pwr  = ina219_get_power(I2CInput);

    float vout_bus  = ina219_get_bus_voltage(I2COutput);
    float vout_shnt = ina219_get_shunt_voltage(I2COutput);
    float vout_curr = ina219_get_current(I2COutput);
    float vout_pwr  = ina219_get_power(I2COutput);

    // Print similar style to ESP_LOGI
    Serial.printf("INPUT  : Bus=%.3f V | Shunt=%.6f V | Current=%.3f mA | Power=%.3f mW\n",
                  vin_bus, vin_shnt, vin_curr, vin_pwr);
    Serial.printf("OUTPUT : Bus=%.3f V | Shunt=%.6f V | Current=%.3f mA | Power=%.3f mW\n",
                  vout_bus, vout_shnt, vout_curr, vout_pwr);

    vTaskDelay(pdMS_TO_TICKS(2000)); // delay 2000 ms
  }
}

// ----------------- I2C helper functions -----------------

// Read 16-bit register from INA219. Returns int (0..65535) or -1 on error
int ina219_read_register(TwoWire &i2c, uint8_t reg) {
  uint8_t data_rd[2];

  // Write register pointer (use repeated start)
  i2c.beginTransmission(INA219_ADDR);
  i2c.write(reg);
  uint8_t err = i2c.endTransmission(false); // false -> send restart
  if (err != 0) {
    // transmission failed
    Serial.printf("i2c write reg 0x%02X failed err=%d\n", reg, err);
    return -1;
  }

  // Request 2 bytes
  uint8_t toRead = 2;
  uint8_t got = i2c.requestFrom((uint8_t)INA219_ADDR, toRead);
  if (got != toRead) {
    Serial.printf("i2c read reg 0x%02X failed got=%d\n", reg, got);
    return -1;
  }

  data_rd[0] = i2c.read();
  data_rd[1] = i2c.read();

  int value = (data_rd[0] << 8) | data_rd[1];
  return value;
}

// High-level INA219 functions (units same asumsi seperti kode asli)

// Shunt voltage: 10uV/LSB => multiply by 0.00001 (V)
float ina219_get_shunt_voltage(TwoWire &i2c) {
  int reg = ina219_read_register(i2c, 0x01);
  if (reg < 0) return -1.0f;
  int16_t val = (int16_t)reg;
  return val * 0.00001f;
}

// Bus voltage: bits [15:3], 4mV/LSB => multiply by 0.004 (V)
float ina219_get_bus_voltage(TwoWire &i2c) {
  int reg = ina219_read_register(i2c, 0x02);
  if (reg < 0) return -1.0f;
  uint16_t val = (reg >> 3) & 0x1FFF;
  return val * 0.004f;
}

// Power register: LSB = 20mW => multiply by 20 (mW)
float ina219_get_power(TwoWire &i2c) {
  int reg = ina219_read_register(i2c, 0x03);
  if (reg < 0) return -1.0f;
  return reg * 20.0f; // mW
}

// Current register: assuming calibration LSB = 1mA => multiply by 0.001 (A) but original prints mA
float ina219_get_current(TwoWire &i2c) {
  int reg = ina219_read_register(i2c, 0x04);
  if (reg < 0) return -1.0f;
  int16_t val = (int16_t)reg;
  return val * 0.001f * 1000.0f; // convert to mA for parity with original print (val*0.001 A -> *1000 => mA)
  // Note: original returned val*0.001f and then printed as "mA" — to match numeric scale we return mA.
}

// ----------------- Arduino setup & loop -----------------

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.println("Starting INA219 dual I2C example (Arduino + FreeRTOS)");

  // Initialize two I2C buses
  // On ESP32 Arduino, TwoWire::begin(sda, scl, frequency)
  I2CInput.begin(I2C_INPUT_SDA, I2C_INPUT_SCL, I2C_MASTER_FREQ_HZ);
  I2COutput.begin(I2C_OUTPUT_SDA, I2C_OUTPUT_SCL, I2C_MASTER_FREQ_HZ);

  // Optionally set clock (redundant if provided in begin)
  I2CInput.setClock(I2C_MASTER_FREQ_HZ);
  I2COutput.setClock(I2C_MASTER_FREQ_HZ);

  // Create a FreeRTOS task for reading sensors
  
  BaseType_t ret = xTaskCreatePinnedToCore(
    readTask,           // task function
    "INA219_ReadTask",  // name
    4096,               // stack size in bytes
    NULL,               // parameter
    1,                  // priority
    NULL,               // handle
    1                   // pinned core (1) - change or use xTaskCreate if not needed
  );

  if (ret != pdPASS) {
    Serial.println("Failed to create readTask");
  }
}

void loop() {
  // Nothing in loop - work done inside FreeRTOS task
  vTaskDelay(pdMS_TO_TICKS(1000));
}
