#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>

#define I2C_INPUT_SDA 16
#define I2C_INPUT_SCL 17

// Ubah SCL kedua ke 22 (lebih aman untuk ESP32). Gunakan SDA 21.
#define I2C_OUTPUT_SDA 21
#define I2C_OUTPUT_SCL 22
#define I2C_FREQ 100000

#define INA219_ADDR_INPUT 0x40
#define INA219_ADDR_OUTPUT 0x41

LiquidCrystal_I2C lcd(0x27, 20, 4);

// === URL Google Apps Script ===
String scriptURL = "https://script.google.com/macros/s/AKfycbz7dw2xFqCBaLJ3yvETnD4z_scdSozUKZ1sCDkfBYtpaUrbv4uUPxssSTurR1uP2U/exec";

// WiFi (sesuaikan)
const char* ssid = "WIDIYA";
const char* password = "rt05rw02no12a";

// Variabel global pembacaan
float vin_bus = 0, vin_curr = 0, vin_pwr = 0;
float vout_bus = 0, vout_curr = 0, vout_pwr = 0;

// Kalibrasi LSB (akan diisi saat kalibrasi)
float INA_current_LSB_input = 0.0f;
float INA_power_LSB_input = 0.0f;
float INA_current_LSB_output = 0.0f;
float INA_power_LSB_output = 0.0f;

// Forward declarations
void ina219_calibrate_auto(TwoWire &wire, uint8_t addr, float R_shunt_ohm,
float max_current, float &out_LSB, float &out_power_LSB);
int ina219_read_register(TwoWire &wire, uint8_t addr, uint8_t reg);
float ina219_get_bus_voltage(TwoWire &wire, uint8_t addr);
float ina219_get_current(TwoWire &wire, uint8_t addr, float LSB);
float ina219_get_power(TwoWire &wire, uint8_t addr, float LSB);
void readTask(void *pvParameters);
bool wifiConnectWithTimeout(int timeout_seconds);
void checkAndReconnectWifiPeriodically();

// Fungsi kirim data ke Google Sheets
void sendToSheet(float vin_v, float vin_mA, float vin_mW,
float vout_v, float vout_mA, float vout_mW) {

if (WiFi.status() != WL_CONNECTED) {
Serial.println("WiFi belum terhubung! (data tidak dikirim)");
return;
}

HTTPClient http;

String url = scriptURL +
"?vin_v=" + String(vin_v, 3) +
"&vin_mA=" + String(vin_mA, 3) +
"&vin_mW=" + String(vin_mW, 3) +
"&vout_v=" + String(vout_v, 3) +
"&vout_mA=" + String(vout_mA, 3) +
"&vout_mW=" + String(vout_mW, 3);

http.begin(url);
int httpCode = http.GET();

if (httpCode > 0) {
Serial.println("Data terkirim ke spreadsheet!");
} else {
Serial.printf("Gagal kirim: %s\n", http.errorToString(httpCode).c_str());
}
http.end();
}

// ====================== Kalibrasi INA219 ======================
void ina219_calibrate_auto(TwoWire &wire, uint8_t addr, float R_shunt_ohm,
float max_current, float &out_LSB, float &out_power_LSB) {

float current_LSB = max_current / 32768.0f; // A/bit
uint32_t cal = (uint32_t)(0.04096f / (current_LSB * R_shunt_ohm));
if (cal > 0xFFFF) {
current_LSB = 0.04096f / (0xFFFF * R_shunt_ohm);
cal = 0xFFFF;
}

// tulis register CALIBRATION (0x05)
wire.beginTransmission(addr);
wire.write(0x05);
wire.write((cal >> 8) & 0xFF);
wire.write(cal & 0xFF);
wire.endTransmission();

out_LSB = current_LSB;
out_power_LSB = 20.0f * current_LSB;
}

// ====================== Baca Register INA219 ======================
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
return ((reg >> 3) & 0x1FFF) * 0.004f; // V
}

float ina219_get_current(TwoWire &wire, uint8_t addr, float LSB) {
int reg = ina219_read_register(wire, addr, 0x04);
if (reg < 0) return -1.0f;
int16_t val = (int16_t)reg;
return fabs(val * LSB * 1000.0f); // mA (fabs agar positif)
}

float ina219_get_power(TwoWire &wire, uint8_t addr, float LSB) {
int reg = ina219_read_register(wire, addr, 0x03);
if (reg < 0) return -1.0f;
return reg * LSB * 1000.0f; // mW
}

// ====================== Task Pembacaan Sensor ======================
void readTask(void *pvParameters) {
(void) pvParameters;
for (;;) {
// baca input di bus Wire (I2C_INPUT)
vin_bus = ina219_get_bus_voltage(Wire, INA219_ADDR_INPUT);
vin_curr = ina219_get_current(Wire, INA219_ADDR_INPUT, INA_current_LSB_input);
vin_pwr = vin_bus * (vin_curr / 1000.0f);

// baca output di bus Wire1 (I2C_OUTPUT)
vout_bus  = ina219_get_bus_voltage(Wire1, INA219_ADDR_OUTPUT);
vout_curr = ina219_get_current(Wire1, INA219_ADDR_OUTPUT, INA_current_LSB_output);
vout_pwr  = vout_bus * (vout_curr / 1000.0f);

Serial.printf("INPUT  : %.3fV %.3fmA %.3fmW\n", vin_bus, vin_curr, vin_pwr * 1000);
Serial.printf("OUTPUT : %.3fV %.3fmA %.3fmW\n\n", vout_bus, vout_curr, vout_pwr * 1000);

// Kirim data (fungsi akan memeriksa status WiFi)
sendToSheet(vin_bus, vin_curr, vin_pwr * 1000,
            vout_bus, vout_curr, vout_pwr * 1000);

vTaskDelay(pdMS_TO_TICKS(2000)); // jeda 2 detik
}
}

// ====================== WiFi helper ======================
bool wifiConnectWithTimeout(int timeout_seconds) {
Serial.printf("Connecting to WiFi SSID: %s\n", ssid);
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);

int elapsed = 0;
while (WiFi.status() != WL_CONNECTED && elapsed < timeout_seconds * 1000) {
delay(500);
Serial.print(".");
elapsed += 500;
}

if (WiFi.status() == WL_CONNECTED) {
Serial.println("\nConnected!");
Serial.print("IP Address: ");
Serial.println(WiFi.localIP());
return true;
} else {
Serial.println("\nFAILED to connect WiFi");
Serial.printf("WiFi.status() = %d\n", WiFi.status());
return false;
}
}

// cek reconnect periodik di loop utama
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL_MS = 30000; // coba reconnect tiap 30 detik

void setup() {
Serial.begin(115200);
delay(200);
Serial.println();
Serial.println("=== Starting Program ===");

// Inisialisasi dua jalur I2C
// Wire -> untuk INA219 input + LCD
// Wire1 -> untuk INA219 output
Wire.begin(I2C_INPUT_SDA, I2C_INPUT_SCL, I2C_FREQ);
Wire1.begin(I2C_OUTPUT_SDA, I2C_OUTPUT_SCL, I2C_FREQ);

// LCD (gunakan Wire default)
lcd.init();
lcd.backlight();
lcd.clear();
lcd.setCursor(0,0);
lcd.print("Init...");

// Coba connect WiFi dengan timeout 15 detik
bool wifiOk = wifiConnectWithTimeout(15);

// Kalibrasi INA219 (jalankan walau WiFi gagal, baca tetap berjalan)
ina219_calibrate_auto(Wire, INA219_ADDR_INPUT, 0.1f, 2.0f,
INA_current_LSB_input, INA_power_LSB_input);

ina219_calibrate_auto(Wire1, INA219_ADDR_OUTPUT, 0.1f, 2.0f,
INA_current_LSB_output, INA_power_LSB_output);

// Setelah inisialisasi, jalankan task pembacaan sensor.
// (Kita jalankan task walau WiFi belum connect agar pembacaan tetap berjalan;
// pengiriman akan otomatis gagal sampai WiFi hidup lagi — dan kita akan coba reconnect di loop())
xTaskCreatePinnedToCore(readTask, "INA219_ReadTask", 4096, NULL, 1, NULL, 1);

// simpan waktu terakhir percobaan reconnect (jika gagal)
lastReconnectAttempt = millis();
if (!wifiOk) {
lcd.clear();
lcd.setCursor(0,0);
lcd.print("WiFi:FAILED");
} else {
lcd.clear();
lcd.setCursor(0,0);
lcd.print("WiFi:OK");
}

Serial.println("Setup complete.");
}

// ====================== LOOP utama ======================
void loop() {
// tampilkan ringkasan ke LCD setiap loop (2 detik)
lcd.clear();
lcd.setCursor(0, 0);
lcd.printf("In:%.2fV %.1fmA", vin_bus, vin_curr);
lcd.setCursor(0, 1);
lcd.printf("Pin:%.2fmW", vin_pwr * 1000);

lcd.setCursor(0, 2);
lcd.printf("Out:%.2fV %.1fmA", vout_bus, vout_curr);
lcd.setCursor(0, 3);
lcd.printf("Pout:%.2fmW", vout_pwr * 1000);

// Jika WiFi belum terhubung, coba reconnect secara berkala
if (WiFi.status() != WL_CONNECTED) {
unsigned long now = millis();
if (now - lastReconnectAttempt >= RECONNECT_INTERVAL_MS) {
Serial.println("Mencoba reconnect WiFi...");
bool ok = wifiConnectWithTimeout(10);
if (ok) {
lcd.clear();
lcd.setCursor(0,0);
lcd.print("WiFi:OK");
} else {
lcd.clear();
lcd.setCursor(0,0);
lcd.print("WiFi:Retry...");
}
lastReconnectAttempt = now;
}
}

delay(2000); // sinkron dengan interval baca sensor (task)
}
