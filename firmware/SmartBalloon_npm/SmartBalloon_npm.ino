/*
 * ============================================================================
 *  ESP32 Smart Weather Balloon Telemetry System — Nimbus-1
 *  Noise-filtered | Wire.h MPU6050 (exact working test method) | Calibration
 *  MODIFIED: WiFi STA Mode (Static IP) + Pure JSON API Backend with CORS
 * ============================================================================
 *
 *  REQUIRED LIBRARIES  (Arduino IDE → Tools → Manage Libraries)
 *  ─────────────────────────────────────────────────────────────
 *  1. WiFi.h                  Built-in with ESP32 Arduino Core
 *  2. WebServer.h             Built-in with ESP32 Arduino Core
 *  3. Wire.h                  Built-in I2C library
 *  4. Adafruit BMP085 Library Search: "Adafruit BMP085" by Adafruit
 *  5. Adafruit Unified Sensor Search: "Adafruit Unified Sensor" by Adafruit
 *  6. TinyGPSPlus             Search: "TinyGPS++" by Mikal Hart
 *  7. ArduinoJson             Search: "ArduinoJson" by Benoit Blanchon (v6.x)
 *
 *  MPU6050: NO external library. Driven by the exact Wire.h method
 *           confirmed working in the standalone test sketch.
 *
 *  WIRING SUMMARY
 *  ──────────────
 *  BMP180  VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
 *  MPU6050 VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22  AD0→GND
 *  NEO-6M  VCC→3.3V  GND→GND  TX→GPIO16   RX→GPIO17
 *  LM35    VCC→3.3V  GND→GND  AO→GPIO33
 *  MQ135   VCC→5V    GND→GND  AO→[Divider]→GPIO34
 *  MQ2     VCC→5V    GND→GND  AO→[Divider]→GPIO35
 *  MQ7     VCC→5V    GND→GND  AO→[Divider]→GPIO32
 *
 *  VOLTAGE DIVIDER — MANDATORY for MQ sensors (5V AO → ESP32 3.3V ADC)
 *  ─────────────────────────────────────────────────────────────────────
 *  MQ AO ──┬── 10kΩ ──── ESP32 ADC pin
 *           └── 10kΩ ──── GND
 *  Halves 5V → 2.5V max. MQ_VD_SCALE=2 compensates in code.
 * ============================================================================
 */

// ============================================================================
//  LIBRARY INCLUDES
// ============================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// ============================================================================
//  WIFI STATION (CLIENT) CONFIGURATION — LOCKED STATIC IP
// ============================================================================
const char* STA_SSID     = "home";          
const char* STA_PASSWORD = "12345678";      

// Static IP Configuration matching your Windows Hotspot subnet
IPAddress local_IP(192, 168, 137, 250);     // Permanently locks your ESP32 to .250
IPAddress gateway(192, 168, 137, 1);       // Your hotspot's master gateway
IPAddress subnet(255, 255, 255, 0);         
IPAddress primaryDNS(192, 168, 137, 1);     // Forces stable routing
// Static IP Configuration
// IPAddress local_IP(192, 168, 0, 200);       // Desired Static IP for Nimbus-1
// IPAddress gateway(192, 168, 0, 1);          // Router Gateway
// IPAddress subnet(255, 255, 255, 0);         // Subnet Mask
// IPAddress primaryDNS(8, 8, 8, 8);       // Optional: Google DNS
// IPAddress secondaryDNS(8, 8, 4, 4);     // Optional: Google DNS

// ============================================================================
//  PIN DEFINITIONS (UNCHANGED)
// ============================================================================
#define PIN_LM35        33
#define PIN_MQ135       34
#define PIN_MQ2         35
#define PIN_MQ7         32
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define GPS_ESP_RX      16
#define GPS_ESP_TX      17
#define GPS_BAUD      9600

// ============================================================================
//  MPU6050 ADDRESS (UNCHANGED)
// ============================================================================
#define MPU_ADDR   0x68   // AD0 → GND = 0x68 | AD0 → 3.3V = 0x69

// ============================================================================
//  CALIBRATION & SCALING CONSTANTS (UNCHANGED)
// ============================================================================
#define MQ_VD_SCALE           2
#define LM35_OFFSET           0.0f

#define MQ135_AQI_MAX         500
#define MQ7_CO_PPM_MAX        1000
#define MQ2_SMOKE_PPM_MAX     1000

#define MPU_LSB_PER_G         16384.0f
#define VIBRATION_SCALE       100.0f
#define MPU_DEADBAND          50
#define MPU_AVG_SAMPLES       20
#define MPU_CALIB_SAMPLES     200

#define SENSOR_INTERVAL_MS        1000
#define SERIAL_PRINT_INTERVAL_MS  3000

// ============================================================================
//  FILTERING CONSTANTS (UNCHANGED)
// ============================================================================
#define MEDIAN_SAMPLES        9

#define EMA_ALPHA_TEMP        0.15f
#define EMA_ALPHA_AQI         0.10f
#define EMA_ALPHA_CO          0.10f
#define EMA_ALPHA_SMOKE       0.10f
#define EMA_ALPHA_ALTITUDE    0.20f
#define EMA_ALPHA_PRESSURE    0.20f
#define EMA_ALPHA_PITCH       0.25f
#define EMA_ALPHA_ROLL        0.25f
#define EMA_ALPHA_VIBRATION   0.30f
#define EMA_ALPHA_SPEED       0.20f

#define SPIKE_THRESH_TEMP     10.0f
#define SPIKE_THRESH_ALTITUDE 50.0f
#define SPIKE_THRESH_PRESSURE 10.0f
#define SPIKE_THRESH_AQI      150
#define SPIKE_THRESH_CO       200
#define SPIKE_THRESH_SMOKE    200
#define SPIKE_THRESH_PITCH    30.0f
#define SPIKE_THRESH_ROLL     30.0f

// ============================================================================
//  OBJECT INSTANCES (UNCHANGED)
// ============================================================================
Adafruit_BMP085  bmp180;
TinyGPSPlus      gps;
HardwareSerial   gpsSerial(2);
WebServer        server(80);

// ============================================================================
//  SENSOR STATUS FLAGS (UNCHANGED)
// ============================================================================
bool bmp180_ok  = false;
bool mpu6050_ok = false;

// ============================================================================
//  MPU6050 CALIBRATION OFFSETS (UNCHANGED)
// ============================================================================
long xOffset = 0;
long yOffset = 0;
long zOffset = 0;

// ============================================================================
//  EMA STATE VARIABLES (UNCHANGED)
// ============================================================================
float ema_temp      = 25.0f;
float ema_aqi       = 0.0f;
float ema_co        = 0.0f;
float ema_smoke     = 0.0f;
float ema_altitude  = 0.0f;
float ema_pressure  = 1013.25f;
float ema_pitch     = 0.0f;
float ema_roll      = 0.0f;
float ema_vibration = 0.0f;
float ema_speed     = 0.0f;

bool first_temp  = true;
bool first_alt   = true;
bool first_pres  = true;
bool first_aqi   = true;
bool first_co    = true;
bool first_smoke = true;
bool first_pitch = true;
bool first_roll  = true;

// ============================================================================
//  LIVE TELEMETRY STRUCT (UNCHANGED)
// ============================================================================
struct {
  float   altitude;
  float   temperature;
  float   pressure;
  int     aqi;
  int     co;
  int     smoke;
  double  latitude;
  double  longitude;
  float   speed;
  float   pitch;
  float   roll;
  int     vibration;
} tele;

// ============================================================================
//  TIMING (UNCHANGED)
// ============================================================================
unsigned long lastSensorRead  = 0;
unsigned long lastSerialPrint = 0;

// ============================================================================
//  FUNCTION PROTOTYPES (UNCHANGED)
// ============================================================================
void  setupWiFiSTA();        // RENAMED: was setupWiFiAP
void  setupSensors();
void  setupMPU6050();
void  readMPU(int16_t &AcX, int16_t &AcY, int16_t &AcZ);
void  readSensors();
void  printTelemetry();
void  handleRoot();
void  handleData();
void  handleOptions();
void  handleNotFound();
void  sendCORSHeaders();
int   multiSampleADC(uint8_t pin, uint8_t samples);
int   medianFilter(uint8_t pin, uint8_t samples);
float applyEMA(float newVal, float prevEMA, float alpha);
bool  spikeReject(float newVal, float prevVal, float threshold, bool isFirst);

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n============================================="));
  Serial.println(F("   Smart Weather Balloon System    "));
  Serial.println(F("  Mode: WiFi STA (Static IP) + JSON API     "));
  Serial.println(F("=============================================\n"));

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_ESP_RX, GPS_ESP_TX);

  memset(&tele, 0, sizeof(tele));

  setupSensors();
  setupWiFiSTA(); // MODIFIED: Call STA setup instead of AP setup

  // --- ROUTE REGISTRATION (UNCHANGED) ---
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/data", HTTP_OPTIONS, handleOptions);
  server.onNotFound(handleNotFound);
  
  server.begin();

  // // MODIFIED: Print new Static IP address
  // Serial.println(F("[SERVER] HTTP server started."));
  // Serial.print(F("[SERVER] API Endpoint: http://"));
  // Serial.print(local_IP); // Use the static IP variable
  // Serial.println(F("/data"));
  // UPDATED: Print the actual live dynamic IP address
  Serial.println(F("[SERVER] HTTP server started."));
  Serial.print(F("[SERVER] True API Endpoint: http://"));
  Serial.print(WiFi.localIP()); // <-- This fixes it to print the real assigned IP!
  Serial.println(F("/data"));
  Serial.println(F("[SERVER] CORS: Enabled for all origins (*)\n"));
}
  // Serial.println(F("[SERVER] CORS: Enabled for all origins (*)\n"));


// ============================================================================
//  LOOP (UNCHANGED)
// ============================================================================
void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = millis();
    readSensors();
  }

  if (millis() - lastSerialPrint >= SERIAL_PRINT_INTERVAL_MS) {
    lastSerialPrint = millis();
    printTelemetry();
  }

  server.handleClient();
}

// ============================================================================
//  setupWiFiSTA() — REPLACES setupWiFiAP()
//  Connects to "home" with Static IP 192.168.0.200
// ============================================================================
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ============================================================================
//  setupWiFiSTA() — FORCED STATIC IP MODE
// ============================================================================
void setupWiFiSTA() {
  Serial.println(F("[WiFi] Starting Station Mode (Static IP)..."));
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Apply static network parameters explicitly before connecting
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)) {
    Serial.println(F("[WiFi] WARNING: Static configuration failed! Falling back to dynamic DHCP."));
  } else {
    Serial.print(F("[WiFi] Permanent Static IP Applied: ")); Serial.println(local_IP);
  }

  Serial.print(F("[WiFi] Connecting to SSID: ")); 
  Serial.println(STA_SSID);
  WiFi.begin(STA_SSID, STA_PASSWORD);

  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 15000; 

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println(); 

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("===================================="));
    Serial.println(F("[WiFi] WI-FI CONNECTED SUCCESSFULLY!"));
    Serial.print(F("[WiFi] Locked IP Address: ")); Serial.println(WiFi.localIP());
    Serial.println(F("===================================="));
  } else {
    Serial.println(F("[WiFi] FAILED to connect. Check if your Hotspot is on."));
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void setupWiFiSTA() {
//   Serial.println(F("[WiFi]  Starting Station Mode (Client)..."));
  
//   // 1. Set WiFi Mode to Station
//   WiFi.mode(WIFI_STA);

//   // 2. Configure Static IP **before** begin()
//   //    WiFi.config(local_ip, gateway, subnet, dns1, dns2)
//   if (!WiFi.config(local_IP, gateway, subnet)) {
//     Serial.println(F("[WiFi]  WARNING: Static IP configuration failed. Check IP/Gateway/Subnet."));
//   } else {
//     Serial.print(F("[WiFi]  Static IP configured: ")); Serial.println(local_IP);
//     Serial.println("\nCONNECTED");
// Serial.print("IP: ");
// Serial.println(WiFi.localIP());

// Serial.print("Gateway: ");
// Serial.println(WiFi.gatewayIP());

// Serial.print("Subnet: ");
// Serial.println(WiFi.subnetMask());
//   }

//   // 3. Begin Connection
//   Serial.print(F("[WiFi]  Connecting to SSID: ")); Serial.println(STA_SSID);
//   WiFi.begin(STA_SSID, STA_PASSWORD);
//   while (WiFi.status() != WL_CONNECTED) {
//     Serial.print(".");
//     Serial.print(" Status=");
//     Serial.println(WiFi.status());
//     delay(1000);
// }

//   // 4. Wait for Connection with Timeout (~15 seconds)
//   unsigned long startAttemptTime = millis();
//   const unsigned long timeout = 15000; // 15s timeout

//   while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
//     Serial.print(F("."));
//     delay(500);
//   }
//   Serial.println(); // New line after dots

//   // 5. Check Result
//   if (WiFi.status() == WL_CONNECTED) {
//     Serial.println(F("[WiFi]  CONNECTED!"));
//     Serial.print(F("[WiFi]  IP Address  : ")); Serial.println(WiFi.localIP());
//     Serial.print(F("[WiFi]  Gateway     : ")); Serial.println(WiFi.gatewayIP());
//     Serial.print(F("[WiFi]  Subnet Mask : ")); Serial.println(WiFi.subnetMask());
//     Serial.print(F("[WiFi]  RSSI        : ")); Serial.print(WiFi.RSSI()); Serial.println(F(" dBm"));
//   } else {
//     Serial.println(F("[WiFi]  FAILED - Connection Timeout."));
//     Serial.println(F("[WiFi]  Check SSID/Password or Router DHCP/Static conflicts."));
//     Serial.println(F("[WiFi]  Continuing in OFFLINE mode (API unreachable)."));
//   }
// }

// ============================================================================
//  setupSensors() (UNCHANGED)
// ============================================================================
void setupSensors() {
  Serial.println(F("[SENSORS] Initialising...\n"));

  if (bmp180.begin()) {
    bmp180_ok = true;
    Serial.println(F("[BMP180]  OK  - Pressure & Altitude ready."));
  } else {
    Serial.println(F("[BMP180]  FAIL - Not detected. altitude/pressure = 0."));
  }

  setupMPU6050();

  Serial.println(F("[NEO-6M]  Listening on UART2. lat/lon/speed = 0 until fix."));
  Serial.println(F("[LM35]    GPIO33 - ready."));
  Serial.println(F("[MQ135]   GPIO34 - ready."));
  Serial.println(F("[MQ2]     GPIO35 - ready."));
  Serial.println(F("[MQ7]     GPIO32 - ready."));
  Serial.println(F("\n[SENSORS] Done.\n"));
}

// ============================================================================
//  readMPU() — EXACT WORKING METHOD (UNCHANGED)
// ============================================================================
void readMPU(int16_t &AcX, int16_t &AcY, int16_t &AcZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
}

// ============================================================================
//  setupMPU6050() — EXACT WORKING CALIBRATION (UNCHANGED)
// ============================================================================
void setupMPU6050() {
  Serial.println(F("[MPU6050] Waking sensor..."));

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(100);

  int16_t testX, testY, testZ;
  readMPU(testX, testY, testZ);

  if (testX == 0 && testY == 0 && testZ == 0) {
    Serial.println(F("[MPU6050] FAIL - all axes zero after wake. Check wiring."));
    mpu6050_ok = false;
    return;
  }

  mpu6050_ok = true;
  Serial.println(F("[MPU6050] OK  - sensor alive."));

  Serial.println(F("[MPU6050] Calibrating... Keep sensor still."));

  long sumX = 0, sumY = 0, sumZ = 0;
  int16_t AcX, AcY, AcZ;

  for (int i = 0; i < MPU_CALIB_SAMPLES; i++) {
    readMPU(AcX, AcY, AcZ);
    sumX += AcX;
    sumY += AcY;
    sumZ += AcZ;
    delay(5);
  }

  xOffset = sumX / MPU_CALIB_SAMPLES;
  yOffset = sumY / MPU_CALIB_SAMPLES;
  zOffset = sumZ / MPU_CALIB_SAMPLES;

  Serial.println(F("[MPU6050] Calibration complete."));
  Serial.print(F("[MPU6050] Offsets  X=")); Serial.print(xOffset);
  Serial.print(F("  Y="));                  Serial.print(yOffset);
  Serial.print(F("  Z="));                  Serial.println(zOffset);
}

// ============================================================================
//  readSensors() (UNCHANGED)
// ============================================================================
void readSensors() {

  // ── LM35 : Temperature ───────────────────────────────────────────────────
  {
    int   medRaw  = medianFilter(PIN_LM35, MEDIAN_SAMPLES);
    float mV      = (medRaw / 4095.0f) * 3300.0f;
    float rawTemp = (mV / 10.0f) + LM35_OFFSET;

    if (spikeReject(rawTemp, ema_temp, SPIKE_THRESH_TEMP, first_temp)) {
      first_temp = false;
      ema_temp   = applyEMA(rawTemp, ema_temp, EMA_ALPHA_TEMP);
    }
    tele.temperature = ema_temp;
  }

  // ── MQ135 : Pseudo-AQI ───────────────────────────────────────────────────
  {
    int medRaw      = medianFilter(PIN_MQ135, MEDIAN_SAMPLES);
    int compensated = constrain(medRaw * MQ_VD_SCALE, 0, 4095);
    int rawAqi      = map(compensated, 0, 4095, 0, MQ135_AQI_MAX);

    if (spikeReject((float)rawAqi, ema_aqi, (float)SPIKE_THRESH_AQI, first_aqi)) {
      first_aqi = false;
      ema_aqi   = applyEMA((float)rawAqi, ema_aqi, EMA_ALPHA_AQI);
    }
    tele.aqi = (int)roundf(ema_aqi);
  }

  // ── MQ2 : Smoke / LPG ────────────────────────────────────────────────────
  {
    int medRaw      = medianFilter(PIN_MQ2, MEDIAN_SAMPLES);
    int compensated = constrain(medRaw * MQ_VD_SCALE, 0, 4095);
    int rawSmoke    = map(compensated, 0, 4095, 0, MQ2_SMOKE_PPM_MAX);

    if (spikeReject((float)rawSmoke, ema_smoke, (float)SPIKE_THRESH_SMOKE, first_smoke)) {
      first_smoke = false;
      ema_smoke   = applyEMA((float)rawSmoke, ema_smoke, EMA_ALPHA_SMOKE);
    }
    tele.smoke = (int)roundf(ema_smoke);
  }

  // ── MQ7 : Carbon Monoxide ────────────────────────────────────────────────
  {
    int medRaw      = medianFilter(PIN_MQ7, MEDIAN_SAMPLES);
    int compensated = constrain(medRaw * MQ_VD_SCALE, 0, 4095);
    int rawCO       = map(compensated, 0, 4095, 0, MQ7_CO_PPM_MAX);

    if (spikeReject((float)rawCO, ema_co, (float)SPIKE_THRESH_CO, first_co)) {
      first_co = false;
      ema_co   = applyEMA((float)rawCO, ema_co, EMA_ALPHA_CO);
    }
    tele.co = (int)roundf(ema_co);
  }

  // ── BMP180 : Pressure & Altitude ─────────────────────────────────────────
  if (bmp180_ok) {
    float rawPa = (float)bmp180.readPressure();
    if (rawPa > 0.0f) {
      float rawPres = rawPa / 100.0f;
      float rawAlt  = 44330.0f * (1.0f - powf(rawPres / 1013.25f, 1.0f / 5.255f));

      if (spikeReject(rawPres, ema_pressure, SPIKE_THRESH_PRESSURE, first_pres)) {
        first_pres   = false;
        ema_pressure = applyEMA(rawPres, ema_pressure, EMA_ALPHA_PRESSURE);
      }
      if (spikeReject(rawAlt, ema_altitude, SPIKE_THRESH_ALTITUDE, first_alt)) {
        first_alt    = false;
        ema_altitude = applyEMA(rawAlt, ema_altitude, EMA_ALPHA_ALTITUDE);
      }
    }
    tele.pressure = ema_pressure;
    tele.altitude = ema_altitude;
  }

  // ── NEO-6M GPS ───────────────────────────────────────────────────────────
  if (gps.location.isValid() && gps.location.isUpdated()) {
    tele.latitude  = gps.location.lat();
    tele.longitude = gps.location.lng();
  }
  if (gps.speed.isValid() && gps.speed.isUpdated()) {
    float rawSpeed = (float)gps.speed.kmph();
    ema_speed      = applyEMA(rawSpeed, ema_speed, EMA_ALPHA_SPEED);
    tele.speed     = ema_speed;
  }

  // ── MPU6050 : Pitch, Roll, Vibration ─────────────────────────────────────
  if (mpu6050_ok) {
    long sumX = 0, sumY = 0, sumZ = 0;
    int16_t AcX, AcY, AcZ;

    for (int i = 0; i < MPU_AVG_SAMPLES; i++) {
      readMPU(AcX, AcY, AcZ);
      sumX += AcX;
      sumY += AcY;
      sumZ += AcZ;
      delay(2);
    }

    long avgX = (sumX / MPU_AVG_SAMPLES) - xOffset;
    long avgY = (sumY / MPU_AVG_SAMPLES) - yOffset;
    long avgZ = (sumZ / MPU_AVG_SAMPLES) - zOffset;

    if (abs(avgX) < MPU_DEADBAND) avgX = 0;
    if (abs(avgY) < MPU_DEADBAND) avgY = 0;
    if (abs(avgZ) < MPU_DEADBAND) avgZ = 0;

    Serial.print(F("[MPU6050] X=")); Serial.print(avgX);
    Serial.print(F("  Y="));        Serial.print(avgY);
    Serial.print(F("  Z="));        Serial.println(avgZ);

    float axg = avgX / MPU_LSB_PER_G;
    float ayg = avgY / MPU_LSB_PER_G;
    float azg = avgZ / MPU_LSB_PER_G;

    float rawPitch = atan2(ayg, sqrt(axg * axg + azg * azg)) * (180.0 / M_PI);
    float rawRoll  = atan2(axg, sqrt(ayg * ayg + azg * azg)) * (180.0 / M_PI);

    Serial.print(F("[MPU6050] rawPitch=")); Serial.print(rawPitch, 2);
    Serial.print(F("  rawRoll="));         Serial.println(rawRoll, 2);

    if (spikeReject(rawPitch, ema_pitch, SPIKE_THRESH_PITCH, first_pitch)) {
      first_pitch = false;
      ema_pitch   = applyEMA(rawPitch, ema_pitch, EMA_ALPHA_PITCH);
    }
    if (spikeReject(rawRoll, ema_roll, SPIKE_THRESH_ROLL, first_roll)) {
      first_roll = false;
      ema_roll   = applyEMA(rawRoll, ema_roll, EMA_ALPHA_ROLL);
    }

    float raxg = (sumX / MPU_AVG_SAMPLES - xOffset) / MPU_LSB_PER_G;
    float rayg = (sumY / MPU_AVG_SAMPLES - yOffset) / MPU_LSB_PER_G;
    float razg = (sumZ / MPU_AVG_SAMPLES - zOffset) / MPU_LSB_PER_G;
    float mag  = sqrt(raxg * raxg + rayg * rayg + razg * razg);
    float rawVib = fabs(mag - 1.0f) * VIBRATION_SCALE;
    ema_vibration = applyEMA(rawVib, ema_vibration, EMA_ALPHA_VIBRATION);

    tele.pitch     = ema_pitch;
    tele.roll      = ema_roll;
    tele.vibration = (int)constrain(ema_vibration, 0.0f, 999.0f);

    Serial.print(F("[MPU6050] pitch=")); Serial.print(tele.pitch, 2);
    Serial.print(F("  roll="));         Serial.print(tele.roll, 2);
    Serial.print(F("  vib="));          Serial.println(tele.vibration);
  }
}

// ============================================================================
//  printTelemetry() (UNCHANGED)
// ============================================================================
void printTelemetry() {
  Serial.println(F("\n+------------------------------------------+"));
  Serial.println(F("|      NIMBUS-1  TELEMETRY SNAPSHOT        |"));
  Serial.println(F("+------------------------------------------+"));
  Serial.print(F("|  [BMP180]  Altitude   : ")); Serial.print(tele.altitude, 1);    Serial.println(F(" m"));
  Serial.print(F("|  [BMP180]  Pressure   : ")); Serial.print(tele.pressure, 1);    Serial.println(F(" hPa"));
  Serial.print(F("|  [LM35]    Temperature: ")); Serial.print(tele.temperature, 1); Serial.println(F(" C"));
  Serial.print(F("|  [MQ135]   AQI        : ")); Serial.println(tele.aqi);
  Serial.print(F("|  [MQ7]     CO         : ")); Serial.print(tele.co);             Serial.println(F(" ppm"));
  Serial.print(F("|  [MQ2]     Smoke/LPG  : ")); Serial.print(tele.smoke);          Serial.println(F(" ppm"));
  Serial.print(F("|  [NEO-6M]  Latitude   : ")); Serial.println(tele.latitude,  6);
  Serial.print(F("|  [NEO-6M]  Longitude  : ")); Serial.println(tele.longitude, 6);
  Serial.print(F("|  [NEO-6M]  Speed      : ")); Serial.print(tele.speed, 1);       Serial.println(F(" km/h"));
  Serial.print(F("|  [NEO-6M]  GPS Fix    : ")); Serial.println(gps.location.isValid() ? F("YES") : F("NO - waiting..."));
  Serial.print(F("|  [MPU6050] Pitch      : ")); Serial.print(tele.pitch, 1);       Serial.println(F(" deg"));
  Serial.print(F("|  [MPU6050] Roll       : ")); Serial.print(tele.roll, 1);        Serial.println(F(" deg"));
  Serial.print(F("|  [MPU6050] Vibration  : ")); Serial.println(tele.vibration);
  Serial.println(F("+------------------------------------------+\n"));
}

// ============================================================================
//  HTTP HANDLERS (UNCHANGED LOGIC, CORS PRESERVED)
// ============================================================================

void sendCORSHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.sendHeader("Access-Control-Max-Age", "86400");
}

void handleOptions() {
  sendCORSHeaders();
  server.send(204);
}

void handleRoot() {
  sendCORSHeaders();
  server.send(200, "text/plain", "smart baloon Telemetry API Active. GET /data for JSON.");
}

void handleData() {
  sendCORSHeaders();

  StaticJsonDocument<256> doc;

  doc["altitude"]    = roundf(tele.altitude    * 10.0f) / 10.0f;
  doc["temperature"] = roundf(tele.temperature * 10.0f) / 10.0f;
  doc["pressure"]    = roundf(tele.pressure    * 10.0f) / 10.0f;
  doc["aqi"]         = tele.aqi;
  doc["co"]          = tele.co;
  doc["smoke"]       = tele.smoke;
  doc["latitude"]    = tele.latitude;
  doc["longitude"]   = tele.longitude;
  doc["speed"]       = roundf(tele.speed * 10.0f) / 10.0f;
  doc["pitch"]       = roundf(tele.pitch * 10.0f) / 10.0f;
  doc["roll"]        = roundf(tele.roll  * 10.0f) / 10.0f;
  doc["vibration"]   = tele.vibration;

  String json;
  serializeJson(doc, json);

  server.send(200, "application/json", json);
}

void handleNotFound() {
  sendCORSHeaders();
  server.send(404, "application/json", "{\"error\":\"Not Found\"}");
}

// ============================================================================
//  FILTERING HELPERS (UNCHANGED)
// ============================================================================

int multiSampleADC(uint8_t pin, uint8_t samples) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return (int)(sum / samples);
}

int medianFilter(uint8_t pin, uint8_t samples) {
  int buf[MEDIAN_SAMPLES];
  uint8_t n = min(samples, (uint8_t)MEDIAN_SAMPLES);

  for (uint8_t i = 0; i < n; i++) {
    buf[i] = multiSampleADC(pin, 8);
  }

  for (uint8_t i = 1; i < n; i++) {
    int    key = buf[i];
    int8_t j   = i - 1;
    while (j >= 0 && buf[j] > key) {
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = key;
  }

  return buf[n / 2];
}

float applyEMA(float newVal, float prevEMA, float alpha) {
  return (alpha * newVal) + ((1.0f - alpha) * prevEMA);
}

bool spikeReject(float newVal, float prevVal, float threshold, bool isFirst) {
  if (isFirst) return true;
  return (fabsf(newVal - prevVal) <= threshold);
}
