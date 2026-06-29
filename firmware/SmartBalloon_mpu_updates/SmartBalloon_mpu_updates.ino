/*
 * ============================================================================
 *  ESP32 Smart Weather Balloon Telemetry System — Nimbus-1
 *  Noise-filtered | Direct Wire.h MPU6050 | Startup calibration
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
 *  NOTE: MPU6050 library removed. Sensor is now driven directly via Wire.h
 *        register reads — identical to the working I2C scanner / test sketch.
 *
 *  WIRING SUMMARY
 *  ──────────────
 *  BMP180  VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
 *  MPU6050 VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22  AD0→GND (addr 0x68)
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
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// ============================================================================
//  WI-FI ACCESS POINT CREDENTIALS
// ============================================================================
const char*     AP_SSID     = "smart air_baloon";
const char*     AP_PASSWORD = "12345678";
const IPAddress AP_IP      (192, 168, 4, 1);
const IPAddress AP_GW      (192, 168, 4, 1);
const IPAddress AP_SUBNET  (255, 255, 255, 0);

// ============================================================================
//  PIN DEFINITIONS
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
//  MPU6050 DIRECT REGISTER DEFINITIONS
//  Source: MPU6050 datasheet, register map rev 4.2
// ============================================================================
#define MPU_ADDR          0x68   // AD0 pin tied to GND → address 0x68
                                 // If AD0 is HIGH change to 0x69

#define MPU_REG_PWR_MGMT  0x6B   // Power management register
#define MPU_REG_ACCEL_CFG 0x1C   // Accelerometer config register
#define MPU_REG_GYRO_CFG  0x1B   // Gyroscope config register
#define MPU_REG_ACCEL_OUT 0x3B   // First accelerometer data register (6 bytes)
#define MPU_REG_WHO_AM_I  0x75   // WHO_AM_I register — returns 0x68 when alive

// Accelerometer full-scale: ±2g → 16384 LSB/g
#define MPU_LSB_PER_G     16384.0f

// ============================================================================
//  CALIBRATION & SCALING CONSTANTS
// ============================================================================
#define MQ_VD_SCALE           2       // 10k/10k voltage divider compensation
#define LM35_OFFSET           0.0f   // Trim offset in °C

#define MQ135_AQI_MAX         500
#define MQ7_CO_PPM_MAX        1000
#define MQ2_SMOKE_PPM_MAX     1000

#define VIBRATION_SCALE       100.0f

// Number of samples averaged during startup calibration
// Balloon must be stationary and flat during power-on
#define MPU_CALIB_SAMPLES     200

#define SENSOR_INTERVAL_MS    1000
#define SERIAL_PRINT_INTERVAL_MS  3000

// ============================================================================
//  FILTERING CONSTANTS
// ============================================================================
#define MEDIAN_SAMPLES        9     // Must be odd

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
//  HTML DASHBOARD PLACEHOLDER
// ============================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!-- PASTE MY EXISTING HTML HERE -->
)rawliteral";

// ============================================================================
//  OBJECT INSTANCES
// ============================================================================
Adafruit_BMP085  bmp180;
TinyGPSPlus      gps;
HardwareSerial   gpsSerial(2);
WebServer        server(80);

// ============================================================================
//  SENSOR STATUS FLAGS
// ============================================================================
bool bmp180_ok  = false;
bool mpu6050_ok = false;

// ============================================================================
//  MPU6050 CALIBRATION OFFSETS
//  Filled during setupMPU6050() — used to zero pitch/roll at startup position
// ============================================================================
float mpu_pitch_offset = 0.0f;
float mpu_roll_offset  = 0.0f;

// ============================================================================
//  EMA STATE VARIABLES
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

// First-sample flags (skip spike rejection on first reading)
bool first_temp  = true;
bool first_alt   = true;
bool first_pres  = true;
bool first_aqi   = true;
bool first_co    = true;
bool first_smoke = true;
bool first_pitch = true;
bool first_roll  = true;

// ============================================================================
//  LIVE TELEMETRY STRUCT
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
//  TIMING
// ============================================================================
unsigned long lastSensorRead  = 0;
unsigned long lastSerialPrint = 0;

// ============================================================================
//  FUNCTION PROTOTYPES
// ============================================================================
void   setupWiFiAP();
void   setupSensors();
void   setupMPU6050();
void   readSensors();
void   readMPU6050Raw(float &axg, float &ayg, float &azg);
void   printTelemetry();
void   handleRoot();
void   handleData();
void   handleNotFound();

int    multiSampleADC(uint8_t pin, uint8_t samples);
int    medianFilter(uint8_t pin, uint8_t samples);
float  applyEMA(float newVal, float prevEMA, float alpha);
bool   spikeReject(float newVal, float prevVal, float threshold, bool isFirst);

// ============================================================================
//  MPU6050 WIRE HELPERS
// ============================================================================

// Write one byte to an MPU6050 register
void mpuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

// Read one byte from an MPU6050 register
uint8_t mpuReadByte(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
  return Wire.available() ? Wire.read() : 0;
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n============================================="));
  Serial.println(F("  Nimbus-1  Smart Weather Balloon System    "));
  Serial.println(F("=============================================\n"));

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_ESP_RX, GPS_ESP_TX);

  memset(&tele, 0, sizeof(tele));

  setupSensors();
  setupWiFiAP();

  server.on("/",     HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println(F("[SERVER] HTTP server started."));
  Serial.println(F("[SERVER] Dashboard → http://192.168.4.1\n"));
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  // Feed GPS parser non-blocking
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Read all sensors every SENSOR_INTERVAL_MS
  if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = millis();
    readSensors();
  }

  // Print telemetry snapshot to Serial every 3 seconds
  if (millis() - lastSerialPrint >= SERIAL_PRINT_INTERVAL_MS) {
    lastSerialPrint = millis();
    printTelemetry();
  }

  server.handleClient();
}

// ============================================================================
//  setupWiFiAP()
// ============================================================================
void setupWiFiAP() {
  Serial.println(F("[WiFi]  Starting Access Point..."));
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_SUBNET);

  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.print(F("[WiFi]  SSID     : ")); Serial.println(AP_SSID);
    Serial.print(F("[WiFi]  Password : ")); Serial.println(AP_PASSWORD);
    Serial.print(F("[WiFi]  IP Addr  : ")); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(F("[WiFi]  ERROR — Access Point failed to start!"));
  }
}

// ============================================================================
//  setupSensors()
// ============================================================================
void setupSensors() {
  Serial.println(F("[SENSORS] Initialising...\n"));

  // BMP180
  if (bmp180.begin()) {
    bmp180_ok = true;
    Serial.println(F("[BMP180]  OK  — Pressure & Altitude ready."));
  } else {
    Serial.println(F("[BMP180]  FAIL — Not detected on I2C. altitude/pressure = 0."));
  }

  // MPU6050 — direct Wire.h initialisation + calibration
  setupMPU6050();

  // GPS
  Serial.println(F("[NEO-6M]  Listening on UART2. lat/lon/speed = 0 until fix."));

  // Analog
  Serial.println(F("[LM35]    GPIO33 — ready."));
  Serial.println(F("[MQ135]   GPIO34 — ready. (verify voltage divider!)"));
  Serial.println(F("[MQ2]     GPIO35 — ready. (verify voltage divider!)"));
  Serial.println(F("[MQ7]     GPIO32 — ready. (verify voltage divider!)"));

  Serial.println(F("\n[SENSORS] Done.\n"));
}

// ============================================================================
//  setupMPU6050()
//  Wakes the sensor, configures full-scale ranges, then collects
//  MPU_CALIB_SAMPLES readings to compute pitch/roll offsets so the
//  dashboard starts at 0° regardless of how the balloon is mounted.
// ============================================================================
void setupMPU6050() {
  Serial.println(F("[MPU6050] Checking WHO_AM_I register..."));

  uint8_t who = mpuReadByte(MPU_REG_WHO_AM_I);
  Serial.print(F("[MPU6050] WHO_AM_I = 0x"));
  Serial.println(who, HEX);   // expect 0x68

  if (who != 0x68 && who != 0x72) {
    Serial.println(F("[MPU6050] FAIL — unexpected WHO_AM_I. Check wiring/address."));
    Serial.println(F("[MPU6050] pitch/roll/vibration will remain 0."));
    mpu6050_ok = false;
    return;
  }

  // Wake from sleep (PWR_MGMT_1 = 0x00 clears the SLEEP bit)
  mpuWriteReg(MPU_REG_PWR_MGMT, 0x00);
  delay(100);   // let oscillator stabilise

  // Accelerometer full-scale ±2g  (bits [4:3] = 00)
  mpuWriteReg(MPU_REG_ACCEL_CFG, 0x00);

  // Gyroscope full-scale ±250°/s  (bits [4:3] = 00)
  mpuWriteReg(MPU_REG_GYRO_CFG, 0x00);

  mpu6050_ok = true;
  Serial.println(F("[MPU6050] OK  — Woken, ±2g accel, ±250°/s gyro configured."));

  // ── Startup calibration ──────────────────────────────────────────────────
  // Keep balloon stationary during boot. We average MPU_CALIB_SAMPLES
  // readings and store the resulting pitch/roll as offsets.
  Serial.print(F("[MPU6050] Calibrating (keep sensor still)... "));

  float sumPitch = 0.0f, sumRoll = 0.0f;

  for (int i = 0; i < MPU_CALIB_SAMPLES; i++) {
    float axg, ayg, azg;
    readMPU6050Raw(axg, ayg, azg);

    sumPitch += atan2f(ayg, sqrtf(axg * axg + azg * azg)) * (180.0f / M_PI);
    sumRoll  += atan2f(axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / M_PI);

    delay(5);   // ~5 ms per sample → ~1 second total for 200 samples
  }

  mpu_pitch_offset = sumPitch / MPU_CALIB_SAMPLES;
  mpu_roll_offset  = sumRoll  / MPU_CALIB_SAMPLES;

  Serial.println(F("done."));
  Serial.print(F("[MPU6050] Pitch offset: ")); Serial.print(mpu_pitch_offset, 2); Serial.println(F(" °"));
  Serial.print(F("[MPU6050] Roll  offset: ")); Serial.print(mpu_roll_offset,  2); Serial.println(F(" °"));
}

// ============================================================================
//  readMPU6050Raw()
//  Reads 6 accelerometer bytes directly from registers 0x3B–0x40.
//  Combines high + low bytes into signed int16, then converts to g.
// ============================================================================
void readMPU6050Raw(float &axg, float &ayg, float &azg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_OUT);    // start at register 0x3B
  Wire.endTransmission(false);       // repeated start
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);

  if (Wire.available() < 6) {
    axg = 0.0f; ayg = 0.0f; azg = 0.0f;
    return;
  }

  int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t az = (int16_t)((Wire.read() << 8) | Wire.read());

  axg = ax / MPU_LSB_PER_G;
  ayg = ay / MPU_LSB_PER_G;
  azg = az / MPU_LSB_PER_G;
}

// ============================================================================
//  readSensors()
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

  // ── NEO-6M GPS : Latitude, Longitude, Speed ──────────────────────────────
  if (gps.location.isValid() && gps.location.isUpdated()) {
    tele.latitude  = gps.location.lat();
    tele.longitude = gps.location.lng();
  }
  if (gps.speed.isValid() && gps.speed.isUpdated()) {
    float rawSpeed = (float)gps.speed.kmph();
    ema_speed      = applyEMA(rawSpeed, ema_speed, EMA_ALPHA_SPEED);
    tele.speed     = ema_speed;
  }

  // ── MPU6050 : Pitch, Roll, Vibration — direct Wire.h reads ───────────────
  if (mpu6050_ok) {

    // Step 1: burst-average 8 raw readings to suppress quantisation noise
    const uint8_t IMU_SAMPLES = 8;
    float sumAxg = 0, sumAyg = 0, sumAzg = 0;

    for (uint8_t i = 0; i < IMU_SAMPLES; i++) {
      float axg, ayg, azg;
      readMPU6050Raw(axg, ayg, azg);
      sumAxg += axg;
      sumAyg += ayg;
      sumAzg += azg;
      delayMicroseconds(500);
    }

    float axg = sumAxg / IMU_SAMPLES;
    float ayg = sumAyg / IMU_SAMPLES;
    float azg = sumAzg / IMU_SAMPLES;

    // ── DEBUG: print raw axis values every sensor cycle ──────────────────
    Serial.print(F("[MPU6050-RAW] Ax="));  Serial.print(axg, 4);
    Serial.print(F("g  Ay="));             Serial.print(ayg, 4);
    Serial.print(F("g  Az="));             Serial.print(azg, 4);
    Serial.println(F("g"));
    // ── END DEBUG ─────────────────────────────────────────────────────────

    // Step 2: compute pitch and roll from averaged axes
    float rawPitch = atan2f(ayg, sqrtf(axg * axg + azg * azg)) * (180.0f / M_PI);
    float rawRoll  = atan2f(axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / M_PI);

    // ── DEBUG: print raw angles before offset and EMA ─────────────────────
    Serial.print(F("[MPU6050-ANGLE] rawPitch="));  Serial.print(rawPitch, 2);
    Serial.print(F("°  rawRoll="));                Serial.print(rawRoll, 2);
    Serial.println(F("°"));
    // ── END DEBUG ─────────────────────────────────────────────────────────

    // Step 3: subtract startup calibration offsets
    float calPitch = rawPitch - mpu_pitch_offset;
    float calRoll  = rawRoll  - mpu_roll_offset;

    // Step 4: spike rejection
    if (spikeReject(calPitch, ema_pitch, SPIKE_THRESH_PITCH, first_pitch)) {
      first_pitch = false;
      ema_pitch   = applyEMA(calPitch, ema_pitch, EMA_ALPHA_PITCH);
    }
    if (spikeReject(calRoll, ema_roll, SPIKE_THRESH_ROLL, first_roll)) {
      first_roll = false;
      ema_roll   = applyEMA(calRoll, ema_roll, EMA_ALPHA_ROLL);
    }

    // Step 5: vibration — total acceleration magnitude deviation from 1g
    float mag     = sqrtf(axg * axg + ayg * ayg + azg * azg);
    float rawVib  = fabsf(mag - 1.0f) * VIBRATION_SCALE;
    ema_vibration = applyEMA(rawVib, ema_vibration, EMA_ALPHA_VIBRATION);

    tele.pitch     = ema_pitch;
    tele.roll      = ema_roll;
    tele.vibration = (int)constrain(ema_vibration, 0.0f, 999.0f);

    // ── DEBUG: print final filtered values ────────────────────────────────
    Serial.print(F("[MPU6050-OUT]  pitch="));  Serial.print(tele.pitch, 2);
    Serial.print(F("°  roll="));               Serial.print(tele.roll, 2);
    Serial.print(F("°  vib="));               Serial.println(tele.vibration);
    // ── END DEBUG ─────────────────────────────────────────────────────────
  }
}

// ============================================================================
//  printTelemetry()  —  Full snapshot to Serial every 3 seconds
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
//  handleRoot()
// ============================================================================
void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

// ============================================================================
//  handleData()
// ============================================================================
void handleData() {
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

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

// ============================================================================
//  handleNotFound()
// ============================================================================
void handleNotFound() {
  server.send(404, "text/plain", "404 Not Found");
}

// ============================================================================
//  FILTERING HELPER FUNCTIONS
// ============================================================================

// Reads ADC pin `samples` times and returns integer average
int multiSampleADC(uint8_t pin, uint8_t samples) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return (int)(sum / samples);
}

// Takes `samples` multi-averaged ADC readings, returns the median value
int medianFilter(uint8_t pin, uint8_t samples) {
  int buf[MEDIAN_SAMPLES];
  uint8_t n = min(samples, (uint8_t)MEDIAN_SAMPLES);

  for (uint8_t i = 0; i < n; i++) {
    buf[i] = multiSampleADC(pin, 8);
  }

  // Insertion sort
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

// Exponential Moving Average
float applyEMA(float newVal, float prevEMA, float alpha) {
  return (alpha * newVal) + ((1.0f - alpha) * prevEMA);
}

// Returns true if value is valid (within threshold), false if spike
bool spikeReject(float newVal, float prevVal, float threshold, bool isFirst) {
  if (isFirst) return true;
  return (fabsf(newVal - prevVal) <= threshold);
}
