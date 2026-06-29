  /*
  * ============================================================================
  *  ESP32 Smart Weather Balloon Telemetry System — Nimbus-1
  *  With full per-sensor noise filtering
  * ============================================================================
  *
  *  REQUIRED LIBRARIES  (Arduino IDE → Tools → Manage Libraries)
  *  ─────────────────────────────────────────────────────────────
  *  1. WiFi.h                  Built-in with ESP32 Arduino Core
  *  2. WebServer.h             Built-in with ESP32 Arduino Core
  *  3. Wire.h                  Built-in I2C library
  *  4. Adafruit BMP085 Library Search: "Adafruit BMP085" by Adafruit
  *  5. Adafruit Unified Sensor Search: "Adafruit Unified Sensor" by Adafruit
  *  6. MPU6050                 Search: "MPU6050" by Electronic Cats
  *  7. TinyGPSPlus             Search: "TinyGPS++" by Mikal Hart
  *  8. ArduinoJson             Search: "ArduinoJson" by Benoit Blanchon (v6.x)
  *
  *  WIRING SUMMARY
  *  ──────────────
  *  BMP180  VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
  *  MPU6050 VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
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
  #include <MPU6050.h>
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
  #define PIN_LM35       33
  #define PIN_MQ135      34
  #define PIN_MQ2        35
  #define PIN_MQ7        32
  #define PIN_I2C_SDA    21
  #define PIN_I2C_SCL    22
  #define GPS_ESP_RX     16
  #define GPS_ESP_TX     17
  #define GPS_BAUD     9600

  // ============================================================================
  //  CALIBRATION & SCALING CONSTANTS
  // ============================================================================
  #define MQ_VD_SCALE           2       // 10k/10k voltage divider compensation
  #define LM35_OFFSET           0.0f   // Trim offset in °C

  #define MQ135_AQI_MAX         500
  #define MQ7_CO_PPM_MAX        1000
  #define MQ2_SMOKE_PPM_MAX     1000

  #define MPU_LSB_PER_G         16384.0f
  #define VIBRATION_SCALE       100.0f

  #define SENSOR_INTERVAL_MS    1000

  // ============================================================================
  //  FILTERING CONSTANTS
  // ============================================================================

  // --- Median filter window size (must be odd) ---
  // Used for: LM35, MQ135, MQ2, MQ7
  // Takes N samples, sorts, picks the middle value. Kills impulse spikes.
  #define MEDIAN_SAMPLES        9

  // --- EMA (Exponential Moving Average) alpha values ---
  // Range: 0.0 – 1.0. Lower = smoother but slower to respond.
  // Higher = faster response but less smoothing.
  #define EMA_ALPHA_TEMP        0.15f   // LM35   — slow thermal changes
  #define EMA_ALPHA_AQI         0.10f   // MQ135  — very noisy, heavy smoothing
  #define EMA_ALPHA_CO          0.10f   // MQ7    — very noisy, heavy smoothing
  #define EMA_ALPHA_SMOKE       0.10f   // MQ2    — very noisy, heavy smoothing
  #define EMA_ALPHA_ALTITUDE    0.20f   // BMP180 altitude
  #define EMA_ALPHA_PRESSURE    0.20f   // BMP180 pressure
  #define EMA_ALPHA_PITCH       0.25f   // MPU6050 pitch
  #define EMA_ALPHA_ROLL        0.25f   // MPU6050 roll
  #define EMA_ALPHA_VIBRATION   0.30f   // MPU6050 vibration — allow faster response
  #define EMA_ALPHA_SPEED       0.20f   // GPS speed

  // --- Spike rejection thresholds ---
  // If a new reading differs from last by more than this, it is discarded.
  #define SPIKE_THRESH_TEMP     10.0f   // °C    — reject if jump > 10°C/sec
  #define SPIKE_THRESH_ALTITUDE 50.0f   // m     — reject if jump > 50m/sec
  #define SPIKE_THRESH_PRESSURE 10.0f   // hPa   — reject if jump > 10hPa/sec
  #define SPIKE_THRESH_AQI      150     // AQI units
  #define SPIKE_THRESH_CO       200     // ppm
  #define SPIKE_THRESH_SMOKE    200     // ppm
  #define SPIKE_THRESH_PITCH    30.0f   // degrees/sec
  #define SPIKE_THRESH_ROLL     30.0f   // degrees/sec

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
  MPU6050          mpu6050;
  TinyGPSPlus      gps;
  HardwareSerial   gpsSerial(2);
  WebServer        server(80);

  // ============================================================================
  //  SENSOR STATUS FLAGS
  // ============================================================================
  bool bmp180_ok  = false;
  bool mpu6050_ok = false;

  // ============================================================================
  //  EMA STATE VARIABLES  (hold the running filtered value for each channel)
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

  // Flags to detect first sample (skip spike check on first reading)
  bool first_temp     = true;
  bool first_alt      = true;
  bool first_pres     = true;
  bool first_aqi      = true;
  bool first_co       = true;
  bool first_smoke    = true;
  bool first_pitch    = true;
  bool first_roll     = true;

  // ============================================================================
  //  LIVE TELEMETRY STRUCT  (filled by readSensors(), served by /data)
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
  #define SERIAL_PRINT_INTERVAL_MS  3000

  // ============================================================================
  //  FUNCTION PROTOTYPES
  // ============================================================================
  void   setupWiFiAP();
  void   setupSensors();
  void   readSensors();
  void   printTelemetry();
  void   handleRoot();
  void   handleData();
  void   handleNotFound();

  // Filtering helpers
  int    medianFilter(uint8_t pin, uint8_t samples);
  float  applyEMA(float newVal, float prevEMA, float alpha);
  bool   spikeReject(float newVal, float prevVal, float threshold, bool isFirst);
  int    multiSampleADC(uint8_t pin, uint8_t samples);

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

    // Read sensors on schedule
    if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS) {
      lastSensorRead = millis();
      readSensors();
    }

    // Print telemetry to Serial every 3 seconds
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

    if (bmp180.begin()) {
      bmp180_ok = true;
      Serial.println(F("[BMP180]  OK  — Pressure & Altitude ready."));
    } else {
      Serial.println(F("[BMP180]  FAIL — Not detected on I2C. altitude/pressure = 0."));
    }

    mpu6050.initialize();
    if (mpu6050.testConnection()) {
      mpu6050_ok = true;
      Serial.println(F("[MPU6050] OK  — Motion & Orientation ready."));
    } else {
      Serial.println(F("[MPU6050] FAIL — Not detected on I2C. pitch/roll/vibration = 0."));
    }

    Serial.println(F("[NEO-6M]  Listening on UART2. lat/lon/speed = 0 until fix."));
    Serial.println(F("[LM35]    GPIO33 — ready."));
    Serial.println(F("[MQ135]   GPIO34 — ready. (verify voltage divider!)"));
    Serial.println(F("[MQ2]     GPIO35 — ready. (verify voltage divider!)"));
    Serial.println(F("[MQ7]     GPIO32 — ready. (verify voltage divider!)"));
    Serial.println(F("\n[SENSORS] Done.\n"));
  }

  // ============================================================================
  //  readSensors()
  //
  //  Filtering pipeline per sensor:
  //
  //  Analog (LM35, MQ135, MQ2, MQ7):
  //    Step 1 — Multi-sample ADC average    (removes random ADC quantisation noise)
  //    Step 2 — Median filter               (removes impulse / spike samples)
  //    Step 3 — Spike rejection             (discards physically impossible jumps)
  //    Step 4 — EMA low-pass filter         (smooths remaining jitter)
  //
  //  BMP180 (I2C):
  //    Step 1 — Spike rejection
  //    Step 2 — EMA low-pass filter
  //
  //  MPU6050 (I2C):
  //    Step 1 — Multi-sample average of raw axes
  //    Step 2 — Spike rejection on pitch/roll
  //    Step 3 — EMA low-pass filter on pitch, roll, vibration
  //
  //  GPS:
  //    Step 1 — TinyGPS++ internal NMEA validation (only isValid() data used)
  //    Step 2 — EMA on speed
  // ============================================================================
  void readSensors() {

    // ── LM35 : Temperature ───────────────────────────────────────────────────
    {
      // Step 1+2: median of MEDIAN_SAMPLES multi-averaged readings
      int   medRaw = medianFilter(PIN_LM35, MEDIAN_SAMPLES);
      float mV     = (medRaw / 4095.0f) * 3300.0f;
      float rawTemp = (mV / 10.0f) + LM35_OFFSET;

      // Step 3: spike rejection
      if (spikeReject(rawTemp, ema_temp, SPIKE_THRESH_TEMP, first_temp)) {
        first_temp = false;
        // Step 4: EMA
        ema_temp = applyEMA(rawTemp, ema_temp, EMA_ALPHA_TEMP);
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

        // Spike rejection
        if (spikeReject(rawPres, ema_pressure, SPIKE_THRESH_PRESSURE, first_pres)) {
          first_pres = false;
          ema_pressure = applyEMA(rawPres, ema_pressure, EMA_ALPHA_PRESSURE);
        }
        if (spikeReject(rawAlt, ema_altitude, SPIKE_THRESH_ALTITUDE, first_alt)) {
          first_alt = false;
          ema_altitude = applyEMA(rawAlt, ema_altitude, EMA_ALPHA_ALTITUDE);
        }
      }
      tele.pressure = ema_pressure;
      tele.altitude = ema_altitude;
    }

    // ── NEO-6M GPS : Latitude, Longitude, Speed ──────────────────────────────
    // TinyGPS++ validates NMEA checksum and fix quality internally.
    // isValid() is the primary filter — only trust confirmed fixes.
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
      // Step 1: average 8 raw samples per axis to reduce quantisation noise
      const uint8_t IMU_SAMPLES = 8;
      long sumAx = 0, sumAy = 0, sumAz = 0;
      for (uint8_t i = 0; i < IMU_SAMPLES; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumAx += ax; sumAy += ay; sumAz += az;
        delayMicroseconds(500);   // short gap between burst reads
      }
      float axg = (sumAx / (float)IMU_SAMPLES) / MPU_LSB_PER_G;
      float ayg = (sumAy / (float)IMU_SAMPLES) / MPU_LSB_PER_G;
      float azg = (sumAz / (float)IMU_SAMPLES) / MPU_LSB_PER_G;

      // Step 2: compute angles
      float rawPitch = atan2f(ayg, sqrtf(axg * axg + azg * azg)) * (180.0f / M_PI);
      float rawRoll  = atan2f(axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / M_PI);

      // Step 3: spike rejection on pitch and roll
      if (spikeReject(rawPitch, ema_pitch, SPIKE_THRESH_PITCH, first_pitch)) {
        first_pitch = false;
        ema_pitch   = applyEMA(rawPitch, ema_pitch, EMA_ALPHA_PITCH);
      }
      if (spikeReject(rawRoll, ema_roll, SPIKE_THRESH_ROLL, first_roll)) {
        first_roll = false;
        ema_roll   = applyEMA(rawRoll, ema_roll, EMA_ALPHA_ROLL);
      }

      // Step 4: vibration from total acceleration deviation from 1g
      float mag    = sqrtf(axg * axg + ayg * ayg + azg * azg);
      float rawVib = fabsf(mag - 1.0f) * VIBRATION_SCALE;
      ema_vibration = applyEMA(rawVib, ema_vibration, EMA_ALPHA_VIBRATION);

      tele.pitch     = ema_pitch;
      tele.roll      = ema_roll;
      tele.vibration = (int)constrain(ema_vibration, 0.0f, 999.0f);
    }
  }

  // ============================================================================
  //  printTelemetry()  —  Pretty-print all sensor values to Serial Monitor
  // ============================================================================
  void printTelemetry() {
    Serial.println(F("\n┌─────────────────────────────────────────┐"));
    Serial.println(F("│       NIMBUS-1  TELEMETRY SNAPSHOT      │"));
    Serial.println(F("├─────────────────────────────────────────┤"));

    // BMP180
    Serial.print(F("│  [BMP180]  Altitude   : "));
    Serial.print(tele.altitude, 1);
    Serial.println(F(" m"));
    Serial.print(F("│  [BMP180]  Pressure   : "));
    Serial.print(tele.pressure, 1);
    Serial.println(F(" hPa"));

    // LM35
    Serial.print(F("│  [LM35]    Temperature: "));
    Serial.print(tele.temperature, 1);
    Serial.println(F(" °C"));

    // MQ sensors
    Serial.print(F("│  [MQ135]   AQI        : "));
    Serial.println(tele.aqi);
    Serial.print(F("│  [MQ7]     CO         : "));
    Serial.print(tele.co);
    Serial.println(F(" ppm"));
    Serial.print(F("│  [MQ2]     Smoke/LPG  : "));
    Serial.print(tele.smoke);
    Serial.println(F(" ppm"));

    // GPS
    Serial.print(F("│  [NEO-6M]  Latitude   : "));
    Serial.println(tele.latitude, 6);
    Serial.print(F("│  [NEO-6M]  Longitude  : "));
    Serial.println(tele.longitude, 6);
    Serial.print(F("│  [NEO-6M]  Speed      : "));
    Serial.print(tele.speed, 1);
    Serial.println(F(" km/h"));
    Serial.print(F("│  [NEO-6M]  GPS Fix    : "));
    Serial.println(gps.location.isValid() ? F("YES") : F("NO — waiting..."));

    // MPU6050
    Serial.print(F("│  [MPU6050] Pitch      : "));
    Serial.print(tele.pitch, 1);
    Serial.println(F(" °"));
    Serial.print(F("│  [MPU6050] Roll       : "));
    Serial.print(tele.roll, 1);
    Serial.println(F(" °"));
    Serial.print(F("│  [MPU6050] Vibration  : "));
    Serial.println(tele.vibration);

    Serial.println(F("└─────────────────────────────────────────┘"));
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

  // ----------------------------------------------------------------------------
  //  multiSampleADC()
  //  Reads the same ADC pin `samples` times and returns the integer average.
  //  Reduces random quantisation noise from the ESP32 SAR ADC.
  // ----------------------------------------------------------------------------
  int multiSampleADC(uint8_t pin, uint8_t samples) {
    long sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
      sum += analogRead(pin);
      delayMicroseconds(200);   // let ADC settle between reads
    }
    return (int)(sum / samples);
  }

  // ----------------------------------------------------------------------------
  //  medianFilter()
  //  Takes `samples` multi-averaged ADC readings, sorts them, returns the median.
  //  `samples` should be odd (3, 5, 7, 9 ...).
  //  Best defence against impulse noise and single-sample spikes.
  // ----------------------------------------------------------------------------
  int medianFilter(uint8_t pin, uint8_t samples) {
    int buf[MEDIAN_SAMPLES];
    uint8_t n = min(samples, (uint8_t)MEDIAN_SAMPLES);

    for (uint8_t i = 0; i < n; i++) {
      buf[i] = multiSampleADC(pin, 8);   // 8 reads averaged per median slot
    }

    // Insertion sort (tiny array — no overhead)
    for (uint8_t i = 1; i < n; i++) {
      int key = buf[i];
      int8_t j = i - 1;
      while (j >= 0 && buf[j] > key) {
        buf[j + 1] = buf[j];
        j--;
      }
      buf[j + 1] = key;
    }

    return buf[n / 2];   // middle element
  }

  // ----------------------------------------------------------------------------
  //  applyEMA()
  //  Exponential Moving Average:
  //    output = alpha * newValue + (1 - alpha) * previousEMA
  //  Lower alpha = more smoothing, slower response.
  // ----------------------------------------------------------------------------
  float applyEMA(float newVal, float prevEMA, float alpha) {
    return (alpha * newVal) + ((1.0f - alpha) * prevEMA);
  }

  // ----------------------------------------------------------------------------
  //  spikeReject()
  //  Returns true  → value is valid, proceed with filtering.
  //  Returns false → value is a spike, discard and keep previous EMA.
  //  `isFirst` bypasses the check on the very first sample to seed the EMA.
  // ----------------------------------------------------------------------------
  bool spikeReject(float newVal, float prevVal, float threshold, bool isFirst) {
    if (isFirst) return true;
    return (fabsf(newVal - prevVal) <= threshold);
  }
