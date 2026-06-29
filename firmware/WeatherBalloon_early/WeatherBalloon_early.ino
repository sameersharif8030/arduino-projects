#include <WiFi.h>
#include <AsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_BMP180.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// ============================================================================
// WIFI CONFIGURATION
// ============================================================================
const char* ssid = "WeatherBalloon";
const char* password = "12345678";
const IPAddress local_ip(192, 168, 4, 1);
const IPAddress gateway(192, 168, 4, 1);
const IPAddress subnet(255, 255, 255, 0);

// ============================================================================
// WEB SERVER
// ============================================================================
AsyncWebServer server(80);

// ============================================================================
// SENSOR OBJECTS
// ============================================================================
Adafruit_BMP180 bmp180;
MPU6050 mpu6050;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define BMP180_SDA 21
#define BMP180_SCL 22
#define MPU6050_SDA 21
#define MPU6050_SCL 22
#define LM35_PIN 34
#define MQ135_PIN 35
#define MQ2_PIN 32
#define MQ7_PIN 33
#define GPS_TX 16
#define GPS_RX 17

// ============================================================================
// SENSOR DATA STRUCTURE
// ============================================================================
struct SensorData {
  float altitude;
  float temperature;
  float pressure;
  int aqi;
  int co;
  int smoke;
  double latitude;
  double longitude;
  double speed;
  float pitch;
  float roll;
  int vibration;
} sensorData;

// ============================================================================
// TIMING VARIABLES
// ============================================================================
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 1000; // 1 second

// ============================================================================
// HTML DASHBOARD PLACEHOLDER
// ============================================================================
const char index_html[] PROGMEM = R"rawliteral(
PASTE_HTML_HERE
)rawliteral";

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initWiFi();
void initWebServer();
void initSensors();
void readBMP180();
void readLM35();
void readMQ135();
void readMQ2();
void readMQ7();
void readGPS();
void readMPU6050();
void updateSensorData();
String getSensorDataJSON();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=== ESP32 Smart Weather Balloon System ===");
  
  // Initialize I2C for BMP180 and MPU6050
  Wire.begin(BMP180_SDA, BMP180_SCL);
  
  // Initialize GPS Serial
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Initialize sensors
  initSensors();
  
  // Initialize WiFi AP
  initWiFi();
  
  // Initialize Web Server
  initWebServer();
  
  Serial.println("System initialized successfully!");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  
  // Update sensors every 1 second
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    updateSensorData();
    lastUpdateTime = currentTime;
    
    Serial.println("Sensor data updated");
  }
  
  // Feed GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  delay(10);
}

// ============================================================================
// WIFI INITIALIZATION
// ============================================================================
void initWiFi() {
  Serial.println("\n[WiFi] Starting WiFi Access Point...");
  
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  
  Serial.print("[WiFi] SSID: ");
  Serial.println(ssid);
  Serial.print("[WiFi] Password: ");
  Serial.println(password);
  Serial.print("[WiFi] IP Address: ");
  Serial.println(WiFi.softAPIP());
}

// ============================================================================
// WEB SERVER INITIALIZATION
// ============================================================================
void initWebServer() {
  Serial.println("\n[Server] Initializing AsyncWebServer...");
  
  // Route: root web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });
  
  // Route: sensor data JSON
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = getSensorDataJSON();
    request->send(200, "application/json", json);
  });
  
  server.begin();
  Serial.println("[Server] AsyncWebServer started on http://192.168.4.1");
}

// ============================================================================
// SENSOR INITIALIZATION
// ============================================================================
void initSensors() {
  Serial.println("\n[Sensors] Initializing sensors...");
  
  // Initialize BMP180
  if (!bmp180.begin()) {
    Serial.println("[BMP180] Initialization failed!");
  } else {
    Serial.println("[BMP180] Initialized successfully");
  }
  
  // Initialize MPU6050
  mpu6050.initialize();
  if (!mpu6050.testConnection()) {
    Serial.println("[MPU6050] Connection failed!");
  } else {
    Serial.println("[MPU6050] Initialized successfully");
  }
  
  // Initialize ADC pins
  pinMode(LM35_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);
  pinMode(MQ2_PIN, INPUT);
  pinMode(MQ7_PIN, INPUT);
  
  Serial.println("[Sensors] All sensors initialized");
}

// ============================================================================
// BMP180: ALTITUDE & PRESSURE
// ============================================================================
void readBMP180() {
  float pressurePa = bmp180.readPressure();
  float temperatureC = bmp180.readTemperature();
  
  if (pressurePa > 0) {
    sensorData.pressure = pressurePa / 100.0; // Convert to hPa
    
    // Calculate altitude using barometric formula
    sensorData.altitude = 44330 * (1.0 - pow(sensorData.pressure / 1013.25, 1.0 / 5.255));
  }
}

// ============================================================================
// LM35: TEMPERATURE
// ============================================================================
void readLM35() {
  int rawValue = analogRead(LM35_PIN);
  float voltage = (rawValue / 4095.0) * 3.3;
  sensorData.temperature = voltage * 100.0; // LM35: 10mV per °C
}

// ============================================================================
// MQ135: AIR QUALITY INDEX
// ============================================================================
void readMQ135() {
  int rawValue = analogRead(MQ135_PIN);
  // Map raw ADC value (0-4095) to AQI (0-500)
  sensorData.aqi = map(rawValue, 0, 4095, 0, 500);
}

// ============================================================================
// MQ2: SMOKE / LPG LEVEL
// ============================================================================
void readMQ2() {
  int rawValue = analogRead(MQ2_PIN);
  // Map raw ADC value (0-4095) to smoke level (0-1000)
  sensorData.smoke = map(rawValue, 0, 4095, 0, 1000);
}

// ============================================================================
// MQ7: CARBON MONOXIDE LEVEL
// ============================================================================
void readMQ7() {
  int rawValue = analogRead(MQ7_PIN);
  // Map raw ADC value (0-4095) to CO level (0-1000)
  sensorData.co = map(rawValue, 0, 4095, 0, 1000);
}

// ============================================================================
// NEO-6M GPS: LATITUDE, LONGITUDE, SPEED
// ============================================================================
void readGPS() {
  if (gps.location.isValid()) {
    sensorData.latitude = gps.location.lat();
    sensorData.longitude = gps.location.lng();
    sensorData.speed = gps.speed.kmph();
  }
}

// ============================================================================
// MPU6050: PITCH, ROLL, VIBRATION
// ============================================================================
void readMPU6050() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate pitch and roll in degrees
  sensorData.pitch = atan2(ay, az) * 180.0 / PI;
  sensorData.roll = atan2(ax, az) * 180.0 / PI;
  
  // Calculate vibration magnitude (acceleration)
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
  sensorData.vibration = (int)(accelMagnitude / 100); // Normalize
}

// ============================================================================
// UPDATE ALL SENSOR DATA
// ============================================================================
void updateSensorData() {
  readBMP180();
  readLM35();
  readMQ135();
  readMQ2();
  readMQ7();
  readGPS();
  readMPU6050();
}

// ============================================================================
// GET SENSOR DATA AS JSON
// ============================================================================
String getSensorDataJSON() {
  StaticJsonDocument<256> doc;
  
  doc["altitude"] = sensorData.altitude;
  doc["temperature"] = sensorData.temperature;
  doc["pressure"] = sensorData.pressure;
  doc["aqi"] = sensorData.aqi;
  doc["co"] = sensorData.co;
  doc["smoke"] = sensorData.smoke;
  doc["latitude"] = sensorData.latitude;
  doc["longitude"] = sensorData.longitude;
  doc["speed"] = sensorData.speed;
  doc["pitch"] = sensorData.pitch;
  doc["roll"] = sensorData.roll;
  doc["vibration"] = sensorData.vibration;
  
  String json;
  serializeJson(doc, json);
  
  return json;
}
