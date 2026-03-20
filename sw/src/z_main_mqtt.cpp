#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <PubSubClient.h>
#include <variables/setget.h>
#include <sensors/usensor.h>
#include <sensors/accsensor.h>
#include <sensors/compass.h>
#include <actuators/motor.h>
#include <actuators/steer.h>
#include <telemetry/mqtt.h>
#include <secrets.h>
#include <atmio.h>

String chipid;
Steer steer;
Motor motor;
Mqtt mqtt;
Usensor ultraSound;
ACCsensor accelSensor;
Compass compass;

long int age;
uint32_t g_seq = 0;
int g_cmdPwm = 0;
const char* g_cmdSteer = "STRAIGHT";

// -----------------------------
// Sensor structs
// -----------------------------
struct RawSensors {
  float ul = NAN;
  float ur = NAN;
  float uf = NAN;
  float ub = NAN;
  float gyX = NAN;
  float gyY = NAN;
  float gyZ = NAN;
  float accX = NAN;
  float accY = NAN;
  float accZ = NAN;
  float magX = NAN;
  float magY = NAN;
  float magZ = NAN;
  float heading = NAN;
};

struct FilteredSensors {
  float ul = NAN;
  float ur = NAN;
  float uf = NAN;
  float ub = NAN;
  float gyX = NAN;
  float gyY = NAN;
  float gyZ = NAN;
  float accX = NAN;
  float accY = NAN;
  float accZ = NAN;
  float magX = NAN;
  float magY = NAN;
  float magZ = NAN;
  float heading = NAN;
};

RawSensors g_raw;
FilteredSensors g_filt;

static const float US_ALPHA  = 0.35f;
static const float IMU_ALPHA = 0.25f;

// -----------------------------
// EMA helpers
// -----------------------------
static float ema(float prev, float current, float alpha) {
  if (isnan(prev))    return current;
  if (isnan(current)) return prev;
  return alpha * current + (1.0f - alpha) * prev;
}

// Circular EMA for angles: always interpolates via the shortest arc.
// Without this, crossing 0°/360° makes the filter traverse the long way
// around (e.g. 350°→2° would pass through ~270° instead of staying near 0°).
static float ema_angle(float prev, float current, float alpha) {
  if (isnan(prev))    return current;
  if (isnan(current)) return prev;
  float diff = current - prev;
  if (diff >  180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;
  float result = prev + alpha * diff;
  if (result <    0.0f) result += 360.0f;
  if (result >= 360.0f) result -= 360.0f;
  return result;
}

// -----------------------------
// Sensor read helpers
// -----------------------------
static float readUltrasonicLeftCm()  { return (float)globalVar_get(rawDistLeft,  &age); }
static float readUltrasonicRightCm() { return (float)globalVar_get(rawDistRight, &age); }
static float readUltrasonicFrontCm() { return (float)globalVar_get(rawDistFront, &age); }
static float readUltrasonicBackCm()  { return (float)globalVar_get(rawDistBack,  &age); }
// rawGyX/Y are stored as raw/13 in accsensor.cpp; convert to deg/s (sensitivity 131 LSB/deg/s)
static float readGyroX()  { return globalVar_get(rawGyX, &age) / 10.08f; }
static float readGyroY()  { return globalVar_get(rawGyY, &age) / 10.08f; }
// rawGyZ is already stored as deg/s in accsensor.cpp (divided by 131 there)
static float readGyroZ()  { return (float)globalVar_get(rawGyZ, &age); }
// Convert raw ADC counts to g (±2g range => 16384 LSB/g)
static float readAccX()   { return globalVar_get(rawAccX, &age) / 16384.0f; }
static float readAccY()   { return globalVar_get(rawAccY, &age) / 16384.0f; }
static float readAccZ()   { return globalVar_get(rawAccZ, &age) / 16384.0f; }
static float readMagX()   { return globalVar_get(rawMagX, &age) / 100.0f; }
static float readMagY()   { return globalVar_get(rawMagY, &age) / 100.0f; }
static float readMagZ()   { return globalVar_get(rawMagZ, &age) / 100.0f; }
static float readHeading() {
  long h = globalVar_get(calcHeading, &age);
  return (h == -1) ? NAN : h / 10.0f;  // stored as 1/10 degrees
}

// -----------------------------
// Sensor filter
// -----------------------------
static void filterSensors(const RawSensors& raw, FilteredSensors& filt) {
  filt.ul   = ema(filt.ul,   raw.ul,   US_ALPHA);
  filt.ur   = ema(filt.ur,   raw.ur,   US_ALPHA);
  filt.uf   = ema(filt.uf,   raw.uf,   US_ALPHA);
  filt.ub   = ema(filt.ub,   raw.ub,   US_ALPHA);
  filt.gyX  = ema(filt.gyX,  raw.gyX,  IMU_ALPHA);
  filt.gyY  = ema(filt.gyY,  raw.gyY,  IMU_ALPHA);
  filt.gyZ  = ema(filt.gyZ,  raw.gyZ,  IMU_ALPHA);
  filt.accX    = ema(filt.accX,    raw.accX,    IMU_ALPHA);
  filt.accY    = ema(filt.accY,    raw.accY,    IMU_ALPHA);
  filt.accZ    = ema(filt.accZ,    raw.accZ,    IMU_ALPHA);
  filt.magX    = ema(filt.magX,    raw.magX,    IMU_ALPHA);
  filt.magY    = ema(filt.magY,    raw.magY,    IMU_ALPHA);
  filt.magZ    = ema(filt.magZ,    raw.magZ,    IMU_ALPHA);
  filt.heading = ema_angle(filt.heading, raw.heading, IMU_ALPHA);
}

// -----------------------------
// JSON publish
// -----------------------------
static void publishSensorData(const FilteredSensors& filt, uint32_t nowMs) {
  if (ESP.getFreeHeap() < 8000) return;

  float width        = filt.ul + filt.ur;
  float centerError  = filt.ur - filt.ul;
  bool frontBlocked  = !isnan(filt.uf) && filt.uf < 20.0f;

  static char jsonStr[512];
  int written = snprintf(jsonStr, sizeof(jsonStr),
    "{"
    "\"truck_id\":\"%s\","
    "\"seq\":%u,"
    "\"t_ms\":%lu,"
    "\"mode\":\"EXPLORE\","
    "\"ul\":%.1f,"
    "\"ur\":%.1f,"
    "\"uf\":%.1f,"
    "\"ub\":%.1f,"
    "\"yaw_rate\":%.2f,"
    "\"gy_x\":%.2f,"
    "\"gy_y\":%.2f,"
    "\"gy_z\":%.2f,"
    "\"heading\":%.1f,"
    "\"compass\":%.1f,"
    "\"mag_x\":%.2f,"
    "\"mag_y\":%.2f,"
    "\"mag_z\":%.2f,"
    "\"acc_x\":%.2f,"
    "\"acc_y\":%.2f,"
    "\"acc_z\":%.2f,"
    "\"width\":%.1f,"
    "\"center_error\":%.1f,"
    "\"front_blocked\":%s,"
    "\"cmd_pwm\":%d,"
    "\"cmd_steer\":\"%s\""
    "}",
    chipid.c_str(), (unsigned)g_seq++, (unsigned long)nowMs,
    filt.ul, filt.ur, filt.uf, filt.ub,
    filt.gyZ,
    filt.gyX, filt.gyY, filt.gyZ,
    filt.heading, filt.heading,
    filt.magX, filt.magY, filt.magZ,
    filt.accX, filt.accY, filt.accZ,
    width, centerError,
    frontBlocked ? "true" : "false",
    g_cmdPwm, g_cmdSteer
  );

  if (written < (int)sizeof(jsonStr)) {
    mqtt.send("telemetry", jsonStr);
  }
}

// -----------------------------
// MQTT callback (motor / steer control)
// -----------------------------
void mqttMessageCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message: ");
  Serial.println(message);

  int motorIdx = message.indexOf("\"motor\"");
  if (motorIdx >= 0) {
    int colon = message.indexOf(':', motorIdx);
    if (colon >= 0) {
      g_cmdPwm = message.substring(colon + 1).toInt();
      Serial.print("Setting motor speed to: ");
      Serial.println(g_cmdPwm);
      motor.driving(g_cmdPwm);
    }
  }

  int steerIdx = message.indexOf("\"direction\"");
  if (steerIdx >= 0) {
    int colon = message.indexOf(':', steerIdx);
    if (colon >= 0) {
      int steerVal = message.substring(colon + 1).toInt();
      if (steerVal > 0)       g_cmdSteer = "RIGHT";
      else if (steerVal < 0)  g_cmdSteer = "LEFT";
      else                    g_cmdSteer = "STRAIGHT";
      Serial.print("Setting steer direction to: ");
      Serial.println(steerVal);
      steer.direction(steerVal);
    }
  }
}

// -----------------------------
// Setup / loop
// -----------------------------
void setup()
{
  Serial.begin(57600);
  uint64_t chipIdHex = ESP.getEfuseMac();
  chipid = String((uint32_t)(chipIdHex >> 32), HEX) + String((uint32_t)chipIdHex, HEX);

  Serial.println("******************************************************");
  Serial.print("ESP32 Chip ID: ");
  Serial.println(chipid);
  Serial.println("******************************************************");

  globalVar_init();
    Serial.println("******************************************************");

  steer.Begin();
  Serial.println("******************************************************");

  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println("******************************************************");

  accelSensor.Begin();
  delay(500);
  compass.Begin();
  delay(500);

  ultraSound.open(TRIGGER_PIN1, ECHO_PIN1, rawDistFront);
  delay(100);
  ultraSound.open(TRIGGER_PIN2, ECHO_PIN2, rawDistRight);
  delay(100);
  ultraSound.open(TRIGGER_PIN3, ECHO_PIN3, rawDistLeft);
  delay(100);
  ultraSound.open(TRIGGER_PIN4, ECHO_PIN4, rawDistBack);
  delay(100);
  Serial.println("******************************************************");
  delay(2000);
  // Disable WiFi modem sleep before connecting — prevents the WiFi stack from
  // temporarily disabling the flash cache (which crashes any ISR executing Flash code)
  WiFi.persistent(false);              // MUST be before WiFi.mode() — prevents credential NVS writes
  esp_wifi_set_storage(WIFI_STORAGE_RAM); // ALL WiFi state in RAM — no background NVS flash writes ever
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  mqtt.init(chipid);
  mqtt.setCallback(mqttMessageCallback);
  mqtt.send("test", "Hello World");
  mqtt.subscribe("control");

  delay(500);
}

void loop()
{
  const uint32_t nowMs = millis();

  // MQTT loop at 10 Hz
  static uint32_t lastMqttLoop = 0;
  if ((nowMs - lastMqttLoop) > 100) {
    lastMqttLoop = nowMs;
    mqtt.loop();
  }

  // Read and filter sensors at 20 Hz
  static uint32_t lastSensorRead = 0;
  if ((nowMs - lastSensorRead) > 50) {
    lastSensorRead = nowMs;

    if (ESP.getFreeHeap() > 9000) {
      g_raw.ul   = readUltrasonicLeftCm();
      g_raw.ur   = readUltrasonicRightCm();
      g_raw.uf   = readUltrasonicFrontCm();
      g_raw.ub   = readUltrasonicBackCm();
      g_raw.gyX  = readGyroX();
      g_raw.gyY  = readGyroY();
      g_raw.gyZ  = readGyroZ();
      g_raw.accX    = readAccX();
      g_raw.accY    = readAccY();
      g_raw.accZ    = readAccZ();
      g_raw.magX    = readMagX();
      g_raw.magY    = readMagY();
      g_raw.magZ    = readMagZ();
      g_raw.heading = readHeading();
      filterSensors(g_raw, g_filt);
    }
  }

  // Publish filtered sensor data to MQTT at 2 Hz
  static uint32_t lastPublish = 0;
  if ((nowMs - lastPublish) > 500) {
    lastPublish = nowMs;
    publishSensorData(g_filt, nowMs);
  }

  // Safety stop if front or back is too close, but still allow reversing away.
  if (!isnan(g_filt.uf) && g_filt.uf < 20.0f && g_cmdPwm > 0) {
    motor.driving(0);
  }
  if (!isnan(g_filt.ub) && g_filt.ub < 20.0f && g_cmdPwm < 0) {
    motor.driving(0);
  }
}
