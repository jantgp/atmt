#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <variables/setget.h>
#include <sensors/usensor.h>
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
};

struct FilteredSensors {
  float ul = NAN;
  float ur = NAN;
  float uf = NAN;
  float ub = NAN;
};

RawSensors g_raw;
FilteredSensors g_filt;

static const float US_ALPHA = 0.35f;

// -----------------------------
// EMA helper
// -----------------------------
static float ema(float prev, float current, float alpha) {
  if (isnan(prev))    return current;
  if (isnan(current)) return prev;
  return alpha * current + (1.0f - alpha) * prev;
}

// -----------------------------
// Sensor read helpers
// -----------------------------
static float readUltrasonicLeftCm()  { return (float)globalVar_get(rawDistLeft,  &age); }
static float readUltrasonicRightCm() { return (float)globalVar_get(rawDistRight, &age); }
static float readUltrasonicFrontCm() { return (float)globalVar_get(rawDistFront, &age); }
static float readUltrasonicBackCm()  { return (float)globalVar_get(rawDistBack,  &age); }

// -----------------------------
// Sensor filter
// -----------------------------
static void filterSensors(const RawSensors& raw, FilteredSensors& filt) {
  filt.ul = ema(filt.ul, raw.ul, US_ALPHA);
  filt.ur = ema(filt.ur, raw.ur, US_ALPHA);
  filt.uf = ema(filt.uf, raw.uf, US_ALPHA);
  filt.ub = ema(filt.ub, raw.ub, US_ALPHA);
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
    "\"yaw_rate\":0.00,"
    "\"heading\":0.0,"
    "\"compass\":0.0,"
    "\"mag_x\":0.00,"
    "\"mag_y\":0.00,"
    "\"mag_z\":0.00,"
    "\"acc_x\":0.00,"
    "\"acc_y\":0.00,"
    "\"acc_z\":0.00,"
    "\"width\":%.1f,"
    "\"center_error\":%.1f,"
    "\"front_blocked\":%s,"
    "\"cmd_pwm\":%d,"
    "\"cmd_steer\":\"%s\""
    "}",
    chipid.c_str(), (unsigned)g_seq++, (unsigned long)nowMs,
    filt.ul, filt.ur, filt.uf, filt.ub,
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
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  mqtt.init(chipid);
  mqtt.setCallback(mqttMessageCallback);
  mqtt.send("test", "Hello World");
  mqtt.subscribe("control");

  delay(1000);
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
      g_raw.ul = readUltrasonicLeftCm();
      g_raw.ur = readUltrasonicRightCm();
      g_raw.uf = readUltrasonicFrontCm();
      g_raw.ub = readUltrasonicBackCm();
      filterSensors(g_raw, g_filt);
    }
  }

  // Publish filtered sensor data to MQTT at 2 Hz
  static uint32_t lastPublish = 0;
  if ((nowMs - lastPublish) > 500) {
    lastPublish = nowMs;
    publishSensorData(g_filt, nowMs);
  }

  // Safety stop if front or back is too close
  if (!isnan(g_filt.uf) && g_filt.uf < 20.0f) {
    motor.driving(0);
  }
  if (!isnan(g_filt.ub) && g_filt.ub < 20.0f) {
    motor.driving(0);
  }
}
