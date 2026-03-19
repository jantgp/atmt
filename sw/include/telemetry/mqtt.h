#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

class Mqtt {
public:
    Mqtt();
    void init(String chipId);
    void send(const String& topic, const String& message);
    void subscribe(const String& topic);
    void loop();
    bool connected();
    void setCallback(std::function<void(char *, byte *, unsigned int)> callback);

private:
    WiFiClient wifiClient;
    PubSubClient mqttClient;
    String chipId;
    void connect();
};

#endif // MQTT_H