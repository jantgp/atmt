#include "telemetry/mqtt.h"
#include <secrets.h>
#include <esp_wifi.h>

Mqtt::Mqtt() : mqttClient(wifiClient)
{
    mqttClient.setServer(mqtt_server, atoi(mqtt_port));
    mqttClient.setBufferSize(512);
}

void Mqtt::init(String chipId)
{
    Mqtt::chipId = chipId;
    // Disable WiFi modem sleep — prevents cache from being disabled
    // when the radio wakes up, which causes "Cache disabled but cached
    // memory region accessed" crashes in ISR context.
    esp_wifi_set_ps(WIFI_PS_NONE);
}

bool Mqtt::connected()
{
    return mqttClient.connected();
}

void Mqtt::subscribe(const String &topic)
{
    if (!mqttClient.connected())
    {
        connect();
    }
    // Use a fixed buffer instead of String concatenation
    char fullTopic[128];
    snprintf(fullTopic, sizeof(fullTopic), "%s/%s", chipId.c_str(), topic.c_str());
    mqttClient.subscribe(fullTopic);
}

void Mqtt::connect()
{
    while (!mqttClient.connected())
    {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect(Mqtt::chipId.c_str(), mqtt_user, mqtt_pass))
        {
            Serial.println("connected");
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(mqttClient.state());
            delay(2000);
        }
    }
}

void Mqtt::send(const String &topic, const String &message)
{
    if (!mqttClient.connected())
    {
        Serial.println("MQTT not connected, reconnecting...");
        connect();
        if (!mqttClient.connected())
        {
            Serial.println("Reconnect failed, dropping message");
            return;
        }
    }

    // Use a fixed stack buffer to avoid heap allocation during publish
    char fullTopic[128];
    snprintf(fullTopic, sizeof(fullTopic), "%s/%s", chipId.c_str(), topic.c_str());

    bool ok = mqttClient.publish(fullTopic, message.c_str());
    mqttClient.loop();

    if (ok)
        Serial.printf("Sent => %s (%d bytes)\n", fullTopic, message.length());
    else
        Serial.printf("publish() failed — payload: %d bytes, buffer: %d\n",
                      message.length(), mqttClient.getBufferSize());
}

void Mqtt::loop()
{
    mqttClient.loop();
}

void Mqtt::setCallback(std::function<void(char *, byte *, unsigned int)> callback)
{
    mqttClient.setCallback(callback);
}
