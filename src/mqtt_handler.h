#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define ssid "Weerapat_WiFi"
#define password "00000000"
#define MQTT_SERVER "192.168.137.1"
#define MQTT_PORT 1883

void setup_wifi();
void setup_mqtt(PubSubClient &client);
void connect_to_mqtt(PubSubClient &client);

#endif