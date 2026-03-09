#include "mqtt_handler.h"


void setup_wifi()
{
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi. IP address: ");
    Serial.println(WiFi.localIP());
}

void setup_mqtt(PubSubClient &client) {
    client.setServer(MQTT_SERVER, MQTT_PORT); // ใช้ค่าที่ #define ไว้
}

void connect_to_mqtt(PubSubClient &client) {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection... "); // ตรงนี้จะไม่ Error แล้ว!
        if (client.connect("ESP32S3_Master")) {
            Serial.println("CONNECTED!");
            client.subscribe("wirecutter/queue/add");
            client.subscribe("wirecutter/control");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            delay(5000);
        }
    }
}