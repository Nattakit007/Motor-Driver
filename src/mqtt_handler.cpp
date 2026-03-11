#include "mqtt_handler.h"


void setup_wifi()
{
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(ssid, password);
        Serial.print("Connecting to WiFi...");
        while (WiFi.status() != WL_CONNECTED){
            vTaskDelay(pdMS_TO_TICKS(500));
            Serial.print(".");
        }

        Serial.println("");
        Serial.print("Connected to WiFi. IP address: ");
        Serial.println(WiFi.localIP());
    }
}

void setup_mqtt(PubSubClient &client) {
    client.setServer(MQTT_SERVER, MQTT_PORT); // ใช้ค่าที่ #define ไว้
}

void connect_to_mqtt(PubSubClient &client) {
    if (WiFi.status() == WL_CONNECTED && !client.connected()){
        Serial.print("Attempting MQTT connection... "); // ตรงนี้จะไม่ Error แล้ว!
        if (client.connect("ESP32S3_Master")) {
            Serial.println("CONNECTED!");
            //-------------------------------------------------------------------------------
            client.subscribe("wirecutter/command");
            client.subscribe("wirecutter/control");
            //-------------------------------------------------------------------------------
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}