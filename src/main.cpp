#include <Arduino.h>
#include <ArduinoJson.h> // เพิ่ม Library นี้
#include "queue-management.cpp"
#include "mqtt_handler.h"

WiFiClient espClient;
PubSubClient client(espClient);

JobQueue Queue;

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.printf("MQTT Message arrived on topic: %s\n", topic);
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error)
    {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }
    Serial.printf("Converted payload to JSON successfully.\n");
    if (!strcmp(topic, "wirecutter/queue/add"))
    {
        if (doc["command"] == "add_queue")
        {

            Serial.printf("Adding to queue: %d\n", Queue.get_count());
            Queue.push_back(doc["total_len"], doc["front_len"], doc["back_len"], doc["quantity"]);
            Serial.printf("Added to queue. Current count: %d\n", Queue.get_count());
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 18, 17); // RX=18, TX=17
    while (!Serial)
        Serial.println("--- SYSTEM STARTING ---");
    setup_wifi();
    setup_mqtt(client);
    connect_to_mqtt(client);
    client.setCallback(callback);
}

void loop()
{
    if (!client.connected())
    {
        connect_to_mqtt(client);
    }
    client.loop();

    // --- ส่วนของ Serial2 (UART) เดิมของคุณ ---
    // ใน loop() ของ S3
    if (Serial2.available())
    {
        String resp = Serial2.readStringUntil('\n');
        resp.trim();
        Serial.print("Raw data from Slave: ");
        Serial.println(resp);

        if (resp.length() > 0)
        {
            StaticJsonDocument<200> responseDoc;
            responseDoc["result"] = resp.toInt();
            responseDoc["status"] = "success";
            char buffer[256];
            serializeJson(responseDoc, buffer);
            client.publish("wirecut/topic/status/esp", buffer);
            Serial.println("Sent Response back to Node-RED: " + String(buffer));
        }
    }
}