#include <Arduino.h>
#include <ArduinoJson.h>
#include "mqtt_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

WiFiClient espClient;
PubSubClient client(espClient);
QueueHandle_t jobQueue;

class JobData {
public:
    String size;
    float total_len;
    float front_len;
    float back_len;
    int qty;
};

void callback(char *topic, byte *payload, unsigned int length){
    Serial.printf("Message arrived on topic: %s\n", topic);
    if (!strcmp(topic, "wirecutter/command"))
    {
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, payload, length);
        if (error) {
                Serial.printf("JSON Error: %s\n", error.c_str());
                return;}
        Serial.printf("Converted payload to JSON successfully.\n");
        const char* cmd = doc["command"];
        if (cmd == nullptr) {
            Serial.println("Command field is missing in JSON.");
            return;
        }

        //-------------------------------------------------------------
        if (!strcmp(cmd, "add_queue")){
            JobData newJob;
            newJob.size = doc["size"].as<String>();
            newJob.total_len = doc["total_length"].as<float>();
            newJob.front_len = doc["front_length"].as<float>();
            newJob.back_len = doc["back_length"].as<float>();
            newJob.qty = doc["quantity"].as<int>();

            if (xQueueSend(jobQueue,&newJob,0) == pdPASS){
                Serial.print("New Queue Confirm ");
                Serial.print(uxQueueMessagesWaiting(jobQueue));
                Serial.println("/10");
            }
            else {Serial.println("Queue Full");}
        }
        else if (!strcmp(cmd, "clear_queue")){
            if (uxQueueMessagesWaiting(jobQueue) > 0){
                xQueueReset(jobQueue);
                Serial.println("Queue Cleared");
            }
            else {Serial.println("Queue Empty");}
        }
        else {
            Serial.printf("Unknown command: %s\n", cmd);
        }
    }
}

void Task_MQTT(void *pvParameters) {
    setup_mqtt(client);
    client.setCallback(callback);
    while(1) {
        setup_wifi();
        connect_to_mqtt(client);
        client.loop();
        vTaskDelay(pdMS_TO_TICKS(10));
        }

}

void Task_Machine(void *pvParameters)
{
    JobData currentJob;
    
    while (1){
        if (uxQueueMessagesWaiting(jobQueue) == 0) {
            Serial.println("All jobs completed. Waiting for new jobs...");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (xQueueReceive(jobQueue, &currentJob, portMAX_DELAY) == pdPASS)
        {
            Serial.println("\n---Start NewJob---");
            Serial.printf("Size: %s | Total: %.1f | Front: %.1f | Back: %.1f | Qty: %d\n",
                          currentJob.size, currentJob.total_len, currentJob.front_len,
                          currentJob.back_len, currentJob.qty);

            // ประกอบร่างเป็น CSV String ส่งให้บอร์ด Slave (STM32)
            StaticJsonDocument<256> doc;
            doc["size"] = currentJob.size;
            doc["total_length"] = currentJob.total_len;
            doc["front_length"] = currentJob.front_len;
            doc["back_length"] = currentJob.back_len;
            doc["quantity"] = currentJob.qty;

            String payload;
            serializeJson(doc, payload);

            Serial2.println(payload);
            Serial.println("Sent command to Slave: " + payload);

            while(1){
                if (Serial2.available()) {
                    
                    String response = Serial2.readStringUntil('\n');
                    response.trim();
                    int commaIndex = response.indexOf(',');

                    Serial.println("Received from Slave: " + response);

                    if (commaIndex != -1) {
                        String cmd = response.substring(0, commaIndex);
                        
                        if (cmd == "done") {
                            Serial.println("Job completed by Slave.");
                            break; // ออกจาก loop รอรับคำสั่งใหม่
                            }
                        else if (cmd == "pg") {
                            String value = response.substring(commaIndex + 1); 
                            int progressValue = value.toInt();
                            Serial.println("Progress: " + String(progressValue) + "/" + String(currentJob.qty));
                            break; // ออกจาก loop รอรับคำสั่งใหม่
                        }
                        else {
                            Serial.println("Unknown response from Slave: " + cmd);
                        }
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            Serial.println("---End Job---\n");
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    while (!Serial) Serial.println("\n--- SYSTEM RTOS STARTING ---");
    
    Serial2.begin(9600, SERIAL_8N1, 6, 7); // RX=6, TX=7

    jobQueue = xQueueCreate(10, sizeof(JobData));
    if (jobQueue == NULL){
        Serial.println("Failed to create queue");
        while (1);
    }

    xTaskCreatePinnedToCore(
        Task_MQTT,       // ชื่อฟังก์ชัน
        "MQTT_Task",     // ชื่อ Task (สำหรับดูตอน Debug)
        8192,            // ขนาด Stack (ใส่เยอะหน่อยเพราะมี WiFi/JSON)
        NULL,            // Parameter ที่ส่งเข้าไป
        1,               // Priority (ความสำคัญ 1 = ปกติ)
        NULL,            // Task Handle
        0                // ผูกติดกับ Core 0
    );

    xTaskCreatePinnedToCore(
        Task_Machine, 
        "Machine_Task", 
        4096,            // Stack size
        NULL, 
        1,               // Priority สูงกว่านิดหน่อย เพื่อให้สั่งมอเตอร์ไม่สะดุด
        NULL, 
        1                // ผูกติดกับ Core 1
    );
}

void loop(){vTaskDelete(NULL);}