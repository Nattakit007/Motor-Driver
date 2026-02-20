#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// --- [ตั้งค่าส่วนตัว] ---
const char* ssid = "WEERAPAT_5G";           // ต้องเป็น 2.4GHz เท่านั้น
const char* password = "00000000";
const char* mqtt_server = "test.mosquitto.org";    // *** เปลี่ยนเป็น IP เครื่องคอมคุณ ***

WiFiClient espClient;
PubSubClient client(espClient);

// ฟังก์ชันเชื่อมต่อ WiFi
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

// ฟังก์ชันเชื่อมต่อ MQTT
void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // สร้าง Client ID แบบสุ่มเพื่อให้ไม่ซ้ำกับเครื่องอื่น
        String clientId = "ESP32S3_Cutter_";
        clientId += String(random(0xffff), HEX);

        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
            client.publish("wirecutter/status", "System Online");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); } // รอ Serial พร้อม (สำหรับ S3)
    
    setup_wifi();
    client.setServer(mqtt_server, 1883);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // ส่ง Hello World ทุกๆ 5 วินาที
    static unsigned long lastMsg = 0;
    unsigned long now = millis();
    if (now - lastMsg > 5000) {
        lastMsg = now;
        
        String msg = "Hello World from ESP32-S3! (Millis: " + String(now) + ")";
        Serial.print("Publish message: ");
        Serial.println(msg);
        
        // ส่งไปที่ Topic สำหรับทดสอบ
        client.publish("kmitl/iot/test", msg.c_str());
    }
}