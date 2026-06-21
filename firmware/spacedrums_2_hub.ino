#include <esp_now.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef struct struct_message {
    uint8_t stick_id;
    uint8_t drum_id;
    uint8_t velocity;
} struct_message;

// Create a queue to hold up to 20 rapid-fire hits
QueueHandle_t hubQueue;

// 🚨 ESP32 Core v3.x Callback 🚨
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    struct_message hit;
    memcpy(&hit, incomingData, sizeof(hit));
    
    // OFF-LOAD TO QUEUE INSTANTLY! DO NOT SERIAL.PRINT HERE!
    // This frees the Wi-Fi stack in ~1 microsecond to catch the other stick's packet.
    xQueueSend(hubQueue, &hit, 0);
}

void setup() {
    Serial.begin(500000);
    WiFi.mode(WIFI_STA);

    // Initialize the FreeRTOS Queue
    hubQueue = xQueueCreate(20, sizeof(struct_message));

    delay(2000);
    Serial.print("HUB MAC ADDRESS: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    struct_message hit;
    
    // Block the loop efficiently until a hit appears in the queue
    if (xQueueReceive(hubQueue, &hit, portMAX_DELAY) == pdPASS) {
        // Safe to take our time printing here, the Wi-Fi task is untouched
        Serial.print("H,");
        Serial.print(hit.stick_id);
        Serial.print(",");
        Serial.print(hit.drum_id);
        Serial.print(",");
        Serial.println(hit.velocity);
    }
}