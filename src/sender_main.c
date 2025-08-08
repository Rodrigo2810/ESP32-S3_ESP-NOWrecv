#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_timer.h>

#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]

#define MAC_ADDRESS_SIZE 6
#define WIFI_CHANNEL 6
#define SENDER_TAG "SENDER_STA"
#define AP_SSID "ESP32_RECEIVER_AP"
#define AP_PASSWORD "password123"
#define TARGET_PACKETS_PER_SECOND 50
#define MAX_RETRIES 10

typedef struct {
    uint32_t packet_number;
    uint8_t data[64];
} esp_now_packet_t;

static uint8_t receiver_mac[MAC_ADDRESS_SIZE] = {0}; 
static uint32_t packets_sent = 0;
static uint32_t packets_failed = 0;
static uint32_t retries_count = 0;

static void app_esp_now_send_cb(const uint8_t *mac, esp_now_send_status_t status) {
    if(status != ESP_NOW_SEND_SUCCESS) {
        packets_failed++;
        ESP_LOGE(SENDER_TAG, "Failed to send to " MACSTR, MAC2STR(mac));
    } else {
        packets_sent++;
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                             int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(SENDER_TAG, "Connected to AP");
        
        // Получаем MAC адрес AP после подключения
        wifi_ap_record_t ap_info;
        esp_wifi_sta_get_ap_info(&ap_info);
        memcpy(receiver_mac, ap_info.bssid, 6);
        
        ESP_LOGI(SENDER_TAG, "AP MAC: " MACSTR, MAC2STR(receiver_mac));
    }
}

static void wifi_init_sta() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = AP_SSID,
            .password = AP_PASSWORD,
            .channel = WIFI_CHANNEL,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    
    ESP_LOGI(SENDER_TAG, "Connecting to AP %s...", AP_SSID);
}

void sender_task(void *pv) {
    // Ждем пока получим MAC адрес AP
    while(receiver_mac[0] == 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    esp_now_peer_info_t peer = {
        .channel = WIFI_CHANNEL,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false
    };
    memcpy(peer.peer_addr, receiver_mac, 6);
    
    esp_err_t ret = esp_now_add_peer(&peer);
    if(ret != ESP_OK) {
        ESP_LOGE(SENDER_TAG, "Failed to add peer: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    uint32_t packet_counter = 0;
    uint32_t last_stat_time = xTaskGetTickCount();
    const uint32_t interval_us = 1000000 / TARGET_PACKETS_PER_SECOND;
    
    while(1) {
        uint32_t start_time = esp_timer_get_time();
        
        esp_now_packet_t packet;
        packet.packet_number = packet_counter++;
        memset(packet.data, 0xAA, sizeof(packet.data));
        
        for(int retry = 0; retry < MAX_RETRIES; retry++) {
            ret = esp_now_send(receiver_mac, (uint8_t*)&packet, sizeof(packet));
            
            if(ret == ESP_OK) break;
            
            retries_count++;
            ESP_LOGE(SENDER_TAG, "Send error (retry %d): %s", retry+1, esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        uint32_t elapsed = esp_timer_get_time() - start_time;
        if(elapsed < interval_us) {
             vTaskDelay(pdMS_TO_TICKS((interval_us - elapsed) / 1000));
        }
        
        if(xTaskGetTickCount() - last_stat_time >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(SENDER_TAG, "Stats: Sent: %"PRIu32", Failed: %"PRIu32", Retries: %"PRIu32, 
                   packets_sent, packets_failed, retries_count);
            packets_sent = 0;
            packets_failed = 0;
            retries_count = 0;
            last_stat_time = xTaskGetTickCount();
        }

        if (packet_counter == 1000) {
            ESP_LOGI(SENDER_TAG, "Sent 1000 packets.");
            vTaskDelay(pdMS_TO_TICKS(2000));
            packet_counter = 0;
        }
    }
}

void app_main() {
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("wifi", ESP_LOG_VERBOSE);
    esp_log_level_set("esp_now", ESP_LOG_VERBOSE);
    
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();
    
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(app_esp_now_send_cb));
    
    xTaskCreate(sender_task, "sender", 4096, NULL, 5, NULL);
    
    uint8_t mac[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    ESP_LOGI(SENDER_TAG, "STA MAC: " MACSTR, MAC2STR(mac));
}