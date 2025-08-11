#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <string.h>

#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]

#define TAG "SENDER"
#define AP_SSID "ESP32-RECEIVER"
#define AP_PASSWORD "password123"
#define WIFI_CHANNEL 6
#define WIFI_CONNECT_TIMEOUT_MS 30000
#define ESP_NOW_RETRY_COUNT 3

#define UART_PORT UART_NUM_2
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define UART_BAUD_RATE 115200
#define UART_BUFFER_SIZE 1024

// Новые константы для управления пакетами
#define PACKET_SIZE 64
#define TARGET_PACKET_RATE 50  // пакетов в секунду
#define PACKET_INTERVAL_MS (1000 / TARGET_PACKET_RATE)

static uint8_t receiver_mac[6] = {0};
static EventGroupHandle_t wifi_event_group = NULL;
const int WIFI_CONNECTED_BIT = BIT0;
const int WIFI_FAIL_BIT = BIT1;

// Структура для пакета ESP-NOW
typedef struct {
    uint8_t data[PACKET_SIZE];
    uint16_t seq_num;      // порядковый номер пакета
    uint16_t data_length;  // фактическая длина данных
} esp_now_packet_t;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                             int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_CONNECTED) {
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                memcpy(receiver_mac, ap_info.bssid, 6);
                xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
                ESP_LOGI(TAG, "Connected to AP, MAC: " MACSTR, MAC2STR(receiver_mac));
            }
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP address");
    }
}

static esp_err_t initialize_wifi() {
    wifi_event_group = xEventGroupCreate();
    if (!wifi_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (!sta_netif) {
        ESP_LOGE(TAG, "Failed to create WiFi STA interface");
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = AP_SSID,
            .password = AP_PASSWORD,
            .channel = WIFI_CHANNEL,
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "Connecting to WiFi AP %s...", AP_SSID);
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi connection: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

static void espnow_send_task(void *pvParameters) {
    QueueHandle_t uart_queue = (QueueHandle_t)pvParameters;
    uart_event_t event;
    uint8_t uart_buffer[UART_BUFFER_SIZE];
    esp_now_packet_t packet;
    uint16_t packet_counter = 0;
    esp_err_t ret;
    uint32_t last_send_time = 0;
    
    // Ожидаем подключения к WiFi
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, 
                                        WIFI_CONNECTED_BIT,
                                        pdFALSE,
                                        pdTRUE,
                                        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    
    if ((bits & WIFI_CONNECTED_BIT) == 0) {
        ESP_LOGE(TAG, "Failed to connect to WiFi within timeout");
        vTaskDelete(NULL);
        return;
    }

    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    esp_now_peer_info_t peer_info = {
        .channel = WIFI_CHANNEL,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false
    };
    memcpy(peer_info.peer_addr, receiver_mac, 6);

    ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
        esp_now_deinit();
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "ESP-NOW initialized and ready to send data");

    while(1) {
        if(xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            if(event.type == UART_DATA) {
                int len = uart_read_bytes(UART_PORT, uart_buffer, event.size, portMAX_DELAY);
                if(len > 0) {
                    // Обрабатываем полученные данные порциями по PACKET_SIZE байт
                    for (int offset = 0; offset < len; offset += PACKET_SIZE) {
                        // Ждем нужное время для соблюдения скорости отправки
                        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        if (now - last_send_time < PACKET_INTERVAL_MS) {
                            vTaskDelay(pdMS_TO_TICKS(PACKET_INTERVAL_MS - (now - last_send_time)));
                        }
                        
                        // Формируем пакет
                        packet.seq_num = packet_counter++;
                        int chunk_size = (len - offset) > PACKET_SIZE ? PACKET_SIZE : (len - offset);
                        packet.data_length = chunk_size;
                        memcpy(packet.data, uart_buffer + offset, chunk_size);
                        
                        // Отправляем пакет
                        for (int i = 0; i < ESP_NOW_RETRY_COUNT; i++) {
                            ret = esp_now_send(receiver_mac, (uint8_t*)&packet, sizeof(packet));
                            if (ret == ESP_OK) {
                                ESP_LOGI(TAG, "Sent packet #%d (%d bytes) to " MACSTR, 
                                       packet.seq_num, packet.data_length, MAC2STR(receiver_mac));
                                last_send_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                                break;
                            }
                            ESP_LOGW(TAG, "Send attempt %d failed: %s", i+1, esp_err_to_name(ret));
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                        
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to send packet #%d after %d attempts", 
                                   packet.seq_num, ESP_NOW_RETRY_COUNT);
                        }
                    }
                }
            }
        }
    }
}


void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, 
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUFFER_SIZE, 
                                      UART_BUFFER_SIZE, 10, &uart_queue, 0));

    uart_write_bytes(UART_PORT, "UART test\n", 10);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    if (initialize_wifi() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    if (xTaskCreate(espnow_send_task, "espnow_send_task", 4096, uart_queue, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ESP-NOW send task");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
    
    ESP_LOGI(TAG, "System initialized successfully");

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}