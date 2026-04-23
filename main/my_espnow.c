// ESPNOW
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "my_espnow.h"


#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#define CONFIG_ESPNOW_PMK "pmk1234567890123"
#define CONFIG_ESPNOW_LMK "lmk1234567890123"
#define CONFIG_ESPNOW_CHANNEL 1
#define CONFIG_ESPNOW_SEND_COUNT 100
#define CONFIG_ESPNOW_SEND_DELAY 1000
#define CONFIG_ESPNOW_SEND_LEN 10

#define ESPNOW_QUEUE_SIZE           6
#define ESPNOW_MAXDELAY 512
#define EXAMPLE_ESP_WIFI_SSID      "asdas"
#define EXAMPLE_ESP_WIFI_PASS      "asdas"
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

static const char *TAG = "espnow_example";
static uint8_t receiverMacAddress[ESP_NOW_ETH_ALEN] = {0x34, 0xb7, 0xda, 0x5e, 0x9e, 0x28  };
static uint8_t robotMacAddress[ESP_NOW_ETH_ALEN] = {0x34, 0xb7, 0xda, 0x5e, 0xae, 0xa8}; // Was the mac address of the AtomS3lite robot


struct_message myData = {0.0,0.0,false,false,false,false,false,false};
/* WiFi should start before using ESPNOW */
void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
}

void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info->src_addr == NULL || data == NULL || len <= 0) return;
    if (pmc_handle == NULL) return;

    memcpy(&myData, data, sizeof(myData));

    uint32_t set_bits =
        (myData.SW)       |
        (myData.SE << 1)  |
        (myData.BA << 2)  |
        (myData.BB << 3)  |
        (myData.BX << 4)  |
        (myData.BY << 5)  |
        (myData.X  << 6)  |
        (myData.Y  << 7);

    uint32_t clear_bits = (~set_bits) & 0xFF;

    xTaskNotify(pmc_handle, set_bits, eSetBits);
    ulTaskNotifyValueClear(pmc_handle, clear_bits);
}



esp_err_t example_espnow_init(void)
{

	/* Initialize ESPNOW and register sending and receiving callback function. */
	ESP_ERROR_CHECK( esp_now_init() );
	ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
	ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );

	/* Set primary master key. */
	ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

	/* Add broadcast peer information to peer list. */
	esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
	if (peer == NULL) {
		ESP_LOGE(TAG, "Malloc peer information fail");
		return ESP_FAIL;
	}
	memset(peer, 0, sizeof(esp_now_peer_info_t));
	peer->channel = CONFIG_ESPNOW_CHANNEL;
	peer->ifidx = ESPNOW_WIFI_IF;
	peer->encrypt = false;
	memcpy(peer->peer_addr, receiverMacAddress, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK( esp_now_add_peer(peer) );
	free(peer);

	return ESP_OK;
}

// my_espnow.c
void espnow_set_pmc_handle(TaskHandle_t handle) {
    pmc_handle = handle;
}

void wifi_init_sta(void)
{
	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = EXAMPLE_ESP_WIFI_SSID,
			.password = EXAMPLE_ESP_WIFI_PASS,
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
	ESP_ERROR_CHECK(esp_wifi_start() );

	ESP_LOGI(TAG, "wifi_init_sta finished.");
}

void set_custom_mac_addr(void)
{
	esp_base_mac_addr_set(&robotMacAddress[0]);
}


void print_mac_addr(void)
{
	uint8_t base_mac_addr[6] = {0};
	esp_err_t retu = ESP_OK;
	retu = esp_read_mac(base_mac_addr, ESP_MAC_WIFI_STA);
	for (int i = 0; i < 6; i++) {
		printf("0x%x, ",base_mac_addr[i]);

	}
}
