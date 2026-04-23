
void example_wifi_init(void);
void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
void example_espnow_task(void *args);
esp_err_t example_espnow_init(void);
void wifi_init_sta(void);
void print_mac_addr(void);
void set_custom_mac_addr(void);
void espnow_set_pmc_handle(TaskHandle_t handle);
/* typedef struct struct_message; */	
#ifndef FOO_H
#define FOO_H

static TaskHandle_t pmc_handle = NULL;

typedef struct struct_message {// From Rui Santos:https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/
  bool X;
  bool Y;
  bool SW;
  bool SE;
  bool BA;
  bool BB;
  bool BX;
  bool BY;
} struct_message;

// Create a struct_message called myData
#endif

