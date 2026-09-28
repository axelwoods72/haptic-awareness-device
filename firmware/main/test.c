#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void app_main(void) {
  vTaskDelay(pdMS_TO_TICKS(2000));

  int counter = 0;
  while (1) {
    ESP_LOGI(TAG, "alive: %d", counter++);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}