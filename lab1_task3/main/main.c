#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

esp_timer_handle_t timer;

static void IRAM_ATTR gpio_isr_handler() {
  // Write your ISR here
}

void app_main(void) {
  // Write your code her
}
