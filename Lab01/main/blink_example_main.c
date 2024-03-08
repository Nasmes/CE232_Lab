#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "Lab01";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define LED_GPIO 2

static uint8_t s_led_state = 0;

static void blink_led(uint8_t GPIO, uint8_t *led_state)
{
    ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");

    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(GPIO, *led_state);

    /* Toggle the LED state */
    *led_state = !*led_state;
}

static void configure_led(uint8_t GPIO)
{
    gpio_reset_pin(GPIO);
    gpio_set_direction(GPIO, GPIO_MODE_OUTPUT); /* Set the GPIO as a push/pull output */

    ESP_LOGI(TAG, "LED configured!");
}

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led(LED_GPIO);

    while (1)
    {
        blink_led(LED_GPIO, &s_led_state);
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
