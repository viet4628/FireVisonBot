#include "relay.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define RELAY_GPIO     GPIO_NUM_17
/* Set to 1 for active-HIGH relay module, 0 for active-LOW */
#define RELAY_ON_LEVEL 1

static const char *TAG = "RELAY";
static bool relay_state = false;

void relay_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << RELAY_GPIO),
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    relay_off();
    ESP_LOGI(TAG, "Relay initialised on GPIO %d (active %s)",
             RELAY_GPIO, RELAY_ON_LEVEL ? "HIGH" : "LOW");
}

void relay_on(void)
{
    gpio_set_level(RELAY_GPIO, RELAY_ON_LEVEL);
    relay_state = true;
}

void relay_off(void)
{
    gpio_set_level(RELAY_GPIO, !RELAY_ON_LEVEL);
    relay_state = false;
}

bool relay_is_on(void)
{
    return relay_state;
}
