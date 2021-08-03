#include "battery.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define TAG "BATT"

#define BATTERY_CHANNEL 6

bool battery_check(void) 
{
    int battery_level;
    battery_level = adc1_get_raw(BATTERY_CHANNEL);
    if (battery_level < 1000) return false;
    else return true;
}

void battery_init(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_CHANNEL, ADC_ATTEN_DB_11);
}