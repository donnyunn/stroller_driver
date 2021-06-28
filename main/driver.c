#include "driver.h"

#include "driver/ledc.h"
#include "driver/dac.h"

#define TAG "motor driver"

#define VR1_IO  GPIO_NUM_18
#define VR2_IO  GPIO_NUM_19
#define ZF1_IO  GPIO_NUM_4
#define ZF2_IO  GPIO_NUM_5
#define EL1_IO  GPIO_NUM_25
#define EL2_IO  GPIO_NUM_26

#define LEDC_CH_NUM       (2)
#define LEDC_FADE_TIME    (50)

ledc_channel_config_t ledc_channel[LEDC_CH_NUM] = {
    {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = VR1_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = LEDC_CHANNEL_1,
        .duty = 0,
        .gpio_num = VR2_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    },
};

brake_mode_t brake_mode = BRAKE_MODE_1;
uint16_t speed_feedback[2] = {0,};

void driver_set_direction(bool dir)
{
    if (dir) {
        gpio_set_level(ZF1_IO, 0);
        gpio_set_level(ZF2_IO, 0);
    } else {
        gpio_set_level(ZF1_IO, 1);
        gpio_set_level(ZF2_IO, 1);
    }
}

void driver_set_speed(uint16_t speed, int16_t steering)
{
    uint32_t duty[2];
    int16_t diff;

    // variable limit
    if (speed > 2047) speed = 2047;
    else if (speed < 16) speed = 0;

    if (speed != 0) {
        // release brake
        driver_set_brake(BRAKE_MODE_NONE);

        // calc duty with differencial
        diff = (int16_t)(steering / 4);
        if (diff > 16) {
            duty[0] = speed;
            duty[1] = speed - diff;
        } else if (diff < -16) {
            duty[0] = speed + diff;
            duty[1] = speed;
        } else {
            duty[0] = speed;
            duty[1] = speed;
        }

        // start pwm
        ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, duty[0]);
        ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, duty[1]);
        ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
        ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
        // ledc_set_fade_with_time(ledc_channel[0].speed_mode, ledc_channel[0].channel, speed, LEDC_FADE_TIME);
        // ledc_set_fade_with_time(ledc_channel[1].speed_mode, ledc_channel[1].channel, speed, LEDC_FADE_TIME);
        // ledc_fade_start(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_FADE_NO_WAIT);
        // ledc_fade_start(ledc_channel[1].speed_mode, ledc_channel[1].channel, LEDC_FADE_NO_WAIT);        
        
    } else {
        // stop pwm
        ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
        ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, 0);
        ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
        ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
        
        // hold brake
        driver_set_brake(brake_mode);
    }
}

void driver_set_brake(brake_mode_t brake)
{
    switch (brake) {
        case BRAKE_MODE_NONE:
            dac_output_voltage(DAC_CHANNEL_1, 0);
            dac_output_voltage(DAC_CHANNEL_2, 0);
            break;
        case BRAKE_MODE_1:
            dac_output_voltage(DAC_CHANNEL_1, 64);
            dac_output_voltage(DAC_CHANNEL_2, 64);
            brake_mode = BRAKE_MODE_1;
        break;
        case BRAKE_MODE_2:
            dac_output_voltage(DAC_CHANNEL_1, 128);
            dac_output_voltage(DAC_CHANNEL_2, 128);
            brake_mode = BRAKE_MODE_2;
        break;
        case BRAKE_MODE_3:
            dac_output_voltage(DAC_CHANNEL_1, 255);
            dac_output_voltage(DAC_CHANNEL_2, 255);
            brake_mode = BRAKE_MODE_3;
        break;
        case BRAKE_MODE_MAX:
            dac_output_voltage(DAC_CHANNEL_1, 255);
            dac_output_voltage(DAC_CHANNEL_2, 255);
        break;
    }
}

brake_mode_t driver_get_brake(void)
{
    return brake_mode;
}

void driver_init(void)
{
    gpio_config_t io_conf;
    int ch;

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_11_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer_config(&ledc_timer);

    for (ch = 0; ch < LEDC_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
    ledc_fade_func_install(0);

    // set pwm 0
    ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
    ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, 0);
    ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
    ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
        
    // set brake
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);
    driver_set_brake(BRAKE_MODE_NONE);

    // set direction
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL<<ZF1_IO) | (1ULL<<ZF2_IO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    driver_set_direction(true);
    // xTaskCreate(driver_test, "driver_test", 2048, (void *) 0, 10, NULL);
}