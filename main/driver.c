#include "driver.h"

#include "driver/ledc.h"
#include "driver/dac.h"

#define TAG "motor driver"

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (3.4179) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (5.78)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

#define VR1_IO  GPIO_NUM_18
#define VR2_IO  GPIO_NUM_19
#define ZF1_IO  GPIO_NUM_4
#define ZF2_IO  GPIO_NUM_5
#define EL1_IO  GPIO_NUM_25
#define EL2_IO  GPIO_NUM_26
#define M1_IO   GPIO_NUM_22
#define M2_IO   GPIO_NUM_23

#define LEDC_CH_NUM       (2)
#define LEDC_FADE_TIME    (10)

#define MAX_SPEED 1000
#define MAX_STEERING 1000

driver_t * this;

// ledc_channel_config_t ledc_channel[LEDC_CH_NUM] = {
//     {
//         .channel = LEDC_CHANNEL_0,
//         .duty = 0,
//         .gpio_num = VR1_IO,
//         .speed_mode = LEDC_HIGH_SPEED_MODE,
//         .hpoint = 0,
//         .timer_sel = LEDC_TIMER_2
//     },
//     {
//         .channel = LEDC_CHANNEL_1,
//         .duty = 0,
//         .gpio_num = VR2_IO,
//         .speed_mode = LEDC_HIGH_SPEED_MODE,
//         .hpoint = 0,
//         .timer_sel = LEDC_TIMER_2
//     },
// };

brake_mode_t brake_mode = BRAKE_MODE_1;
uint16_t speed_feedback[2] = {0,};
uint16_t duty_result[2] = {0,};

xQueueHandle pwm_queue;
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;
xQueueHandle timer_queue;

uint32_t pwm_num;

// static void IRAM_ATTR feedback1_isr_handler(void* arg)
// {
//     speed_feedback[0]++;
// }
static void IRAM_ATTR feedback2_isr_handler(void* arg)
{
    speed_feedback[1]++;
}

uint16_t driver_get_speed(int num) {
    if (num == 0) {
        return speed_feedback[0];
    } else if (num == 1) {
        return speed_feedback[1];
    } else {
        return 0;
    }
}

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

void driver_set_brake_mode(brake_mode_t mode)
{
    brake_mode = mode;
}

brake_mode_t driver_get_brake_mode(void)
{
    return brake_mode;
}

void driver_set_speed(uint16_t speed, int16_t steering)
{
    uint32_t duty[2];
    int16_t diff;
    int16_t corr[2];

    // variable limit
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    else if (speed < 16) speed = 0;
    if (steering > MAX_STEERING) steering = MAX_STEERING;
    else if (steering < -MAX_STEERING) steering = -MAX_STEERING;
    if ( (steering < 100) && (steering > -100) ) steering = 0;

    if (speed != 0) {
        // release brake
        // driver_set_brake(BRAKE_MODE_NONE);

        // calc duty with differencial
        diff = (int16_t)(steering / 2);

        // corr[0] = speed - (25 * this->speed_fb[0]);
        corr[0] = speed - (25 * this->speed_fb[1]);
        corr[1] = speed - (25 * this->speed_fb[1]);
        if (corr[0] < -1000) corr[0] = -1000;
        if (corr[1] < -1000) corr[1] = -1000;
        // corr[0] = 0;
        // corr[1] = 0;
        duty[0] = speed + corr[0];
        duty[1] = speed + corr[1];
        duty[0] += diff;
        duty[1] -= diff;

        if (duty[0] > 2000) duty[0] = 2000;
        if (duty[1] > 2000) duty[1] = 2000;

        ESP_LOGI(TAG, "cmd:%d, fdb:%d, %d, cor:%d, %d, dut:%d, %d", speed, this->speed_fb[0], this->speed_fb[1], corr[0], corr[1], duty[0], duty[1]);
        // ESP_LOGI(TAG, "cmd:%d, dut:%d, %d", speed, duty[0], duty[1]);

        // start pwm
        duty_result[0] = duty[0];
        duty_result[1] = duty[1];

        // ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, duty[0]);
        // ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, duty[1]);
        // ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
        // ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (uint32_t)(duty[0]/8));  
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, (uint32_t)(duty[1]/8));  

        // ledc_set_fade_with_time(ledc_channel[0].speed_mode, ledc_channel[0].channel, duty[0], LEDC_FADE_TIME);
        // ledc_set_fade_with_time(ledc_channel[1].speed_mode, ledc_channel[1].channel, duty[1], LEDC_FADE_TIME);
        // ledc_fade_start(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_FADE_NO_WAIT);
        // ledc_fade_start(ledc_channel[1].speed_mode, ledc_channel[1].channel, LEDC_FADE_NO_WAIT);        
        
    } else {
        // stop pwm
        duty_result[0] = 0;
        duty_result[1] = 0;
        // ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
        // ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, 0);
        // ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
        // ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);  
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0);  
        
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
            dac_output_voltage(DAC_CHANNEL_1, 192);
            dac_output_voltage(DAC_CHANNEL_2, 192);
            brake_mode = BRAKE_MODE_3;
        break;
        case BRAKE_MODE_MAX:
            dac_output_voltage(DAC_CHANNEL_1, 255);
            dac_output_voltage(DAC_CHANNEL_2, 255);
        break;
    }
    // ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
    // ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, 0);
    // ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
    // ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);  
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0);  
}

brake_mode_t driver_get_brake(void)
{
    return brake_mode;
}

void driver_emergency_brake(void)
{
    driver_set_brake(BRAKE_MODE_MAX);
    
    // ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
    // ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, 0);
    // ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
    // ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);  
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0);  
}

void IRAM_ATTR timer_group0_isr(void *para) 
{
    timer_spinlock_take(TIMER_GROUP_0);
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if (timer_intr & TIMER_INTR_T0) {
        evt.type = TEST_WITHOUT_RELOAD;
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, timer_idx, timer_counter_value);
    } else if (timer_intr & TIMER_INTR_T1) {
        evt.type = TEST_WITH_RELOAD;
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
    timer_spinlock_give(TIMER_GROUP_0);
}
static void example_tg0_timer_init(int timer_idx,
                                   bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
                       (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

void driver_test(void* arg)
{
    while (1) {
        timer_event_t evt;
        if (xQueueReceive(timer_queue, &evt, 10/portTICK_PERIOD_MS)) {
            // ESP_LOGI(TAG, "feedback: %d,\t%d",speed_feedback[0],speed_feedback[1]);
            // ESP_LOGI(TAG, "feedback= %d", this->speed_fb);
            this->speed_fb[0] = speed_feedback[0];
            this->speed_fb[1] = speed_feedback[1];
            speed_feedback[0] = 0;
            speed_feedback[1] = 0;
        }
    }
}

void driver_init(driver_t* driver)
{
    gpio_config_t io_conf;
    int ch;

    this = driver;

    // ledc_timer_config_t ledc_timer = {
    //     .duty_resolution = LEDC_TIMER_11_BIT,
    //     .freq_hz = 2000,
    //     .speed_mode = LEDC_HIGH_SPEED_MODE,
    //     .timer_num = LEDC_TIMER_2,
    //     .clk_cfg = LEDC_AUTO_CLK,
    // };
    // ledc_timer_config(&ledc_timer);

    // for (ch = 0; ch < LEDC_CH_NUM; ch++) {
    //     ledc_channel_config(&ledc_channel[ch]);
    // }
    // ledc_fade_func_install(0);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, VR1_IO);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, VR2_IO);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 4000;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);

    // set pwm 0
    // ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
    // ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, 0);
    // ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
    // ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);  
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0);  
        
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

    // set hall feedback
    pwm_queue = xQueueCreate(10, sizeof(uint32_t));
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL<<M2_IO);//(1ULL<<M1_IO) | (1ULL<<M2_IO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    // gpio_isr_handler_add(M1_IO, feedback1_isr_handler, (void*)M1_IO);
    gpio_isr_handler_add(M2_IO, feedback2_isr_handler, (void*)M2_IO);

    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    // example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
    example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD, 0.2);

    driver_set_direction(true);
    xTaskCreate(driver_test, "driver_test", 4096, (void *) 0, 10, NULL);
}