/*
   This code is written by Rotom Ltd. but it is in the Public Domain 
   (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This file is a main code for the controller device.
*
****************************************************************************/

#include "main.h"

#define TAG "rotom app"

typedef enum {
    WORK_INIT = 0,
    WORK_ADVERTISING,
    WORK_CONNECTED,
} main_work_t;
main_work_t work = WORK_INIT;

ble_spp_server_t spp;
packet_t packet;
driver_t driver = {
    .direction = true,
    .speed = 0,
    .steering = 0,
};

void app_main(void)
{
    uint8_t * spp_data;
    uint16_t temp;
    drive_mode_t drive_mode = DRIVE_MODE_STOP;

    while (1) {
        switch (work) {
            case WORK_INIT:
                driver_init();
                battery_init();
                ledbar_init();
                button_init();
                ble_spp_server_init(&spp);
                work = WORK_ADVERTISING;
            break;
            case WORK_ADVERTISING:
                if (isConnected()) {
                    work = WORK_CONNECTED;
                } else {
                    vTaskDelay(100/portTICK_PERIOD_MS);
                }
            break;
            case WORK_CONNECTED:
                // receive event
                if (xQueueReceive(spp.data_queue, &spp_data, 500/portTICK_RATE_MS)) {
                    // spp_debug((char *)spp_data, 10);
                    pakcet_decoding(&packet, spp_data, 10);
                    free(spp_data);

                    if (packet.ledbar != 0) {
                        ledbar_toggle();
                    }

                    if (packet.brake != 0) {
                        // emergency brake
                        driver_set_brake(BRAKE_MODE_MAX);
                        drive_mode = DRIVE_MODE_STOP;
                    } else if (battery_check() == false) {
                        driver_emergency_brake();
                        ESP_LOGI(TAG, "Low Battery Power!");
                        ledbar_blink_start(1);
                        drive_mode = DRIVE_MODE_STOP;
                    } else {
                        switch (drive_mode) {
                            case DRIVE_MODE_STOP:
                                // if ((driver_get_speed(0) != 0) || (driver_get_speed(1) != 0)) {
                                if (true) {
                                    if (packet.forward < 0) {
                                        driver_set_brake(BRAKE_MODE_NONE);
                                        driver_set_direction(false);
                                        drive_mode = DRIVE_MODE_BACKWARD;
                                    } else if (packet.forward > 0) {
                                        driver_set_brake(BRAKE_MODE_NONE);
                                        drive_mode = DRIVE_MODE_FORWARD;
                                        driver_set_direction(true);
                                    } else {
                                        driver_set_speed(0, 0);
                                        driver_set_direction(true);
                                    }
                                } else {
                                    driver_set_speed(0, 0);
                                    driver_set_direction(true);
                                }
                            break;
                            case DRIVE_MODE_FORWARD:
                                if (packet.forward < 0) packet.forward = 1;
                                if (packet.forward != 0) {
                                    if (packet.forward > 0) {
                                        driver_set_brake(BRAKE_MODE_NONE);
                                        temp = abs(packet.forward);
                                        driver.speed = (7*driver.speed + temp)>>3;
                                        // driver.speed = temp;
                                        driver.steering = packet.clockwise;
                                        driver_set_speed(driver.speed, driver.steering);
                                    }
                                } else {
                                    driver_set_brake(driver_get_brake_mode());
                                    driver.speed = (3*driver.speed)>>2;
                                    // driver.speed = 0;
                                    driver.steering = packet.clockwise;
                                    driver_set_speed(driver.speed, driver.steering);
                                    // driver_set_speed(0, 0);
                                    if (driver.speed == 0)
                                        drive_mode = DRIVE_MODE_STOP;
                                }
                            break;
                            case DRIVE_MODE_BACKWARD:
                                if (packet.forward > 0) packet.forward = -1;
                                if (packet.forward != 0) {
                                    if (packet.forward < 0) {
                                        driver_set_brake(BRAKE_MODE_NONE);
                                        temp = abs(packet.forward);
                                        driver.speed = (7*driver.speed + temp)>>3;
                                        // driver.speed = temp;
                                        driver.steering = packet.clockwise;
                                        driver_set_speed(driver.speed, driver.steering);
                                    }
                                } else {
                                    driver_set_brake(driver_get_brake_mode());
                                    driver.speed = (3*driver.speed)>>2;
                                    // driver.speed = 0;
                                    driver.steering = packet.clockwise;
                                    driver_set_speed(driver.speed, driver.steering);
                                    // driver_set_speed(0, 0);
                                    if (driver.speed == 0)
                                        drive_mode = DRIVE_MODE_STOP;
                                }
                            break;
                        }
                    }       
                } else {
                    driver_emergency_brake();
                    drive_mode = DRIVE_MODE_STOP;
                    if (!isConnected()) {
                        work = WORK_ADVERTISING;
                    }
                }
            break;
        }

        if (isButtonPressed()) {
            brake_mode_t brake = driver_get_brake_mode();
            switch (brake) {
                case BRAKE_MODE_NONE:
                    driver_set_brake_mode(BRAKE_MODE_1);
                    ledbar_blink_start(1);
                break;
                case BRAKE_MODE_1:
                    driver_set_brake_mode(BRAKE_MODE_2);
                    ledbar_blink_start(2);
                break;
                case BRAKE_MODE_2:
                    driver_set_brake_mode(BRAKE_MODE_3);
                    ledbar_blink_start(3);
                break;
                case BRAKE_MODE_3:
                    driver_set_brake_mode(BRAKE_MODE_1);
                    ledbar_blink_start(1);
                break;
                case BRAKE_MODE_MAX:
                    driver_set_brake_mode(BRAKE_MODE_1);
                    ledbar_blink_start(1);
                break;
            } 
            ESP_LOGI(TAG, "Brake Mode = %d", driver_get_brake_mode());
            vTaskDelay(500/portTICK_PERIOD_MS);
            releaseButton();
        }
    }
}
