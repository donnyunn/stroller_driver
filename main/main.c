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
    .speed = 0,
};

void app_main(void)
{
    uint8_t * spp_data;
    uint16_t temp;

    while (1) {
        switch (work) {
            case WORK_INIT:
                driver_init();
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

                    if (packet.brake != 0) {
                        // emergency brake
                        driver_set_brake(BRAKE_MODE_MAX);
                    } else {
                        // forward or backward
                        if (packet.forward < 0) {
                            driver_set_direction(false);
                            temp = (uint16_t)(-1 * packet.forward);
                            if (temp > driver.speed) {
                                driver.speed = (15*driver.speed + temp)>>4;
                            } else {
                                // driver.speed = temp;
                                driver.speed = (15*driver.speed + temp)>>4;
                            }
                            driver_set_speed(driver.speed, packet.clockwise);
                            // driver_set_speed((uint16_t)(-1 * packet.forward), packet.clockwise);
                        } else {
                            driver_set_direction(true);
                            temp = packet.forward;
                            if (temp > driver.speed) {
                                driver.speed = (15*driver.speed + temp)>>4;
                            } else {
                                // driver.speed = temp;
                                driver.speed = (15*driver.speed + temp)>>4;
                            }
                            driver_set_speed(driver.speed, packet.clockwise);
                            // driver_set_speed((uint16_t)packet.forward, packet.clockwise);
                        }
                    }            
                } else {
                    driver_emergency_brake();
                    if (!isConnected()) {
                        work = WORK_CONNECTED;
                    }
                }
            break;
        }
    }
}
