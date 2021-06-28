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

ble_spp_server_t spp;
packet_t packet;

void app_main(void)
{
    uint8_t * spp_data;

    driver_init();
    ble_spp_server_init(&spp);

    while (1) {
        // disconnect event
        if (!isConnected()) {
            driver_set_brake(BRAKE_MODE_MAX);
            driver_set_speed(0, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
        }

        // receive event
        if (xQueueReceive(spp.data_queue, &spp_data, 10/portTICK_RATE_MS)) {
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
                    driver_set_speed((uint16_t)(-1 * packet.forward), packet.clockwise);
                } else {
                    driver_set_direction(true);
                    driver_set_speed((uint16_t)packet.forward, packet.clockwise);
                }
            }            
        }
    }
}
