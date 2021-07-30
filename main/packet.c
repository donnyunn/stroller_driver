#include "packet.h"

#define TAG "packet"

void pakcet_decoding(packet_t * packet, uint8_t * spp_data, uint16_t len)
{
    memcpy((uint8_t *)packet, spp_data, len);
    // packet->forward /= 551;
    // packet->clockwise /= 551;
    packet->forward = (uint16_t)(packet->forward / 10) * 10;
    packet->clockwise = (uint16_t)(packet->clockwise / 100) * 100;
    ESP_LOGI(TAG, "%d %d %4d %4d", packet->brake, packet->ledbar, packet->forward, packet->clockwise);
}
