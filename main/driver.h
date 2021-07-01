#ifndef __DRIVER_H
#define __DRIVER_H

#include "main.h"

typedef enum {
    BRAKE_MODE_NONE = 0,
    BRAKE_MODE_1,
    BRAKE_MODE_2,
    BRAKE_MODE_3,
    BRAKE_MODE_MAX
} brake_mode_t;

void driver_set_direction(bool dir);

void driver_set_speed(uint16_t speed, int16_t steering);

void driver_set_brake(brake_mode_t brake);
brake_mode_t driver_get_brake(void);
void driver_emergency_brake(void);

void driver_init(void);

#endif /* __DRIVER_H */