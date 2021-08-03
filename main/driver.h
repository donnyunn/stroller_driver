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

typedef enum {
    DRIVE_MODE_STOP = 0,
    DRIVE_MODE_FORWARD,
    DRIVE_MODE_BACKWARD,
} drive_mode_t;

typedef struct {
    bool direction;
    uint16_t speed;
    int16_t steering;
} driver_t;

void driver_set_brake_mode(brake_mode_t mode);
brake_mode_t driver_get_brake_mode(void);
uint16_t driver_get_speed(int num);
void driver_set_direction(bool dir);

void driver_set_speed(uint16_t speed, int16_t steering);

void driver_set_brake(brake_mode_t brake);
brake_mode_t driver_get_brake(void);
void driver_emergency_brake(void);

void driver_init(void);

#endif /* __DRIVER_H */