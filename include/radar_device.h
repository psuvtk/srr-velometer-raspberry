#ifndef RADAR_DEVICE_H
#define RADAR_DEVICE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>    // 错误号定义
#include <termios.h>  // 终端定义
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#define PORT_UART "/dev/ttyACM0"
#define PORT_DATA "/dev/ttyACM1"

extern bool sensor_start();
extern bool sensor_stop();

extern bool device_open(int *fd);
extern bool device_close(int fd);

static bool config_serial(int fd, int baudrate);

#endif // RADAR_DEVICE_H
