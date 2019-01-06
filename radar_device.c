#include "./include/radar_device.h"

bool sensor_start() {
    int fd = open(PORT_UART, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return false;

    if (!config_serial(fd, B115200))
        return false;

    // urgly but work
    char cmd1[] = "advFrameCfg\n";
    char cmd2[] = "sensorStart\n";

    write(fd, cmd1, strlen(cmd1));
    usleep(100000);
    write(fd, cmd2, strlen(cmd2));
    
    close(fd);
    return true;
}

bool sensor_stop() {
    int fd = open(PORT_UART, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return false;

    if (!config_serial(fd, B115200))
        return false;

    char cmd[] = "sensorStop\n";
    write(fd, cmd, strlen(cmd));
    
    close(fd);
    return true;
}

bool device_open(int *fd) {
    *fd = open(PORT_DATA, O_RDONLY | O_NOCTTY | O_NDELAY);
    if (*fd == -1) return false;

    if (!config_serial(*fd, B921600)) 
        return false;
    tcflush(*fd, TCIOFLUSH);
    return true;
}


bool device_close(int fd) {
    close(fd);
}

static bool config_serial(int fd, int baudrate) {
    struct termios options;

    if (tcgetattr(fd, &options) == -1)
        return false;

    // set up raw mode / no echo / binary
    options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
    options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN); //|ECHOPRT

    options.c_oflag &= (tcflag_t) ~(OPOST);
    options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);

#ifdef IUCLC
    options.c_iflag &= (tcflag_t) ~IUCLC;
#endif
#ifdef PARMRK
    options.c_iflag &= (tcflag_t) ~PARMRK;
#endif

    // 设置波特率
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    // 8位数据位
    options.c_cflag &= (tcflag_t) ~CSIZE;
    options.c_cflag |= CS8;

    // 1位停止位
    options.c_cflag &= (tcflag_t) ~(CSTOPB);

    // 无校验
    options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
    options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);

#ifdef IXANY
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
#else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
#endif

    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSANOW, &options) == -1)
        return false;
    return true;
}