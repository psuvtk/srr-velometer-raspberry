#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

#include "./include/radar_device.h"
#include "./include/radar_packet.h"
#include "./include/bytearray.h"
#include "./include/display.h"

#define RECVBUF_LEN 1024
#define SYNC_LEN 8

#define ERROR_LOG(a) fprintf(stderr, a)
#define INFO_LOG(a) \
    do{             \
        fprintf(stdout, a); \
        fflush(stdout); \
    }while(0)

void timer_init();
void timer_handler(int signum);

void deal_with_packet(packet_t *packet);   // not impl
void deal_with_packet_maxvel(packet_t *packet); // 取最大速度

/**
 * 空闲状态变量
 * 0 非空闲（显示）状态
 * 1 空闲状态
 */
int g_idle_watchdog = 1;

int main (int argc, char *argv[]) {
    display_init();
    timer_init();

    /* 启动雷达 */
    if (sensor_start()) {
        INFO_LOG("sensor start success.\n");
    } else {
        ERROR_LOG("sensor start failed.\n");
        goto clear_and_exit;
    }

    /* 打开数据串口准备读取 */
    int fd;
    if (device_open(&fd)) {
        INFO_LOG("[+] device open success.\n");
    } else {
        ERROR_LOG("device open failed.");
        goto clear_and_exit;
    }

    packet_t packet;
    packet_init(&packet);
    bytearray_t framebuf;
    bytearray_init(&framebuf);

    char recvbuf[RECVBUF_LEN];
    const char sync[] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
    /**
     * 将串口数据追加到缓冲(bytearray)，然后：
     * 1. 如果缓冲区以sync code开始，并存在另一个sync code，则存在完整一帧数据，进行解析；
     * 2. 如果缓冲区以sync code开始，且不存在另一个sync code，继续读取；
     * 3. 如果缓冲区不以sync code开始，且不存在另一个sync code，则清空缓冲区；
     * 4. 如果缓冲区不以sync code开始，并存在sync code，则将sync code之前的数据删除。
     */
    while (1) {
        int num_recv = read(fd, recvbuf, RECVBUF_LEN);
        if (num_recv == 0) continue;

        bytearray_append(&framebuf, recvbuf, num_recv);
        if (bytearray_startswith(&framebuf, sync, SYNC_LEN)) {
            int next = bytearray_find(&framebuf, sync, 8, 8);
            if (next != -1) {
                /* 得到完整一帧，进行处理 */
                packet_parse(&packet, framebuf.data, next);
                
                if (packet.isvalid) {
                    deal_with_packet_maxvel(&packet);
                } else {
                    ERROR_LOG("Broken packet\n");
                }
                /* 移除处理完成的帧数据 */
                packet_clear(&packet);
                bytearray_lremove(&framebuf, next);
            } else {
                continue;
            }
        } else {
            int len = bytearray_find(&framebuf, sync, 8, 0);
            if (len == -1)
                bytearray_clear(&framebuf);
            else
                bytearray_lremove(&framebuf, len);
        }
    }

clear_and_exit:
    ERROR_LOG("clear and exit...\n");
    device_close(fd);
    bytearray_destroy(&framebuf);
    return 0;
}

/**
 * 定时器初始化函数
 * TODO: 计时不准确，如何准确计时？
 * Tips: 实际延时 **大约15倍** 于设置值
 */
void timer_init() {
    struct sigaction sa;
    struct itimerval timer;

    memset(&sa, 0, sizeof (sa));
    sa.sa_handler = &timer_handler;
    sigaction(SIGVTALRM, &sa, NULL);

    /* 起始间隔 */
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = 100000;
    /* 定时间隔 */
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 200000;

    setitimer(ITIMER_VIRTUAL, &timer, NULL);
    INFO_LOG("timer start\n");
}

/**
 * 定时器中断处理函数
 * 若指定时间间隔内没有喂狗（将g_idle_watchdog变量置零）
 * 则为空闲状态，数码管熄灭状态（省电）
 */
void timer_handler(int signum) {
    if (g_idle_watchdog == 1)
        display_write_idle();
    g_idle_watchdog = 1;
}

void deal_with_packet(packet_t *packet) {
    if (!packet->isvalid || packet->trackers == NULL) return;

    /**
     * 如果使用qsort会导致 segment fault, 具体原因未知
     * 反正数据量很小, 手动排序一下
     */
    {
        Tracker_t tmp;
        for (int i = 0; i < packet->nTracker-1; i++) {
            for (int j = i+1; j < packet->nTracker; j++) {
                if ((packet->trackers)[i].range  > (packet->trackers)[i].range) {
                    memcpy(&tmp, &(packet->trackers)[i], sizeof(Tracker_t));
                    memcpy(&(packet->trackers)[i], &(packet->trackers)[j], sizeof(Tracker_t));
                    memcpy(&(packet->trackers)[j], &tmp, sizeof(Tracker_t));
                }
            }
        }
    }

    // notimplement
}

/**
 * 选取追踪目标速度最大的目标进行积累
 * 积累帧数及时间： 25帧 * 33ms = 825ms 
 * TODO: 调优算法
 * TODO: 限定范围
 */
void deal_with_packet_maxvel(packet_t *packet) {
    const double THREASHHOLD_SPEED = 2.0;
    static int frameid_since = 0;  // 起始id
    static int acc_counter = 0;
    static double acc_velosity = 0.0;

    /* 包损坏或者无追踪目标则立即返回 */
    if (!packet->isvalid || packet->nTracker == 0) return;

    /* 选取卡尔曼滤波速度最大的目标进行累积平均 */
    double maxvel = 0.0;
    for (int i = 0; i < packet->nTracker; i++) {
        Tracker_t *ct = &(packet->trackers)[i];
        double cv = sqrt(ct->vx * ct->vx + ct->vy * ct->vy) * 3.6;
        if (cv > maxvel) maxvel = cv;
    }
    
    /**
     * 仅累积非零值
     * TODO: 是否需要修改为仅累积过门限的值，如何选取阈值？
     */
    if (maxvel > 0.0) {
        acc_counter++;
        acc_velosity += maxvel;
    }

    /* 累积平均速度 */
    double meanvel = acc_velosity / acc_counter;
    INFO_LOG("[%d]frame number: %d\tmean velosity: %lf\n", packet->header.timeCpuCycles, packet->header.frameNumber, meanvel);
    
    /* 若帧间隔25帧(约825ms)且累积平均速度过门限，则进行显示 */
    if (packet->header.frameNumber - frameid_since > 25 && meanvel > THREASHHOLD_SPEED) {
        display_write_speed(meanvel, false);
        INFO_LOG("\n\t[DISPLAY]mean velosity: %lf\n\n", meanvel);

        frameid_since = packet->header.frameNumber;
        g_idle_watchdog = 0; // 喂狗
        acc_counter = 0;
        acc_velosity = 0.0;
    }
}