#ifndef RADAR_PACKET_H
#define RADAR_PACKET_H

#include <stdbool.h>
#include <inttypes.h>
#include <malloc.h>
#include <math.h>

static const uint32_t HEAD_STRUCT_SIZE_BYTES = 40;
static const uint32_t TL_STRUCT_SIZE_BYTES = 8;
static const uint32_t DESCR_STRUCT_SIZE_BYTES = 4;

static const uint32_t OBJ_STRUCT_SIZE_BYTES = 8;
static const uint32_t CLUSTER_STRUCT_SIZE_BYTES = 8;
static const uint32_t TRACKER_STRUCT_SIZE_BYTES = 12;
static const uint32_t PARKING_ASSIST_BIN_SIZE_BYTES = 2;
static const uint32_t STATS_SIZE_BYTES = 16;

enum __TLV_Type {
    MMWDEMO_UART_MSG_DETECTED_POINTS    = 1,     // 目标
    MMWDEMO_UART_MSG_CLUSTERS           = 2,     // 聚类
    MMWDEMO_UART_MSG_TRACKED_OBJ        = 3,     // 跟踪
    MMWDEMO_UART_MSG_PARKING_ASSIST     = 4,     // 停车辅助
    MMWDEMO_UART_MSG_STATS              = 6,     // 状态信息
};

typedef struct {
    double x;
    double y;
    double doppler;
    double peakVal;
    double range;
} DetObj_t;

typedef struct {
    double xCenter;
    double yCenter;
    double xSize;
    double ySize;
} Cluster_t;

typedef struct {
    double x;
    double y;
    double vx;
    double vy;
    double xSize;
    double ySize;
    double range;
    double doppler;
} Tracker_t;

typedef struct {
    double x1;
    double x2;
    double y1;
    double y2;
} ParkingAssistBin_t;

typedef struct {
    uint32_t interFrameProcessingTime;
    uint32_t transmitOutputTime;
    uint32_t interFrameProcessingMargin;
    uint32_t interChirpProcessingMargin;
    uint32_t activeFrameCpuLoad;
    uint32_t interFrameCpuLoad;
} StatsInfo_t;


typedef struct {
    uint16_t magicWord[4];
    uint32_t version;
    uint32_t totalPacketLen;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;
} __header_t;

typedef struct {
    uint32_t type;          // 类型
    uint32_t length;        // 长度
} __tl_t;

typedef struct {
    int16_t  speed;        /*!< @brief Doppler index */
    uint16_t peakVal;     /*!< @brief Peak value */
    int16_t  x;             /*!< @brief x - coordinate in meters. */
    int16_t  y;             /*!< @brief y - coordinate in meters.  */
} __detObj_t;

typedef struct {
    int16_t x;               /**< the clustering center on x direction */
    int16_t y;               /**< the clustering center on y direction */
    int16_t x_size;                 /**< the clustering size on x direction */
    int16_t y_size;                 /**< the clustering size on y direction */
} __cluster_t;

typedef struct {
    int16_t x;                  /**< the tracking output -> x co-ordinate */
    int16_t y;                  /**< the tracking output -> y co-ordinate */
    int16_t xd;                 /**< velocity in the x direction */
    int16_t yd;                 /**< velocity in the y direction */
    int16_t xSize;              /**< cluster size (x direction). */
    int16_t ySize;              /**< cluster size (y direction). */
} __tracker_t;

typedef struct {
    uint16_t range;
} __parkingAssistBin_t;

typedef struct {
    uint32_t interFrameProcessingTime;
    uint32_t transmitOutputTime;
    uint32_t interFrameProcessingMargin;
    uint32_t interChirpProcessingMargin;
    uint32_t activeFrameCpuLoad;
    uint32_t interFrameCpuLoad;
} __statsInfo_t;

typedef struct {
    uint16_t numObj;
    uint16_t xyzQFormat;
} __dataObjDescr_t;


typedef struct {
    __header_t header;
    bool isvalid;
    int nDetObj;
    DetObj_t *detObj;
    int nCluster;
    Cluster_t *cluster;
    int nTracker;
    Tracker_t *trackers;
    int nParkingAssistBins;
    ParkingAssistBin_t *parkingAssitBins;
    StatsInfo_t statsInfo;
} packet_t;

/* 接口 */
extern void packet_init(packet_t *packet);
extern void packet_parse(packet_t *packet, char *raw, int len);
extern void packet_clear(packet_t *packet);


/* 内部实现 */
static void construct_header(packet_t *packet, char *raw_packet);

static uint32_t getTlvType(const char *tl);
static uint32_t getTlvLength(const char *tl);
static uint16_t getDescrNumObj(const char *descr);
static uint16_t getDescrQFormat(const char *descr);

static void constructDetObj(packet_t *packet, const char *tl);
static void constructCluster(packet_t *packet, const char *tl);
static void constructTracker(packet_t *packet, const char *tl);
static void constructParkingAssisBin(packet_t *packet, const char *tl);
static void constructStatsInfo(packet_t *packet, const char *tl);

#endif // RADAR_PACKET_H