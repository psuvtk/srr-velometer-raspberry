#include "./include/radar_packet.h"

void packet_parse(packet_t *packet, char *raw_packet, int length) {
    construct_header(packet, raw_packet);
    char *tl = raw_packet + HEAD_STRUCT_SIZE_BYTES;

    if ((uint32_t)length != packet->header.totalPacketLen) {
        packet->isvalid = false;
        return;
    }

    for (uint32_t i = 0; i < packet->header.numTLVs; i++) {
        uint32_t type = getTlvType(tl);
        uint32_t len = getTlvLength(tl);
        tl += TL_STRUCT_SIZE_BYTES;

        switch (type) {
        case MMWDEMO_UART_MSG_DETECTED_POINTS:
            constructDetObj(packet, tl);
            tl += len;
            break;
        case MMWDEMO_UART_MSG_CLUSTERS:
            constructCluster(packet, tl);
            tl += len;
            break;
        case MMWDEMO_UART_MSG_TRACKED_OBJ:
            constructTracker(packet, tl);
            tl += len;
            break;
        case MMWDEMO_UART_MSG_PARKING_ASSIST:
            constructParkingAssisBin(packet, tl);
            tl += len;
            break;
        case MMWDEMO_UART_MSG_STATS:
            constructStatsInfo(packet, tl);
            tl += STATS_SIZE_BYTES;
            break;
        default:
            packet->isvalid = false;
            // 出错直接返回跳过该帧
            return;
        }
        packet->isvalid = true;
    }
}
void packet_init(packet_t *packet) {
    packet->detObj = NULL;
    packet->cluster = NULL;
    packet->trackers = NULL;
    packet->parkingAssitBins = NULL;
}
void packet_clear(packet_t *packet) {
    if (packet->detObj != NULL) {
        free(packet->detObj);
        packet->detObj = NULL;
    }
    if (packet->cluster != NULL) {
        free(packet->cluster);
        packet->cluster = NULL;
    }
    if (packet->trackers != NULL) {
        free(packet->trackers);
        packet->trackers = NULL;
    }
    if (packet->parkingAssitBins != NULL) {
        free(packet->parkingAssitBins);
        packet->parkingAssitBins = NULL;
    }
}

static void construct_header(packet_t *packet, char *raw_packet) {
    packet->header.frameNumber = ((const __header_t *)raw_packet)->frameNumber;
    packet->header.numDetectedObj = ((const __header_t *)raw_packet)->numDetectedObj;
    packet->header.numTLVs = ((const __header_t *)raw_packet)->numTLVs;
    packet->header.platform = ((const __header_t *)raw_packet)->platform;
    packet->header.subFrameNumber = ((const __header_t *)raw_packet)->subFrameNumber;
    packet->header.timeCpuCycles = ((const __header_t *)raw_packet)->timeCpuCycles;
    packet->header.totalPacketLen = ((const __header_t *)raw_packet)->totalPacketLen;
    packet->header.version = ((const __header_t *)raw_packet)->version;
}


static uint32_t getTlvType(const char *tl) {
    return ((const __tl_t *)(tl))->type;
}

static uint32_t getTlvLength(const char *tl) {
    return ((const __tl_t *)(tl))->length;
}

static uint16_t getDescrNumObj(const char *descr) {
    return ((const __dataObjDescr_t *)descr)->numObj;
}

static uint16_t getDescrQFormat(const char *descr) {
    return ((const __dataObjDescr_t *)descr)->xyzQFormat;
}


static void constructDetObj(packet_t *packet, const char *tl) {
    uint16_t numObjs = getDescrNumObj(tl);
    uint16_t xyzQFormat = getDescrQFormat(tl);
    double invQFormat = 1.0 / (1 << xyzQFormat);
    tl += DESCR_STRUCT_SIZE_BYTES;

    packet->nDetObj = numObjs;
    packet->detObj = (DetObj_t *)malloc(numObjs * sizeof(DetObj_t));

    const __detObj_t *rawDetObj = (const __detObj_t *)(tl);

    for (uint16_t j = 0; j < numObjs; j++, rawDetObj++) {
        (packet->detObj)[j].x = rawDetObj->x * invQFormat;
        (packet->detObj)[j].y = rawDetObj->y * invQFormat;
        (packet->detObj)[j].peakVal = rawDetObj->peakVal;
        (packet->detObj)[j].doppler = rawDetObj->speed * invQFormat;
        (packet->detObj)[j].range = sqrt((packet->detObj)[j].x * (packet->detObj)[j].x + (packet->detObj)[j].y * (packet->detObj)[j].y);
    }
}

static void constructCluster(packet_t *packet, const char *tl) {
    uint16_t numObjs = getDescrNumObj(tl);
    uint16_t xyzQFormat = getDescrQFormat(tl);
    double invQFormat = 1.0 / (1 << xyzQFormat);
    tl += DESCR_STRUCT_SIZE_BYTES;

    packet->nCluster = numObjs;
    packet->cluster = (Cluster_t*)malloc(numObjs * sizeof(Cluster_t));

    const __cluster_t *rawClusterObj = (const __cluster_t*)(tl);
    for (uint16_t i = 0; i < numObjs; i++, rawClusterObj++) {
        (packet->cluster)[i].xCenter = rawClusterObj->x * invQFormat;
        (packet->cluster)[i].yCenter = rawClusterObj->y * invQFormat;
        (packet->cluster)[i].xSize = rawClusterObj->x_size * invQFormat;
        (packet->cluster)[i].ySize = rawClusterObj->y_size * invQFormat;
    }
}

static void constructTracker(packet_t *packet, const char *tl) {
    uint16_t numObjs = getDescrNumObj(tl);
    uint16_t xyzQFormat = getDescrQFormat(tl);
    double invQFormat = 1.0 / (1 << xyzQFormat);
    tl += DESCR_STRUCT_SIZE_BYTES;

    packet->nTracker = numObjs;
    packet->trackers = (Tracker_t*)malloc(numObjs * sizeof(Tracker_t));

    const __tracker_t *rawTrackerObj = (const __tracker_t *)(tl);
    for (uint16_t i = 0; i < numObjs; i++, rawTrackerObj++) {

        (packet->trackers)[i].x = rawTrackerObj->x * invQFormat;
        (packet->trackers)[i].y = rawTrackerObj->y * invQFormat;
        (packet->trackers)[i].vx = rawTrackerObj->xd * invQFormat;
        (packet->trackers)[i].vy = rawTrackerObj->yd * invQFormat;

        (packet->trackers)[i].xSize = rawTrackerObj->xSize * invQFormat;
        (packet->trackers)[i].ySize = rawTrackerObj->ySize * invQFormat;

        (packet->trackers)[i].range = sqrt((packet->trackers)[i].x * (packet->trackers)[i].x +(packet->trackers)[i].y * (packet->trackers)[i].y);
        (packet->trackers)[i].doppler = ((packet->trackers)[i].x * (packet->trackers)[i].vx + (packet->trackers)[i].y*(packet->trackers)[i].vy) / (packet->trackers)[i].range;
    }
}

static void constructParkingAssisBin(packet_t *packet, const char *tl) {
    // not impl
}

static void constructStatsInfo(packet_t *packet, const char *tl) {
    const __statsInfo_t *rawStatsInfo = (const __statsInfo_t *)(tl);

    packet->statsInfo.interFrameProcessingTime = rawStatsInfo->interFrameProcessingTime;
    packet->statsInfo.transmitOutputTime = rawStatsInfo->transmitOutputTime;
    packet->statsInfo.interFrameProcessingMargin = rawStatsInfo->interFrameProcessingMargin;
    packet->statsInfo.interChirpProcessingMargin = rawStatsInfo->interChirpProcessingMargin;
    packet->statsInfo.activeFrameCpuLoad = rawStatsInfo->activeFrameCpuLoad;
    packet->statsInfo.interFrameCpuLoad = rawStatsInfo->interFrameCpuLoad;
}