#ifndef PAYLOADTYPE_H
#define PAYLOADTYPE_H

/**
 * @brief The PayloadType enum lists possible type for stream packets format
 */
enum PayloadType {
    IMAGE = 0x0001,
    IMAGE_EXTENDED = 0x4001,

    RAW_DATA = 0x0002,
    RAW_DATA_EXTENDED = 0x4002,

    FILE_DATA = 0x0003,
    FILE_DATA_EXTENDED = 0x4003,

    CHUNK_DATA = 0x0004,
    CHUNK_DATA_EXTENDED = 0x0005,

    JPEG = 0x0006,
    JPEG_EXTENDED = 0x4006,

    JPEG_2000 = 0x0007,
    JPEG_2000_EXTENDED = 0x4007,

    H264 = 0x0008,
    H264_EXTENDED = 0x4008,

    MULTI_ZONE = 0x0009,
    MULTI_ZONE_EXTENDED = 0x4009

};

#endif // PAYLOADTYPE_H
