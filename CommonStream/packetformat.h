#ifndef PACKETFORMAT_H
#define PACKETFORMAT_H

/**
 * @brief The PacketFormat enum lists possible types for stream packets.
 */
enum PacketFormat {
    DATA_LEADER_FORMAT = 1,
    DATA_TRAILER_FORMAT = 2,
    DATA_PAYLOAD_GENIRIC_FORMAT = 3,
    DATA_ALLIN_FORMAT = 4,
    DATA_PAYLOAD_H264_FORMAT = 5,
    DATA_PAYLOAD_MULTI_ZONE_FORMAT = 6
};

#endif // PACKETFORMAT_H
