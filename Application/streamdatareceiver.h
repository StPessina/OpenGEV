#ifndef STREAMDATARECEIVER_H
#define STREAMDATARECEIVER_H

#include <QObject>
#include <iostream>

#include "CommonComponent/gvcomponent.h"

#ifdef USE_QT_SOCKET
    #include "CommonUdpChannel/qtudpchannel.h"
#endif
#ifdef USE_BOOST_SOCKET
    #include "CommonUdpChannel/boostudpchannel.h"
#endif
#ifdef USE_OSAPI_SOCKET
    #include "CommonUdpChannel/osapiudpchannel.h"
#endif

#include "CommonStreamImageFormat/PixelMap.h"

#include "ApplicationStreamDataHandler/streamimagedatahandlerfactory.h"
#include "ApplicationCommand/packetresendcommand.h"

/**
 * @brief The StreamDataReceiver class create UDP channel to receive
 * stream data from a stream channel on the application. When a new
 * stream packet is reveived methods to open / add and save data is called
 * directly from stream packet handler. This class implements also a cache
 * to store the last stream frames (see streamDataCacheSize member).
 */
class StreamDataReceiver : public QObject, GVComponent
{
    Q_OBJECT
public:

    /**
     * @brief StreamDataReceiver explict constructor for QObject
     * @param parent
     */
    explicit StreamDataReceiver(QObject* parent = 0);

    /**
     * @brief StreamDataReceiver constructor
     * @param address where the UDP channel is bind
     * @param port where the UDP channel is bind
     * @param channelId
     * @param requestRetrasmissionChannel is the channel that this stream channel
     * use to request packet retrasmission
     */
    StreamDataReceiver(QHostAddress address, quint16 port,
                       quint16 channelId, UDPChannel &requestRetrasmissionChannel);

    /**
     * @brief ~StreamDataReceiver deconstructor
     */
    virtual ~StreamDataReceiver();

    /**
     * @brief openStreamData open a new PixelMap on the cache to store incoming stream data.
     * if the block id is already open, this method do nothing. If the block id is not
     * exist this method try to reuse an old PixelMap in the cache (no reallocation). If a
     * old map is not available it will allocate a new map. A PixelMap can be reused if
     * it store the same pixelFormat and sizex and sizey. Normally the stream data on a stream
     * channel has the same format for each trasmission, this will be reduce reallocation of pixel map.
     * This method will fire startGetStreamData() qt signals.
     * @param blockId
     * @param pixelFormat
     * @param sizex
     * @param sizey
     * @param offsetx
     * @param offsety
     * @param paddingx
     * @param paddingy
     */
    virtual void openStreamData(quint64 blockId,
                        quint32 pixelFormat, quint32 sizex, quint32 sizey,
                        quint32 offsetx, quint32 offsety,
                        quint16 paddingx, quint16 paddingy);

    /**
     * @brief blockIdExist check if a PixelMap is stored in the cache with a particular blockId
     * @param blockId
     * @return true if PixelMap exists
     */
    virtual bool blockIdExist(quint64 blockId);

    /**
     * @brief checkPacketIdSequence check if a new packet id respect sequence. If not a retrasmission
     * request will be fired on the requestRetrasmissionChannel.
     * @param blockId
     * @param packetId
     * @return true if the packet id respect sequence
     */
    virtual bool checkPacketIdSequence(quint64 blockId, quint32 packetId);

    /**
     * @brief addData copy the stream data in a PixelMap
     * @param blockId
     * @param packetId
     * @param data stream data
     * @param dataLenght
     * @return true if data is copied
     */
    virtual bool addData(quint64 blockId, quint32 packetId, const char* data, quint32 dataLenght);

    /**
     * @brief closeStreamData close PixelMap and send newStreamDataAvailable() qt signal
     * @param blockId
     * @param packetId
     */
    virtual void closeStreamData(quint64 blockId, quint32 packetId);

    /**
     * @brief getStreamData
     * @return last received PixelMap
     */
    virtual PixelMap::Ptr getStreamData();

    /**
     * @brief getStreamData
     * @param blockId
     * @return stream data by blockId
     */
    virtual PixelMap::Ptr getStreamData(quint64 blockId);

    /**
     * @brief getPixelFormat
     * @return  last received PixelMap pixelFormat
     */
    virtual quint32 getPixelFormat();

    /**
     * @brief getBlockId
     * @return last received PixelMap blockId
     */
    virtual quint64 getBlockId();

    /**
     * @brief getLastPacketId
     * @return last received PixelMap PacketId
     */
    virtual quint32 getLastPacketId();

signals:
    /**
     * @brief startGetStreamData is emitted when new stream leader packet
     * is received
     */
    void startGetStreamData();

    /**
     * @brief newStreamDataAvailable is emitted when stream trailer packet
     * is reveived
     */
    void newStreamDataAvailable();

protected:
    /**
     * @brief checkPacketIdSequence check if a new packet id respect sequence. If not a retrasmission
     * request will be fired on the requestRetrasmissionChannel. This method use cacheIndex without
     * search blockId in cache.
     * @param cacheIndex
     * @param blockId
     * @param packetId
     * @return true if the packet id respect sequence
     */
    virtual bool checkPacketIdSequence(int cacheIndex, quint64 blockId, quint32 packetId);

private:

    /**
     * @brief channelId
     */
    quint16 channelId;

    /**
     * @brief streamReceiver udp channel where stream packet is received
     */
    UDPChannel *streamReceiver;
    QThread communicationThread;

    /**
     * @brief requestRetrasmissionChannel is udp channel used for resend message
     */
    UDPChannel &requestRetrasmissionChannel;

    /**
     * @brief streamData reference to PixelMap cache
     */
    PixelMap::Ptr* streamData;

    /**
     * @brief blockId reference to blockId cache
     */
    quint64* blockId;

    /**
     * @brief packetId reference to last packetId cache (used for sequence control)
     */
    quint32* packetId;

    /**
     * @brief lastDataWriteIndex reference to last write data in pixelmap (used for sequential memcopy)
     */
    quint32* lastDataWriteIndex;

    /**
     * @brief streamDataCacheSize cache size
     */
    int streamDataCacheSize = 30;

#ifdef ENABLE_LOG4CPP
    log4cpp::Category &logger = log4cpp::Category::getInstance("StreamReceiverLog");
#endif

    /**
     * @brief lastClosedStream index to last closed data stream
     */
    int lastClosedStream = 0;

    /**
     * @brief getStreamDataIndexFromBlockId cache block id search function
     * @param blockId
     * @return cache index or -1 if blockId is not found
     */
    int getStreamDataIndexFromBlockId(quint64 blockId);

    /**
     * @brief getFreeStreamData
     * @return cache index to free PixelMap
     */
    int getFreeStreamData();

    /**
     * @brief freeStreamData
     * @param index
     * @return 0 if successful
     */
    int freeStreamData(int index);

    /**
     * @brief clearOldCache clear old PixelMap if blockId is too old.
     * @param lastBlockId
     * @return 0 if cache is cleaned
     */
    int clearOldCache(quint64 lastBlockId);
};

#endif // STREAMDATARECEIVER_H
