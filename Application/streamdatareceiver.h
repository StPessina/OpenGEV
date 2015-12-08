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

class StreamDataReceiver : public QObject, GVComponent
{
    Q_OBJECT
public:

    /**
     * @brief StreamDataReceiver explict constructor for QObject
     * @param parent
     */
    explicit StreamDataReceiver(QObject* parent = 0);

    StreamDataReceiver(QHostAddress address, quint16 port, quint16 channelId, UDPChannel &requestRetrasmissionChannel);

    virtual ~StreamDataReceiver();

    virtual void openStreamData(quint64 blockId,
                        quint32 pixelFormat, quint32 sizex, quint32 sizey,
                        quint32 offsetx, quint32 offsety,
                        quint16 paddingx, quint16 paddingy);

    virtual bool blockIdExist(quint64 blockId);

    virtual bool checkPacketIdSequence(quint64 blockId, quint32 packetId);

    virtual bool addData(quint64 blockId, quint32 packetId, const char* data, quint32 dataLenght);

    virtual void closeStreamData(quint64 blockId, quint32 packetId);

    virtual PixelMap<Pixel<2>>::Ptr getStreamData();

    virtual PixelMap<Pixel<2>>::Ptr getStreamData(quint64 blockId);

    virtual quint32 getPixelFormat();

    virtual quint64 getBlockId();

    virtual quint32 getLastPacketId();

signals:
    void startGetStreamData();
    void newStreamDataAvailable();

protected:

    virtual bool checkPacketIdSequence(int cacheIndex, quint64 blockId, quint32 packetId);

private:

    quint16 channelId;

    UDPChannel *streamReceiver;

    UDPChannel &requestRetrasmissionChannel;

    PixelMap<Pixel<2>>::Ptr* streamData;
    quint64* blockId;
    quint32* packetId;
    quint32* lastDataWriteIndex;

    int streamDataCacheSize = 15;

    bool blockOpen;

#ifdef ENABLE_LOG4CPP
    log4cpp::Category &logger = log4cpp::Category::getInstance("StreamReceiverLog");
#endif

    quint32 pixelFormat, sizex, sizey;
    quint32 offsetx, offsety;
    quint16 paddingx, paddingy;

    int lastPixelId = 0;

    int missedPacket = 0;

    int lastClosedStream = 0;

    int getStreamDataIndexFromBlockId(quint64 blockId);

    int getFreeStreamData();

    int freeStreamData(int index);

    int clearOldCache(quint64 lastBlockId);
};

#endif // STREAMDATARECEIVER_H
