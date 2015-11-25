#ifndef STREAMDATARECEIVER_H
#define STREAMDATARECEIVER_H

#include <QObject>

#include "CommonComponent/gvcomponent.h"

#include "CommonUdpChannel/udpchannelreceiver.h"

#include "CommonStreamImageFormat/PixelMap.h"

#include "ApplicationStreamDataHandler/streamimagedatahandlerfactory.h"

class StreamDataReceiver : public QObject, GVComponent
{
    Q_OBJECT
public:

    /**
     * @brief StreamDataReceiver explict constructor for QObject
     * @param parent
     */
    explicit StreamDataReceiver(QObject* parent = 0);

    StreamDataReceiver(QHostAddress address, quint16 port);

    virtual ~StreamDataReceiver();

    virtual void openStreamData(quint64 blockId,
                        quint32 pixelFormat, quint32 sizex, quint32 sizey,
                        quint32 offsetx, quint32 offsety,
                        quint16 paddingx, quint16 paddingy);

    virtual void checkNewAllocation(quint32 pixelFormat, quint32 sizex, quint32 sizey,
                               quint32 offsetx, quint32 offsety,
                               quint16 paddingx, quint16 paddingy);

    virtual bool checkNewPayload(quint64 blockId, quint32 packetId);

    virtual void addStreamData(quint64 blockId, quint32 packetId,
                               Pixel pixel);

    /*
    virtual void addStreamData(quint64 blockId, quint32 packetId,
                               int position, Pixel *pixel);*/

    virtual void closeStreamData(quint64 blockId, quint32 packetId);

    virtual PixelMap<Pixel>::Ptr getStreamData();

    virtual quint32 getPixelFormat();

    virtual quint64 getBlockId();

    virtual quint32 getLastPacketId();

signals:
    void newDataAvailable();

private:
    UdpChannelReceiver* streamReceiver;

    PixelMap<Pixel>::Ptr streamData;

    quint64 blockId;

    quint32 lastPacketId;

    bool blockOpen;

    log4cpp::Category &logger = log4cpp::Category::getInstance("StreamReceiverLog");

    quint32 pixelFormat, sizex, sizey;
    quint32 offsetx, offsety;
    quint16 paddingx, paddingy;

    int lastPixelId = 0;

};

#endif // STREAMDATARECEIVER_H
