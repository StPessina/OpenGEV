#ifndef STREAMDATARECEIVER_H
#define STREAMDATARECEIVER_H

#include <QObject>

#include "CommonComponent/gvcomponent.h"

#include "CommonUdpChannel/udpchannelreceiver.h"

#include "CommonStreamImageFormat/abstractpixelformat.h"
#include "CommonStreamImageFormat/pixelsmap.h"

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

    virtual bool checkNewPayload(quint64 blockId, quint32 packetId);

    virtual void addStreamData(quint64 blockId,
                               AbstractPixelFormat *pixel);

    virtual void closeStreamData(quint64 blockId, quint32 packetId);

    virtual PixelsMap getStreamData();

    virtual quint32 getPixelFormat();

    virtual quint64 getBlockId();

    virtual quint32 getLastPacketId();

signals:
    void newDataAvailable();

private:
    UdpChannelReceiver* streamReceiver;

    PixelsMap* streamData;

    quint64 blockId;

    quint32 lastPacketId;

};

#endif // STREAMDATARECEIVER_H
