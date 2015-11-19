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

    void openStreamData(quint32 pixelFormat, quint32 sizex, quint32 sizey,
                        quint32 offsetx, quint32 offsety,
                        quint16 paddingx, quint16 paddingy);

    void addStreamData(AbstractPixelFormat* pixel);

    void closeStreamData();

    PixelsMap getStreamData();

signals:
    void newDataAvailable();

private:
    UdpChannelReceiver* streamReceiver;

    PixelsMap* streamData;

};

#endif // STREAMDATARECEIVER_H
