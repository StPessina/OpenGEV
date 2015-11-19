#include "streamdatareceiver.h"

StreamDataReceiver::StreamDataReceiver(QHostAddress address, quint16 port)
{
    streamReceiver = new UdpChannelReceiver(address, port,
                                            new StreamImageDataHandlerFactory(this));
    streamReceiver->initSocket();
    streamData = NULL;
}

StreamDataReceiver::~StreamDataReceiver()
{
    delete streamReceiver;

    if(streamData!=NULL)
        delete streamData;
}

void StreamDataReceiver::openStreamData(quint32 pixelFormat, quint32 sizex, quint32 sizey,
                                        quint32 offsetx, quint32 offsety,
                                        quint16 paddingx, quint16 paddingy)
{
    streamData = new PixelsMap(pixelFormat, sizex, sizey,
                               offsetx, offsety,
                               paddingx, paddingy);
}

void StreamDataReceiver::addStreamData(AbstractPixelFormat *pixel)
{
    if(streamData!=NULL)
        streamData->addPixel(pixel);
}

void StreamDataReceiver::closeStreamData()
{
    emit newDataAvailable();
}

PixelsMap StreamDataReceiver::getStreamData()
{
    return * streamData;
}
