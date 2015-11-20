#include "streamdatareceiver.h"

StreamDataReceiver::StreamDataReceiver(QHostAddress address, quint16 port)
{
    streamReceiver = new UdpChannelReceiver(address, port,
                                            new StreamImageDataHandlerFactory(this));
    streamReceiver->initSocket();

    streamData = new PixelsMap(0,0,0,0,0,0,0);
}

StreamDataReceiver::~StreamDataReceiver()
{
    delete streamReceiver;
    delete streamData;
}

void StreamDataReceiver::openStreamData(quint64 blockId,
                                        quint32 pixelFormat, quint32 sizex, quint32 sizey,
                                        quint32 offsetx, quint32 offsety,
                                        quint16 paddingx, quint16 paddingy)
{
    streamData = new PixelsMap(pixelFormat, sizex, sizey,
                               offsetx, offsety,
                               paddingx, paddingy);
    this->blockId = blockId;
    lastPacketId=1;
}

bool StreamDataReceiver::checkNewPayload(quint64 blockId, quint32 packetId)
{
    lastPacketId = packetId; //TODO: fire require retrasmission
    if(blockId!=this->blockId)
        return false;
    return true;
}

void StreamDataReceiver::addStreamData(quint64 blockId,
                                       AbstractPixelFormat *pixel)
{
    if(streamData!=NULL && blockId==this->blockId)
        streamData->addPixel(pixel);
}

void StreamDataReceiver::closeStreamData(quint64 blockId, quint32 packetId)
{
    if(this->blockId==blockId) {
        this->lastPacketId=packetId;
        emit newDataAvailable();
    }
}

PixelsMap StreamDataReceiver::getStreamData()
{
    return * streamData;
}

quint32 StreamDataReceiver::getPixelFormat()
{
    return streamData->getPixelFormat();
}

quint64 StreamDataReceiver::getBlockId()
{
    return blockId;
}

quint32 StreamDataReceiver::getLastPacketId()
{
    return lastPacketId;
}
