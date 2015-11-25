#include "streamdatareceiver.h"

StreamDataReceiver::StreamDataReceiver(QHostAddress address, quint16 port)
{
    streamReceiver = new UdpChannelReceiver(QHostAddress::Any, port,
                                            new StreamImageDataHandlerFactory(this), false);
    streamReceiver->initSocket();

    streamData = new PixelMap<Pixel>(0, 0, 0, 0, 0, 0, 0);
}

StreamDataReceiver::~StreamDataReceiver()
{
    delete streamReceiver;
    streamData->destroyPixelMap();
}

void StreamDataReceiver::openStreamData(quint64 blockId,
                                        quint32 pixelFormat, quint32 sizex, quint32 sizey,
                                        quint32 offsetx, quint32 offsety,
                                        quint16 paddingx, quint16 paddingy)
{
    checkNewAllocation(pixelFormat, sizex, sizey,
                       offsetx, offsety,
                       paddingx, paddingy);

    this->blockId = blockId;
    lastPacketId=1;
    lastPixelId=0;
    if(blockOpen)
        logger.warnStream()<<"Block already open";
    blockOpen = true;
}

void StreamDataReceiver::checkNewAllocation(quint32 pixelFormat, quint32 sizex, quint32 sizey, quint32 offsetx, quint32 offsety, quint16 paddingx, quint16 paddingy)
{
    if(this->pixelFormat!=pixelFormat ||
            this->sizex != sizex ||
            this->sizey != sizey ||
            this->offsetx != offsetx ||
            this->offsety != offsety ||
            this->paddingx != paddingx ||
            this->paddingy != paddingy) {
        streamData->destroyPixelMap();
        delete streamData;
        streamData = new PixelMap<Pixel>(pixelFormat, sizex, sizey,
                                   offsetx, offsety,
                                   paddingx, paddingy);
    }
}

bool StreamDataReceiver::checkNewPayload(quint64 blockId, quint32 packetId)
{
    if(packetId!=lastPacketId+1)
        logger.warnStream()<<"Data loss "<<this->blockId<<" "<<(lastPacketId+1); //TODO: fire require retrasmission

    lastPacketId = packetId;
    if(blockId!=this->blockId) {
        logger.warnStream()<<"Data loss "<<this->blockId<<" "<<packetId;
        return false;
    }
    return true;
}

void StreamDataReceiver::addStreamData(quint64 blockId, quint32 packetId,
                                       Pixel pixel)
{
    streamData->setNextPixel(pixel);
}

/*
void StreamDataReceiver::addStreamData(quint64 blockId, quint32 packetId,
                                       int position, Pixel pixel)
{
    streamData->setNextPixel(position, pixel);
}
*/

void StreamDataReceiver::closeStreamData(quint64 blockId, quint32 packetId)
{
    logger.infoStream()<<"Close";
    if(this->blockId==blockId) {
        this->lastPacketId=packetId;
        emit newDataAvailable();
    } else {
        logger.warnStream()<<"Data loss "<<this->blockId;
    }

    if(!blockOpen)
        logger.warnStream()<<"Block was close";
    blockOpen=false;
}

PixelMap<Pixel>::Ptr StreamDataReceiver::getStreamData()
{
    return streamData;
}

quint32 StreamDataReceiver::getPixelFormat()
{
    return streamData->pixelFormat;
}

quint64 StreamDataReceiver::getBlockId()
{
    return blockId;
}

quint32 StreamDataReceiver::getLastPacketId()
{
    return lastPacketId;
}
