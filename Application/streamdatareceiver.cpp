#include "streamdatareceiver.h"

StreamDataReceiver::StreamDataReceiver(QHostAddress address, quint16 port)
{
#ifdef USE_QT_SOCKET
    streamReceiver = new QtUDPChannel(QHostAddress::Any, port,
                                            new StreamImageDataHandlerFactory(this));
#endif
#ifdef USE_BOOST_SOCKET
    streamReceiver = new BoostUDPChannel(QHostAddress::Any, port,
                                         new StreamImageDataHandlerFactory(this));
#endif
#ifdef USE_OSAPI_SOCKET
    streamReceiver = new OSAPIUDPChannel(QHostAddress::Any, port,
                                         new StreamImageDataHandlerFactory(this));
#endif
    streamReceiver->initSocket();
    streamReceiver->start();


    streamData = new PixelMap<Pixel<2>>::Ptr[streamDataCacheSize];

    blockId = new quint64[streamDataCacheSize];
    for (int i = 0; i < streamDataCacheSize; ++i)
        blockId[i]=-1;

    packetId = new quint32[streamDataCacheSize];
    for (int i = 0; i < streamDataCacheSize; ++i)
        packetId[i]=0;
}

StreamDataReceiver::~StreamDataReceiver()
{
    delete streamReceiver;
    for (int i = 0; i < streamDataCacheSize; ++i) {
        if(blockId[i]!=-1)
            freeStreamData(i);
    }
    delete streamData;
    delete blockId;
    delete packetId;
}

void StreamDataReceiver::openStreamData(quint64 blockId,
                                        quint32 pixelFormat, quint32 sizex, quint32 sizey,
                                        quint32 offsetx, quint32 offsety,
                                        quint16 paddingx, quint16 paddingy)
{
    emit startGetStreamData();

    clearOldCache(blockId);

    if(getStreamDataIndexFromBlockId(blockId)==-1) {
        int i = getFreeStreamData();
        this->blockId[i] = blockId;
        streamData[i] = new PixelMap<Pixel<2>>(pixelFormat, sizex, sizey,
                                           offsetx, offsety,
                                           paddingx, paddingy);
        this->packetId[i] = 1;
    }
#ifdef ENABLE_LOG4CPP
    else
        logger.warnStream()<<"Stream was already open "<<blockId;
#endif

}

/*
void StreamDataReceiver::checkNewAllocation(quint32 pixelFormat, quint32 sizex, quint32 sizey, quint32 offsetx, quint32 offsety, quint16 paddingx, quint16 paddingy)
{
    if(this->pixelFormat!=pixelFormat ||
            this->sizex != sizex ||
            this->sizey != sizey) {
        streamData->destroyPixelMap();
        delete streamData;
        streamData = new PixelMap<Pixel>(pixelFormat, sizex, sizey,
                                   offsetx, offsety,
                                   paddingx, paddingy);
    }
}
*/

bool StreamDataReceiver::checkNewPayload(quint64 blockId, quint32 packetId)
{
    int i = getStreamDataIndexFromBlockId(blockId);

    bool sequentiallyCheckResult = false;

    sequentiallyCheckResult = (i!=-1);

    if(sequentiallyCheckResult)
        sequentiallyCheckResult = (packetId==this->packetId[i]+1);

    this->packetId[i] = packetId;

    return sequentiallyCheckResult;
}

/*
void StreamDataReceiver::addStreamData(quint64 blockId, quint32 packetId,
                                       Pixel<2> pixel)
{
    int i = getStreamDataIndexFromBlockId(blockId);
    if(i!=-1)
        streamData[i]->setNextPixel(pixel);
    else
        logger.warnStream()<<"Stream was not open "<<blockId<<" "<<packetId;
}
*/
/*
void StreamDataReceiver::addStreamData(quint64 blockId, quint32 packetId,
                                       int position, Pixel pixel)
{
    streamData->setNextPixel(position, pixel);
}
*/

void StreamDataReceiver::closeStreamData(quint64 blockId, quint32 packetId)
{
    int i = getStreamDataIndexFromBlockId(blockId);
    if(i!=-1) {
        //freeStreamData(i);
        lastClosedStream=i;
        emit newStreamDataAvailable();
    }
#ifdef ENABLE_LOG4CPP
    else
        logger.warnStream()<<"Stream was not open "<<blockId<<" "<<packetId;
#endif
}

PixelMap<Pixel<2>>::Ptr StreamDataReceiver::getStreamData()
{
    return streamData[lastClosedStream];
}

PixelMap<Pixel<2>>::Ptr StreamDataReceiver::getStreamData(quint64 blockId)
{
    int i = getStreamDataIndexFromBlockId(blockId);
    if(i!=-1)
        return streamData[i];
    return NULL;
}

quint32 StreamDataReceiver::getPixelFormat()
{
    return streamData[lastClosedStream]->pixelFormat;
}

quint64 StreamDataReceiver::getBlockId()
{
    return blockId[0];
}

quint32 StreamDataReceiver::getLastPacketId()
{
    return packetId[lastClosedStream];
}

int StreamDataReceiver::getStreamDataIndexFromBlockId(quint64 blockId)
{
    for (int i = 0; i < streamDataCacheSize; ++i)
       if(this->blockId[i]==blockId)
           return i;
    return -1;
}

int StreamDataReceiver::getFreeStreamData()
{
    for (int i = 0; i < streamDataCacheSize; ++i)
        if(this->blockId[i]==-1)
            return i;
    return -1;
}

int StreamDataReceiver::freeStreamData(int index)
{
    streamData[index]->destroyPixelMap();
    delete streamData[index];
    blockId[index]=-1;
    return 0;
}

int StreamDataReceiver::clearOldCache(quint64 lastBlockId)
{
    for (int i = 0; i < streamDataCacheSize; ++i)
        if(this->blockId[i]<=lastBlockId-(streamDataCacheSize-1)
                && this->blockId[i]!=-1)
            freeStreamData(i);
    return 0;
}
