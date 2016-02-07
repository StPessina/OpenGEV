#include "streamdatareceiver.h"

StreamDataReceiver::StreamDataReceiver(QHostAddress address,
                                       quint16 port,
                                       quint16 channelId,
                                       UDPChannel &requestRetrasmissionChannel)
    : requestRetrasmissionChannel(requestRetrasmissionChannel),
      channelId(channelId)
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

    //This allow signals/slots separation between stream receiver
    //and frame observers class, registered to signal newStreamDataAvailable
#ifdef USE_QT_SOCKET
    this->moveToThread(streamReceiver);
#else
    this->moveToThread(&communicationThread);
    communicationThread.start();
#endif
    streamReceiver->start();

    streamDataCacheMap = new CacheMapEntry[streamDataCacheSize];
}

StreamDataReceiver::~StreamDataReceiver()
{
    for (int i = 0; i < streamDataCacheSize; ++i) {
        if(streamDataCacheMap[i].blockId!=-1) {
            freeStreamData(i);
            if(streamDataCacheMap[i].exists) {
                streamDataCacheMap[i].map->destroyPixelMap();
                delete streamDataCacheMap[i].map;
            }
        }
    }

#ifndef USE_QT_SOCKET
    communicationThread.quit();
    communicationThread.wait();
#endif

    streamReceiver->quit();
    streamReceiver->wait();

    delete streamReceiver;
    delete streamDataCacheMap;
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
        streamDataCacheMap[i].blockId = blockId;

        if(!streamDataCacheMap[i].exists) {
            streamDataCacheMap[i].map = new PixelMap(pixelFormat, sizex, sizey,
                                           offsetx, offsety,
                                           paddingx, paddingy);
            streamDataCacheMap[i].exists = true;
        } else
            streamDataCacheMap[i].map->renew(pixelFormat, sizex, sizey,
                                 offsetx, offsety,
                                 paddingx, paddingy);
        streamDataCacheMap[i].packetId = 1;
    }
#ifdef ENABLE_LOG4CPP
    else
        logger.warnStream()<<"Stream was already open "<<blockId;
#endif

}

bool StreamDataReceiver::blockIdExist(quint64 blockId)
{
    int i = getStreamDataIndexFromBlockId(blockId);

    bool sequentiallyCheckResult = false;

    sequentiallyCheckResult = (i!=-1);

    return sequentiallyCheckResult;
}

bool StreamDataReceiver::checkPacketIdSequence(quint64 blockId, quint32 packetId)
{
    int cacheIndex = getStreamDataIndexFromBlockId(blockId);

    if(cacheIndex==-1)
        return false;

    return checkPacketIdSequence(cacheIndex, blockId, packetId);
}

bool StreamDataReceiver::addData(quint64 blockId, quint32 packetId, const char *data, quint32 dataLenght)
{
    int cacheIndex = getStreamDataIndexFromBlockId(blockId);

    if(cacheIndex==-1)
        return false;

    checkPacketIdSequence(cacheIndex, blockId, packetId);

    memcpy(&(streamDataCacheMap[cacheIndex].map
             ->data[streamDataCacheMap[cacheIndex].lastDataWriteIndex]),
           data, dataLenght*sizeof(char));

    streamDataCacheMap[cacheIndex].lastDataWriteIndex+=dataLenght;

    return true;
}

bool StreamDataReceiver::checkPacketIdSequence(int cacheIndex, quint64 blockId, quint32 packetId)
{
    if(streamDataCacheMap[cacheIndex].packetId>packetId) //Accept previous as resend packets
        return true;

    bool sequentiallyCheckResult = (packetId==streamDataCacheMap[cacheIndex].packetId+1);

    if(!sequentiallyCheckResult) {
        PacketResendCommand resend (this,
                                    requestRetrasmissionChannel.getStandardDestinationAddress(),
                                    requestRetrasmissionChannel.getStandardDestinationPort(),
                                    channelId,
                                    blockId,
                                    streamDataCacheMap[cacheIndex].packetId+1, packetId-1);
        requestRetrasmissionChannel.sendPacket(resend);
    }

    streamDataCacheMap[cacheIndex].packetId = packetId;

    return sequentiallyCheckResult;
}

void StreamDataReceiver::closeStreamData(quint64 blockId, quint32 packetId)
{
    int i = getStreamDataIndexFromBlockId(blockId);
    if(i!=-1) {
        lastClosedStream=i;
        emit newStreamDataAvailable();
    }
#ifdef ENABLE_LOG4CPP
    else
        logger.warnStream()<<"Stream was not open "<<blockId<<" "<<packetId;
#endif
}

PixelMap::Ptr StreamDataReceiver::getStreamData()
{
    return streamDataCacheMap[lastClosedStream].map;
}

PixelMap::Ptr StreamDataReceiver::getStreamData(quint64 blockId)
{
    int i = getStreamDataIndexFromBlockId(blockId);
    if(i!=-1)
        return streamDataCacheMap[i].map;
    return NULL;
}

quint32 StreamDataReceiver::getPixelFormat()
{
    return streamDataCacheMap[lastClosedStream].map->pixelFormat;
}

quint64 StreamDataReceiver::getBlockId()
{
    return streamDataCacheMap[0].blockId;
}

quint32 StreamDataReceiver::getLastPacketId()
{
    return streamDataCacheMap[lastClosedStream].packetId;
}

int StreamDataReceiver::getStreamDataIndexFromBlockId(quint64 blockId)
{
    for (int i = 0; i < streamDataCacheSize; ++i)
       if(streamDataCacheMap[i].blockId==blockId)
           return i;
    return -1;
}

int StreamDataReceiver::getFreeStreamData()
{
    for (int i = 0; i < streamDataCacheSize; ++i)
        if(streamDataCacheMap[i].blockId==-1)
            return i;
    return -1;
}

int StreamDataReceiver::freeStreamData(int index)
{
    streamDataCacheMap[index].blockId=-1;
    streamDataCacheMap[index].lastDataWriteIndex=0;
    streamDataCacheMap[index].packetId=0;
    if(!streamDataCacheMap[index].exists)
        streamDataCacheMap[index].map = NULL;
    return 0;
}

int StreamDataReceiver::clearOldCache(quint64 lastBlockId)
{    
    for (int i = 0; i < streamDataCacheSize; ++i)
        if(streamDataCacheMap[i].blockId<=lastBlockId-(streamDataCacheSize-1)
                && streamDataCacheMap[i].blockId!=-1)
            freeStreamData(i);
    return 0;
}
