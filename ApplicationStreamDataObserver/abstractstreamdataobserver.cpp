#include "abstractstreamdataobserver.h"

AbstractStreamDataObserver::AbstractStreamDataObserver(StreamDataReceiver &channel)
    : channel(channel), ptrCloud(&cloud)
{
    connect(&channel,SIGNAL(newStreamDataAvailable()),
            this,SLOT(newStreamDataReceived()));
    connect(&channel,SIGNAL(startGetStreamData()),
            this,SLOT(startReceiveStreamData()));
}

void AbstractStreamDataObserver::startReceiveStreamData()
{

}

void AbstractStreamDataObserver::newStreamDataReceived()
{
    const PixelMap::Ptr map = (const PixelMap::Ptr)channel.getStreamData();
    convertFromPixelMapToCloud(map, cloud);
    emit pointCloudUpdate(ptrCloud);
}
