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
    emit pointCloudUpdate();
}

void AbstractStreamDataObserver::computeFocalParameters(int width, int height, float hFOVRad, float vFOVRad,
                                                        float &focalLengthX, float &focalLengthY,
                                                        float &opticalCenterX, float &opticalCenterY,
                                                        float &centerX, float &centerY)
{
    focalLengthX = width / (2*tan(hFOVRad/2));
    focalLengthY = height / (2*tan(vFOVRad/2));

    opticalCenterX = 1.0f / focalLengthX;
    opticalCenterY = 1.0f / focalLengthY;

    centerX = ((float) width - 1.f) / 2.f;
    centerY = ((float) height - 1.f) / 2.f;
}
