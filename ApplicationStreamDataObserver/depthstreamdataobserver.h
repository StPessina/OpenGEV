#ifndef DEPTHSTREAMDATAOBSERVER_H
#define DEPTHSTREAMDATAOBSERVER_H

#define PI 3.14159265

#include <math.h>

#include <ApplicationStreamDataObserver/abstractstreamdataobserver.h>

class DepthStreamDataObserver : public AbstractStreamDataObserver
{
    Q_OBJECT
public:
    DepthStreamDataObserver(StreamDataReceiver &channel, int hFOVDegree, int vFOVDegree);

protected:
    virtual void convertFromPixelMapToCloud(const PixelMap::Ptr map,
                                            pcl::PointCloud<pcl::PointXYZRGBA> &cloud);
private:
    int hFOVRad, vFOVRad;
    int hFOVDegree, vFOVDegree;
};

#endif // DEPTHSTREAMDATAOBSERVER_H
