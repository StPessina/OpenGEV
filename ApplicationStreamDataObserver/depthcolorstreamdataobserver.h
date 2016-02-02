#ifndef DEPTHCOLORSTREAMDATAOBSERVER_H
#define DEPTHCOLORSTREAMDATAOBSERVER_H

#define PI 3.14159265

#include <math.h>

#include <ApplicationStreamDataObserver/abstractstreamdataobserver.h>
#include <ApplicationStreamDataObserver/depthstreamdataobserver.h>
#include <ApplicationStreamDataObserver/colorstreamdataobserver.h>

class DepthColorStreamDataObserver : public AbstractStreamDataObserver
{
    Q_OBJECT
public:
    DepthColorStreamDataObserver(StreamDataReceiver &channel, int hFOVDegree, int vFOVDegree);

protected:
    virtual void convertFromPixelMapToCloud(const PixelMap::Ptr map,
                                            pcl::PointCloud<pcl::PointXYZRGBA> &cloud);
private:
    float hFOVRad, vFOVRad;
    int hFOVDegree, vFOVDegree;
};

#endif // DEPTHCOLORSTREAMDATAOBSERVER_H
