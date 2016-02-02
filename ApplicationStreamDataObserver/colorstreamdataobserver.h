#ifndef COLORSTREAMDATAOBSERVER_H
#define COLORSTREAMDATAOBSERVER_H

#include <ApplicationStreamDataObserver/abstractstreamdataobserver.h>

#define PI 3.14159265

class ColorStreamDataObserver : public AbstractStreamDataObserver
{
    Q_OBJECT
public:
    ColorStreamDataObserver(StreamDataReceiver &channel, int hFOVDegree, int vFOVDegree);

    static void setPointColor(pcl::PointXYZRGBA& pt, int pixelFormat, const char* data);

protected:
    virtual void convertFromPixelMapToCloud(const PixelMap::Ptr map,
                                            pcl::PointCloud<pcl::PointXYZRGBA> &cloud);
private:
    float hFOVRad, vFOVRad;
    int hFOVDegree, vFOVDegree;
};

#endif // COLORSTREAMDATAOBSERVER_H
