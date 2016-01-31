#ifndef COLORSTREAMDATAOBSERVER_H
#define COLORSTREAMDATAOBSERVER_H

#include <ApplicationStreamDataObserver/abstractstreamdataobserver.h>

#define PI 3.14159265

class ColorStreamDataObserver : public AbstractStreamDataObserver
{
    Q_OBJECT
public:
    ColorStreamDataObserver(StreamDataReceiver &channel, int hFOVDegree, int vFOVDegree);

    void setColorInformation(pcl::PointXYZRGBA& pt, const char* data);

protected:
    virtual void convertFromPixelMapToCloud(const PixelMap::Ptr map,
                                            pcl::PointCloud<pcl::PointXYZRGBA> &cloud);
private:
    int hFOVRad, vFOVRad;
    int hFOVDegree, vFOVDegree;
};

#endif // COLORSTREAMDATAOBSERVER_H
