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

    static bool setPointDepth(pcl::PointXYZRGBA &pt, int pixelFormat, const char *data,
                              int pixelXIndex, int pixelYIndex,
                              float centerX, float centerY,
                              float constantX, float constantY);

protected:
    virtual void convertFromPixelMapToCloud(const PixelMap::Ptr map,
                                            pcl::PointCloud<pcl::PointXYZRGBA> &cloud);
private:
    float hFOVRad, vFOVRad;
    int hFOVDegree, vFOVDegree;
};

#endif // DEPTHSTREAMDATAOBSERVER_H
