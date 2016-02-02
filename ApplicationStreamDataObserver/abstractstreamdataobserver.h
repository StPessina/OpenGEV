#ifndef ABSTRACTSTREAMDATAOBSERVER_H
#define ABSTRACTSTREAMDATAOBSERVER_H

#include <QObject>

#include "Application/streamdatareceiver.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "CommonStreamImageFormat/pixelformat.h"
#include "CommonStreamImageFormat/PixelMap.h"

class AbstractStreamDataObserver : public QObject
{
    Q_OBJECT
public:
    AbstractStreamDataObserver(StreamDataReceiver &channel);

    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptrCloud;

signals:
    void pointCloudUpdate(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

public slots:
    virtual void startReceiveStreamData() final;

    virtual void newStreamDataReceived() final;

protected:

    virtual void convertFromPixelMapToCloud(const PixelMap::Ptr map,
                                            pcl::PointCloud<pcl::PointXYZRGBA> &cloud) = 0;

    static void computeFocalParameters(int width, int height, float hFOVRad, float vFOVRad,
                                    float &focalLengthX, float &focalLengthY,
                                    float &opticalCenterX, float &opticalCenterY,
                                    float &centerX, float &centerY);

private:

    StreamDataReceiver &channel;

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;


};

#endif // ABSTRACTSTREAMDATAOBSERVER_H
