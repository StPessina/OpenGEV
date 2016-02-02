#include "depthcolorstreamdataobserver.h"

DepthColorStreamDataObserver::DepthColorStreamDataObserver(StreamDataReceiver &channel,
                                                 int hFOVDegree, int vFOVDegree)
    : AbstractStreamDataObserver(channel), hFOVDegree(hFOVDegree), vFOVDegree(vFOVDegree)
{
    vFOVRad = (vFOVDegree * PI) / 180.0;
    hFOVRad = (hFOVDegree * PI) / 180.0;
}

void DepthColorStreamDataObserver::convertFromPixelMapToCloud(const PixelMap::Ptr map, pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
    cloud.width=map->sizex;
    cloud.height=map->sizey;
    cloud.is_dense=false;
    cloud.points.resize(map->sizex*map->sizey);

    cloud.width=map->sizex;
    cloud.height=map->sizey;
    cloud.is_dense=false;
    cloud.points.resize(map->sizex*map->sizey);

    float focalLengthX, focalLengthY, constantX, constantY,
            centerX, centerY;

    computeFocalParameters(cloud.width, cloud.height, hFOVRad, vFOVRad,
                           focalLengthX, focalLengthY,
                           constantX, constantY,
                           centerX, centerY);

    int char_idx = 0;
    int depth_idx = 0;
    for (int v = 0; v < cloud.height; ++v)
    {
        for (int u = 0; u < cloud.width; ++u, ++depth_idx, char_idx+=map->bytePerPixel)
        {
            pcl::PointXYZRGBA& pt = cloud.points[depth_idx];

            if(DepthStreamDataObserver::setPointDepth(pt, map->pixelFormat, &(map->data[char_idx]),
                             u,v, centerX, centerY, constantX, constantY)) {
                ColorStreamDataObserver::setPointColor(pt, map->pixelFormat, &(map->data[char_idx]));
            }
        }
    }

    cloud.sensor_origin_.setZero ();
    cloud.sensor_orientation_.w () = 1.0f;
    cloud.sensor_orientation_.x () = 0.0f;
    cloud.sensor_orientation_.y () = 0.0f;
    cloud.sensor_orientation_.z () = 0.0f;
}
