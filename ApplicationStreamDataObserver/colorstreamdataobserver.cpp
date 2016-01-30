#include "colorstreamdataobserver.h"

ColorStreamDataObserver::ColorStreamDataObserver(StreamDataReceiver &channel,
                                                 int hFOVDegree, int vFOVDegree)
    : AbstractStreamDataObserver(channel), hFOVDegree(hFOVDegree), vFOVDegree(vFOVDegree)
{
    vFOVRad = (vFOVDegree * PI) / 180.0;
    hFOVRad = (hFOVDegree * PI) / 180.0;
}

void ColorStreamDataObserver::convertFromPixelMapToCloud(const PixelMap::Ptr map, pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
    cloud.width=map->sizex;
    cloud.height=map->sizey;
    cloud.is_dense=false;
    cloud.points.resize(map->sizex*map->sizey);

    float focalLengthX = cloud.width / (2*tan(((float) hFOVRad)/2));
    float focalLengthY = cloud.height / (2*tan(((float) vFOVRad)/2));

    float constant_x = 1.0f / focalLengthX;
    float constant_y = 1.0f / focalLengthY;

    float centerX = ((float)cloud.width - 1.f) / 2.f;
    float centerY = ((float)cloud.height - 1.f) / 2.f;

    const char* data = (const char*) map->data;

    int depth_idx = 0;
    int char_idx = 0;
    for (int v = 0; v < cloud.height; ++v)
    {
        for (int u = 0; u < cloud.width; ++u, ++depth_idx, char_idx+=map->bytePerPixel)
        {
            pcl::PointXYZRGBA& pt = cloud.points[depth_idx];

            pt.z = 0.001f;
            pt.x = (static_cast<float> (u) - centerX) * constant_x;
            pt.y = (static_cast<float> (v) - centerY) * constant_y;

            pt.r = data[char_idx];
            pt.g = data[char_idx+1];
            pt.b = data[char_idx+2];
        }
    }

    cloud.sensor_origin_.setZero ();
    cloud.sensor_orientation_.w () = 1.0f;
    cloud.sensor_orientation_.x () = 0.0f;
    cloud.sensor_orientation_.y () = 0.0f;
    cloud.sensor_orientation_.z () = 0.0f;
}
