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

    float focalLengthX = cloud.width / (2*tan(((float) hFOVRad)/2));
    float focalLengthY = cloud.height / (2*tan(((float) vFOVRad)/2));

    float constant_x = 1.0f / focalLengthX;
    float constant_y = 1.0f / focalLengthY;


    float centerX = ((float)cloud.width - 1.f) / 2.f;
    float centerY = ((float)cloud.height - 1.f) / 2.f;

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    int depth_idx = 0;
    for (int v = 0; v < cloud.height; ++v)
    {
        for (int u = 0; u < cloud.width; ++u, ++depth_idx)
        {
            float depth;
            if(map->bytePerPixel==2)
                depth=(float) ((const quint16*) map->data)[depth_idx];
            else
                depth=((const float*) map->data)[depth_idx];
            pcl::PointXYZRGBA& pt = cloud.points[depth_idx];
            // Check for invalid measurements
            if (depth == 0)
            {
                // not valid
                pt.x = pt.y = pt.z = bad_point;
                continue;
            }

            pt.z = depth * 0.001f;
            pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
            pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
            pt.r = 255;
            pt.g = 255;
            pt.b = 255;
        }
    }

    cloud.sensor_origin_.setZero ();
    cloud.sensor_orientation_.w () = 1.0f;
    cloud.sensor_orientation_.x () = 0.0f;
    cloud.sensor_orientation_.y () = 0.0f;
    cloud.sensor_orientation_.z () = 0.0f;
}
