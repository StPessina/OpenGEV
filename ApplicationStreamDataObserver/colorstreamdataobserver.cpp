#include "colorstreamdataobserver.h"

ColorStreamDataObserver::ColorStreamDataObserver(StreamDataReceiver &channel,
                                                 int hFOVDegree, int vFOVDegree)
    : AbstractStreamDataObserver(channel), hFOVDegree(hFOVDegree), vFOVDegree(vFOVDegree)
{
    vFOVRad = (((float) vFOVDegree) * PI) / 180.0;
    hFOVRad = (((float) hFOVDegree) * PI) / 180.0;
}

void ColorStreamDataObserver::setPointColor(pcl::PointXYZRGBA &pt, int pixelFormat, const char *data)
{
    switch (pixelFormat) {
    case GVSP_PIX_RGB8:
        pt.r = data[0];
        pt.g = data[1];
        pt.b = data[2];
        break;
    case GVSP_PIX_BGR8:
        pt.b = data[0];
        pt.g = data[1];
        pt.r = data[2];
        break;
    case GVSP_PIX_BGRA8:
        pt.b = data[0];
        pt.g = data[1];
        pt.r = data[2];
        pt.a = data[3];
        break;
    case GVSP_PIX_MONO16_RGB8:
        pt.r = data[2];
        pt.g = data[3];
        pt.b = data[4];
        break;
    case GVSP_PIX_MONO32_RGB8:
        pt.r = data[4];
        pt.g = data[5];
        pt.b = data[6];
        break;
    default:
        break;
    }
}

void ColorStreamDataObserver::convertFromPixelMapToCloud(const PixelMap::Ptr map, pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
    cloud.width=map->sizex;
    cloud.height=map->sizey;
    cloud.is_dense=false;
    cloud.points.resize(map->sizex*map->sizey);

    float focalLengthX, focalLengthY, constant_x, constant_y,
            centerX, centerY;

    computeFocalParameters(cloud.width, cloud.height, hFOVRad, vFOVRad,
                           focalLengthX, focalLengthY,
                           constant_x, constant_y,
                           centerX, centerY);

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

            setPointColor(pt, map->pixelFormat, &data[char_idx]);
        }
    }

    cloud.sensor_origin_.setZero ();
    cloud.sensor_orientation_.w () = 1.0f;
    cloud.sensor_orientation_.x () = 0.0f;
    cloud.sensor_orientation_.y () = 0.0f;
    cloud.sensor_orientation_.z () = 0.0f;
}
