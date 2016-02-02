#include "depthstreamdataobserver.h"

DepthStreamDataObserver::DepthStreamDataObserver(StreamDataReceiver &channel,
                                                 int hFOVDegree, int vFOVDegree)
    : AbstractStreamDataObserver(channel), hFOVDegree(hFOVDegree), vFOVDegree(vFOVDegree)
{
    vFOVRad = (((float) vFOVDegree) * PI) / 180.0;
    hFOVRad = (((float) hFOVDegree) * PI) / 180.0;
}

bool DepthStreamDataObserver::setPointDepth(pcl::PointXYZRGBA &pt,
                                            int pixelFormat,
                                            const char *data,
                                            int pixelXIndex, int pixelYIndex,
                                            float centerX, float centerY,
                                            float constantX, float constantY)
{
    float depth;

    switch (pixelFormat) {
    case GVSP_PIX_MONO16:
    case GVSP_PIX_MONO16_RGB8:
        depth=(float) ((const quint16*) data)[0];
        break;
    case GVSP_PIX_MONO32:
    case GVSP_PIX_MONO32_RGB8:
        depth=((const float*) data)[0];
        break;
    default:
        return false;
        break;
    }


    float bad_point =  std::numeric_limits<float>::quiet_NaN();
    // Check for invalid measurements
    if (depth == 0)
    {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
        return false;
    }

    pt.z = depth * 0.001f;
    pt.x = (static_cast<float> (pixelXIndex) - centerX) * pt.z * constantX;
    pt.y = (static_cast<float> (pixelYIndex) - centerY) * pt.z * constantY;

    return true;
}

void DepthStreamDataObserver::convertFromPixelMapToCloud(const PixelMap::Ptr map, pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
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

            if(setPointDepth(pt, map->pixelFormat, &(map->data[char_idx]),
                             u,v,centerX, centerY, constantX, constantY)) {
                pt.r = 255;
                pt.g = 255;
                pt.b = 255;
            }
        }
    }

    cloud.sensor_origin_.setZero ();
    cloud.sensor_orientation_.w () = 1.0f;
    cloud.sensor_orientation_.x () = 0.0f;
    cloud.sensor_orientation_.y () = 0.0f;
    cloud.sensor_orientation_.z () = 0.0f;
}
