#ifndef KINECTV2CAMERA_H
#define KINECTV2CAMERA_H

#include <QThread>

#include <string>

#include <opengev.h>

#include <Application/gvapplication.h>

#include <Device/gvdevice.h>
#include <DeviceCommandHandler/devicecommandhandlerfactory.h>

#include <fstream>
#include "iostream"

#include <stdio.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/frame_listener_impl.h>

using namespace libfreenect2;

class KinectV2Camera : public QThread
{
    Q_OBJECT
public:
    KinectV2Camera();

    ~KinectV2Camera();

protected:
    void run();

signals:
    void initialization();

public slots:
    void setupTimer();

    void readDataFromCam();

    void sendDepthDataStream();

    void sendRgbDataStream();

    void sendDepthRgbDataStream();

private:
    GVDevice* gvdevice;

    PixelMap::Ptr depthMap;

    PixelMap::Ptr RGBMap;

    PixelMap::Ptr DepthRGBMap;

    bool initOk;

    Freenect2 freenect2;
    Freenect2Device *dev = 0;

    SyncMultiFrameListener *listener;
    FrameMap frames;

    Frame *rgbFrame;
    Frame *irFrame;
    Frame *depthFrame;
};

#endif // KINECTV2CAMERA_H
