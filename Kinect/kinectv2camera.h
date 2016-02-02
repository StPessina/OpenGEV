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
#include <libfreenect2/registration.h>
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

    PixelMap::Ptr colorMap;

    PixelMap::Ptr DepthColorMap;

    bool initOk;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;

    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::FrameMap frames;

    libfreenect2::Frame *colorFrame;
    libfreenect2::Frame *irFrame;
    libfreenect2::Frame *depthFrame;

    libfreenect2::Registration* registration;
    libfreenect2::Frame* undistorted;
    libfreenect2::Frame* registered;
};

#endif // KINECTV2CAMERA_H
