#ifndef OPENNICAMERA_H
#define OPENNICAMERA_H

#include <QThread>

#include <string>

#include <opengv.h>

#include <Application/gvapplication.h>

#include <Device/gvdevice.h>
#include <DeviceCommandHandler/devicecommandhandlerfactory.h>

#include "iostream"

#include <stdio.h>

#include <OpenNI.h>

class OpenNICamera : public QThread
{
    Q_OBJECT
public:
    OpenNICamera();

    ~OpenNICamera();

    void shutdown(openni::Status status);

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


    openni::Status openiniInitResult;

    openni::Device opennidevice;
    openni::VideoStream ir_;
    openni::VideoStream color_;
    openni::Status rc_;
    openni::VideoFrameRef irf_;
    openni::VideoFrameRef colorf_;

};

#endif // OPENNICAMERA_H
