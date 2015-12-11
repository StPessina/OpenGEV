#include "opennicamera.h"

OpenNICamera::OpenNICamera()
{
    openiniInitResult = openni::OpenNI::initialize ();
    if(openiniInitResult!=openni::STATUS_OK) {
        perror("OPENNI ERROR: " + openiniInitResult);
        return;
    }

    openiniInitResult = opennidevice.open(openni::ANY_DEVICE);
    if(openiniInitResult!=openni::STATUS_OK) {
        perror("OPENNI ERROR: " + openiniInitResult);
        return;
    }

    openiniInitResult = ir_.create (opennidevice, openni::SENSOR_DEPTH);
    if(openiniInitResult!=openni::STATUS_OK) {
        perror("OPENNI ERROR: " + openiniInitResult);
        return;
    }

    openiniInitResult = ir_.start ();
    if(openiniInitResult!=openni::STATUS_OK) {
        perror("OPENNI ERROR: " + openiniInitResult);
        return;
    }

    if(opennidevice.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
        opennidevice.setImageRegistrationMode (openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    else
        perror("OPENNI WARNING: Device can't support image registration depth to color");

    openiniInitResult = color_.create (opennidevice, openni::SENSOR_COLOR);
    if(openiniInitResult!=openni::STATUS_OK) {
        perror("OPENNI ERROR: " + openiniInitResult);
        return;
    }

    openiniInitResult = color_.start();
    if(openiniInitResult!=openni::STATUS_OK) {
        perror("OPENNI ERROR: " + openiniInitResult);
        return;
    }

    gvdevice = new GVDevice(opennidevice.getDeviceInfo().getVendor(),
                          opennidevice.getDeviceInfo().getName(),
                          opennidevice.getDeviceInfo().getUri());

    gvdevice->createStreamChannel();
    gvdevice->createStreamChannel();
    gvdevice->createStreamChannel();

    connect(this,SIGNAL(initialization()),this,SLOT(setupTimer()));

    depthMap = new PixelMap(GVSP_PIX_MONO16, 320,240,0,0,0,0);
    RGBMap = new PixelMap(GVSP_PIX_RGB8, 320,240,0,0,0,0);
    DepthRGBMap = new PixelMap(GVSP_PIX_MONO16_RGB8, 320,240,0,0,0,0);
}

OpenNICamera::~OpenNICamera()
{
    if(gvdevice!=NULL)
        delete gvdevice;
    if(depthMap!=NULL)
        depthMap->destroyPixelMap();
    if(RGBMap!=NULL)
        RGBMap->destroyPixelMap();
    if(DepthRGBMap!=NULL)
        DepthRGBMap->destroyPixelMap();
}

void OpenNICamera::shutdown(openni::Status status)
{

}

void OpenNICamera::setupTimer()
{
    QTimer* readFromCamTimer = new QTimer(this);
    connect(readFromCamTimer,SIGNAL(timeout()),this,SLOT(readDataFromCam()));
    readFromCamTimer->start(30);
}

void OpenNICamera::readDataFromCam()
{
    openni::Status rcDepth, rcColor;
    rcDepth = ir_.readFrame (&irf_);
    rcColor = color_.readFrame (&colorf_);

    if(rcDepth==openni::STATUS_OK)
        sendDepthDataStream();
    else
        perror("OPENI ERROR: During depth frame acquisition " + rcColor);

    if(rcColor==openni::STATUS_OK)
        sendRgbDataStream();
    else
        perror("OPENI ERROR: During color frame acquisition " + rcColor);

    if(rcDepth==openni::STATUS_OK &&
           rcColor ==openni::STATUS_OK)
        sendDepthRgbDataStream();
}

void OpenNICamera::sendDepthDataStream()
{
    if(!gvdevice->getStreamChannel(0)->isChannelOpen())
        return;

    depthMap->data = (char*) irf_.getData();
    depthMap->sizex = irf_.getWidth();
    depthMap->sizey = irf_.getHeight();

    gvdevice->getStreamChannel(0)->writeIncomingData(*depthMap);
}

void OpenNICamera::sendRgbDataStream()
{
    if(!gvdevice->getStreamChannel(1)->isChannelOpen())
        return;

    RGBMap->data = (char*) colorf_.getData();
    RGBMap->sizex = colorf_.getWidth();
    RGBMap->sizey = colorf_.getHeight();

    gvdevice->getStreamChannel(1)->writeIncomingData(*RGBMap);
}

void OpenNICamera::sendDepthRgbDataStream()
{
    if(!gvdevice->getStreamChannel(2)->isChannelOpen())
        return;

    char* depthData = (char*) irf_.getData();
    char* rgbData = (char*) colorf_.getData();

    int depth_idx = 0;
    int rgb_idx = 0;
    int char_idx = 0;
    for (int v = 0; v < DepthRGBMap->sizey; ++v)
    {
        for (int u = 0; u < DepthRGBMap->sizex; ++u, depth_idx+=2, rgb_idx+=3, char_idx+=5)
        {
            DepthRGBMap->data[char_idx] = depthData[depth_idx];
            DepthRGBMap->data[char_idx+1] = depthData[depth_idx+1];

            DepthRGBMap->data[char_idx+2] = rgbData[rgb_idx];
            DepthRGBMap->data[char_idx+3] = rgbData[rgb_idx+1];
            DepthRGBMap->data[char_idx+4] = rgbData[rgb_idx+2];
        }
    }

    gvdevice->getStreamChannel(2)->writeIncomingData(*DepthRGBMap);
}

void OpenNICamera::run()
{
    if(openiniInitResult!=openni::STATUS_OK) {
        perror("GVDEVICE OPENNICAMERA: Can't start because error during initialitation.");
        return;
    }

    emit initialization();

    exec();
}
