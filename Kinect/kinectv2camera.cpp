#include "kinectv2camera.h"

KinectV2Camera::KinectV2Camera()
{
    //Discovery device
    if(freenect2.enumerateDevices() == 0) {
        perror("FREENECT ERROR: no device connected");
        return;
    }

    string serial = freenect2.getDefaultDeviceSerialNumber();

    //Open device
    dev = freenect2.openDevice(serial);
    if(dev==0) {
        perror("OPENNI ERROR: Can't open device");
        return;
    }

    //Set listener
    listener = new SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);

    gvdevice = new GVDevice("Microsoft Corp.",
                            "Kinect V2",
                            serial);
    gvdevice->configure3DCapabilities(57,44);

    gvdevice->createStreamChannel();
    gvdevice->createStreamChannel();
    gvdevice->createStreamChannel();

    connect(this,SIGNAL(initialization()),this,SLOT(setupTimer()));

    depthMap = new PixelMap(GVSP_PIX_MONO32, 512,424,0,0,0,0);
    RGBMap = new PixelMap(GVSP_PIX_RGB8, 1920,1080,0,0,0,0);
    DepthRGBMap = new PixelMap(GVSP_PIX_MONO32_RGB8, 1920, 1080,0,0,0,0);

    initOk=true;

    dev->start();
}

KinectV2Camera::~KinectV2Camera()
{
    if(gvdevice!=NULL)
        delete gvdevice;
    if(depthMap!=NULL)
        depthMap->destroyPixelMap();
    if(RGBMap!=NULL)
        RGBMap->destroyPixelMap();
    if(DepthRGBMap!=NULL)
        DepthRGBMap->destroyPixelMap();

    dev->stop();
    dev->close();

    delete dev;
    delete listener;
}

void KinectV2Camera::setupTimer()
{
    QTimer* readFromCamTimer = new QTimer(this);
    connect(readFromCamTimer,SIGNAL(timeout()),this,SLOT(readDataFromCam()));
    readFromCamTimer->start(30);
}

void KinectV2Camera::readDataFromCam()
{
    listener->waitForNewFrame(frames);

    rgbFrame = frames[libfreenect2::Frame::Color];
    irFrame = frames[libfreenect2::Frame::Ir];
    depthFrame = frames[libfreenect2::Frame::Depth];

    sendDepthDataStream();
    sendRgbDataStream();
    sendDepthRgbDataStream();

    listener->release(frames);

}

void KinectV2Camera::sendDepthDataStream()
{
    if(!gvdevice->getStreamChannel(0)->isChannelOpen())
        return;

    depthMap->data =  (char*) irFrame->data;
    depthMap->sizex = irFrame->width;
    depthMap->sizey = irFrame->height;

    gvdevice->getStreamChannel(0)->writeIncomingData(*depthMap);
}

void KinectV2Camera::sendRgbDataStream()
{
    if(!gvdevice->getStreamChannel(1)->isChannelOpen())
        return;

    char* rgbData = (char*) rgbFrame->data;
    RGBMap->sizex = rgbFrame->width;
    RGBMap->sizey = rgbFrame->height;

    //Converting data from BGR to RGB
    int rgb_idx = 0;
    int char_idx = 0;
    for (int v = 0; v < RGBMap->sizey; ++v)
    {
        for (int u = 0; u < RGBMap->sizex; ++u, rgb_idx+=4, char_idx+=3)
        {
            RGBMap->data[char_idx] = rgbData[rgb_idx+2];
            RGBMap->data[char_idx+1] = rgbData[rgb_idx+1];
            RGBMap->data[char_idx+2] = rgbData[rgb_idx];
        }
    }

    gvdevice->getStreamChannel(1)->writeIncomingData(*RGBMap);
}

void KinectV2Camera::sendDepthRgbDataStream()
{
    if(!gvdevice->getStreamChannel(2)->isChannelOpen())
        return;

    char* depthData =  (char*) depthFrame->data;
    char* rgbData =  (char*) rgbFrame->data;

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

void KinectV2Camera::run()
{
    if(!initOk) {
        perror("GVDEVICE KINECT V2 CAMERA: Can't start because error during initialitation.");
        return;
    }

    emit initialization();

    exec();
}
