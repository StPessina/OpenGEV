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
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);

    gvdevice = new GVDevice("Microsoft Corp.",
                            "Kinect V2",
                            serial);
    gvdevice->configure3DCapabilities(70,60);

    gvdevice->createStreamChannel();
    gvdevice->createStreamChannel();
    gvdevice->createStreamChannel();

    connect(this,SIGNAL(initialization()),this,SLOT(setupTimer()));

    depthMap = new PixelMap(GVSP_PIX_MONO32, 512,424,0,0,0,0);
    colorMap = new PixelMap(GVSP_PIX_BGRA8, 1920,1080,0,0,0,0);
    DepthColorMap = new PixelMap(GVSP_PIX_MONO16_RGB8, 512, 424,0,0,0,0);

    initOk=true;

    dev->start();

    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    undistorted = new libfreenect2::Frame(512,424,4);
    registered = new libfreenect2::Frame(512,424,4);
}

KinectV2Camera::~KinectV2Camera()
{
    if(gvdevice!=NULL)
        delete gvdevice;
    if(depthMap!=NULL)
        depthMap->destroyPixelMap();
    if(colorMap!=NULL)
        colorMap->destroyPixelMap();
    if(DepthColorMap!=NULL)
        DepthColorMap->destroyPixelMap();

    dev->stop();
    dev->close();

    delete dev;
    delete listener;
    delete registration;
    delete undistorted;
    delete registered;
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

    colorFrame = frames[libfreenect2::Frame::Color];
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

    depthMap->data =  (char*) depthFrame->data;
    depthMap->sizex = depthFrame->width;
    depthMap->sizey = depthFrame->height;

    gvdevice->getStreamChannel(0)->writeIncomingData(*depthMap);
}

void KinectV2Camera::sendRgbDataStream()
{
    if(!gvdevice->getStreamChannel(1)->isChannelOpen())
        return;

    colorMap->sizex = colorFrame->width;
    colorMap->sizey = colorFrame->height;

    colorMap->data = (char*) colorFrame->data;;

    gvdevice->getStreamChannel(1)->writeIncomingData(*colorMap);
}

void KinectV2Camera::sendDepthRgbDataStream()
{
    if(!gvdevice->getStreamChannel(2)->isChannelOpen())
        return;

    registration->apply(colorFrame, depthFrame, undistorted, registered);

    char* depthColorData =  (char*) registered->data;

    int reg_idx = 0;
    int char_idx = 0;
    for (int v = 0; v < DepthColorMap->sizey; ++v)
    {
        for (int u = 0; u < DepthColorMap->sizex; ++u, char_idx+=5, reg_idx+=4)
        {
            //Space 8 bit
            DepthColorMap->data[char_idx] = 0;
            DepthColorMap->data[char_idx+1] = depthColorData[reg_idx+3];

            //BGR to RBG conversion
            DepthColorMap->data[char_idx+2] = depthColorData[reg_idx+2];
            DepthColorMap->data[char_idx+3] = depthColorData[reg_idx+1];
            DepthColorMap->data[char_idx+4] = depthColorData[reg_idx];
        }
    }

    gvdevice->getStreamChannel(2)->writeIncomingData(*DepthColorMap);
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
