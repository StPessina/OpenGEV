#include "partnerdevice.h"

PartnerDevice::PartnerDevice()
{
    channelOpen=false;
}

PartnerDevice::~PartnerDevice()
{
    if(isChannelOpen())
        delete controlChannel;
}

bool PartnerDevice::openControlChannel(quint16 port)
{
    controlChannel = new ControlChannelMaster(ipAddress, port);

    controlChannel->initSocket();

    //send open channel message
    WriteRegisterCommand* writeReg = new WriteRegisterCommand(this,
                                                              REG_CONTROL_CHANNEL_PRIVILEGE,
                                                              1,
                                                              ipAddress,
                                                              CONTROL_CHANNEL_DEF_PORT);
    controlChannel->sendCommand(writeReg);
    short statusCode = writeReg->getStatusCode();
    delete writeReg;
    if(statusCode==GEV_STATUS_SUCCESS)
        channelOpen=true;
    else {
        channelOpen=false;
        delete controlChannel;
        return false;
    }

    return true;
}

void PartnerDevice::closeControlChannel()
{
    if(isChannelOpen()) { //Send message for channel unlock
        WriteRegisterCommand* writeReg = new WriteRegisterCommand(this,
                                                                  REG_CONTROL_CHANNEL_PRIVILEGE,
                                                                  0,
                                                                  ipAddress,
                                                                  CONTROL_CHANNEL_DEF_PORT);
        controlChannel->sendCommand(writeReg);
        delete writeReg;
        delete controlChannel;
    }

    channelOpen=false;
}

bool PartnerDevice::isChannelOpen()
{
    return channelOpen;
}

int PartnerDevice::getStreamingChannelNumber()
{
    if(!channelOpen)
        return -1;

    ReadRegisterCommand* readReg = new ReadRegisterCommand(this,
                                                           REG_NR_STREAM_CHANNELS,
                                                           ipAddress,
                                                           CONTROL_CHANNEL_DEF_PORT);
    controlChannel->sendCommand(readReg);
    int value = readReg->getRegisterValue();
    delete readReg;
    return value;
}

bool PartnerDevice::setControlAccessKey(int key)
{
    if(!channelOpen)
        return false;

    WriteRegisterCommand* writeReg = new WriteRegisterCommand(this,
                                                              REG_ACTION_DEVICE_KEY,
                                                              key,
                                                              ipAddress,
                                                              CONTROL_CHANNEL_DEF_PORT);
    int result = controlChannel->sendCommand(writeReg);
    delete writeReg;

    return result == GEV_STATUS_SUCCESS;
}
