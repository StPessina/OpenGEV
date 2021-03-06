﻿#include "discoverycommandhandler.h"

DiscoveryCommandHandler::DiscoveryCommandHandler(GVComponent *target, const QByteArray &receivedDatagram,
                                                 QHostAddress senderAddress, quint16 senderPort)
    : AbstractCommandHandler(target, DISCOVERY_ACK, receivedDatagram, senderAddress, senderPort)
{
}

int DiscoveryCommandHandler::execute()
{

    if(!checkHeader()) {
        resultStatus = GEV_STATUS_INVALID_ADDRESS;
        return 1;
    }

    resultStatus = GEV_STATUS_SUCCESS;
    return 0;
}

quint16 DiscoveryCommandHandler::getAckBodyLength()
{
    if(resultStatus==GEV_STATUS_SUCCESS)
        return 248;
    return 0;
}

void DiscoveryCommandHandler::appendAckBody(QByteArray &datagram)
{
    if(resultStatus!=GEV_STATUS_SUCCESS)
        return;

    char answerChar[248];

    ConversionUtils::setShortToCharArray(answerChar, SPEC_VERSION_MAJOR, 0);
    ConversionUtils::setShortToCharArray(answerChar, SPEC_VERSION_MINOR, 2);

    int deviceMode = (dynamic_cast<GVDevice*>(target))->getRegister(REG_DEVICE_MODE)->getValue();
    ConversionUtils::setIntToCharArray(answerChar, deviceMode, 4);

    answerChar[8]=0; //RESERVED
    answerChar[9]=0; //RESERVED

    int MACHigh = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_DEVICE_MAC_ADD_HIGH)->getValue();
    int MACLow = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_DEVICE_MAC_ADD_LOW)->getValue();

    ConversionUtils::setShortToCharArray(answerChar, MACHigh, 10);
    ConversionUtils::setIntToCharArray(answerChar, MACLow, 12);

    //IP options
    ConversionUtils::setIntToCharArray(answerChar, 0, 16);

    //IP current config
    ConversionUtils::setIntToCharArray(answerChar, 0, 20);

    //12 byte reserved
    for (int var = 24; var < 36; ++var)
        answerChar[var]=0;

    //Current IP
    int currentIP = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_CURRENT_IP_ADD)->getValue();
    ConversionUtils::setIntToCharArray(answerChar, currentIP, 36);

    //12 byte reserved
    for (int var = 40; var < 52; ++var)
        answerChar[var]=0;

    //Subnet mask
    int subnetMask = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_CURRENT_SUBNET_MASK)->getValue();
    ConversionUtils::setIntToCharArray(answerChar, subnetMask, 52);

    //12 byte reserved
    for (int var = 56; var < 68; ++var)
        answerChar[var]=0;

    //Default gateway
    int defGateway = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_CURRENT_DEFAULT_GATEWAY)->getValue();
    ConversionUtils::setIntToCharArray(answerChar, defGateway, 68);

    //Manufacture name 32 byte
    std::string manName = (dynamic_cast<GVDevice*>(target))->getRegister(REG_MANUFACTURE_NAME)->getValueString();
    for (int var = 0; var < manName.size() && var < 32; ++var)
        answerChar[var+72] = manName.data()[var];
    for (int var = manName.size(); var < 32; ++var)
        answerChar[var+72] = 0;

    //Model name 32 byte
    std::string modelName = (dynamic_cast<GVDevice*>(target))->getRegister(REG_MODEL_NAME)->getValueString();
    for (int var = 0; var < modelName.size() && var < 32; ++var)
        answerChar[var+104] = modelName.data()[var];
    for (int var = modelName.size(); var < 32; ++var)
        answerChar[var+104] = 0;

    //Device version name 32 byte
    std::string deviceVersion = (dynamic_cast<GVDevice*>(target))->getRegister(REG_DEVICE_VERSION)->getValueString();
    for (int var = 0; var < deviceVersion.size() && var < 32; ++var)
        answerChar[var+136] = deviceVersion.data()[var];
    for (int var = deviceVersion.size(); var < 32; ++var)
        answerChar[var+136] = 0;

    //Manufacture specific info 48 byte
    std::string manInfo = (dynamic_cast<GVDevice*>(target))->getRegister(REG_MANUFACTURE_INFO)->getValueString();
    for (int var = 0; var < manInfo.size() && var < 48; ++var)
        answerChar[var+168] = manInfo.data()[var];
    for (int var = manInfo.size(); var < 48; ++var)
        answerChar[var+168] = 0;

    //Serial number 16 byte
    std::string serialNumber = (dynamic_cast<GVDevice*>(target))->getRegister(REG_SERIAL_NUMBER)->getValueString();
    for (int var = 0; var < serialNumber.size() && var < 16; ++var)
        answerChar[var+216] = serialNumber.data()[var];
    for (int var = serialNumber.size(); var < 16; ++var)
        answerChar[var+216] = 0;

    //User definied name 16 byte
    std::string userDefienedName = (dynamic_cast<GVDevice*>(target))->getRegister(REG_USER_DEFINIED_NAME)->getValueString();
    for (int var = 0; var < userDefienedName.size() && var < 16; ++var)
        answerChar[var+232] = userDefienedName.data()[var];
    for (int var = userDefienedName.size(); var < 16; ++var)
        answerChar[var+232] = 0;

    datagram.append(answerChar, getAckBodyLength());
}


