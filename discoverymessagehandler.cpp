#include "discoverymessagehandler.h"

DiscoveryMessageHandler::DiscoveryMessageHandler(GVDevice* target, QByteArray datagram,
                                                 QHostAddress senderAddress, quint16 senderPort)
    : AbstractMessageHandler(target, DISCOVERY_ACK, datagram, senderAddress, senderPort)
{
}

bool DiscoveryMessageHandler::isAllowed(Privilege ctrlChannelPrivilege)
{
    return true;
}

int DiscoveryMessageHandler::execute(Privilege ctrlChannelPrivilege)
{
    return 0;
}

int DiscoveryMessageHandler::getAckDatagramLengthWithoutHeader()
{
    return 248;
}

char *DiscoveryMessageHandler::getAckDatagramWithoutHeader()
{
    char* answer = new char[248];

    answer[0]=SPEC_VERSION_MAJOR >> 8;
    answer[1]=SPEC_VERSION_MAJOR;

    answer[2]=SPEC_VERSION_MINOR >> 8;
    answer[3]=SPEC_VERSION_MINOR;

    int deviceMode = (dynamic_cast<GVDevice*>(target))->getRegister(REG_DEVICE_MODE)->getValueNumb();
    answer[4]=deviceMode >> 24;
    answer[5]=deviceMode >> 16;
    answer[6]=deviceMode >> 8;
    answer[7]=deviceMode;

    answer[8]=0; //RESERVED
    answer[9]=0; //RESERVED

    int MACHigh = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_DEVICE_MAC_ADD_HIGH)->getValueNumb();
    int MACLow = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_DEVICE_MAC_ADD_LOW)->getValueNumb();

    answer[10] = MACHigh >> 8;
    answer[11] = MACHigh;
    answer[12] = MACLow >> 24;
    answer[13] = MACLow >> 16;
    answer[14] = MACLow >> 8;
    answer[15] = MACLow;

    //IP options
    answer[16] = 0;
    answer[17] = 0;
    answer[18] = 0;
    answer[19] = 0;

    //IP current config
    answer[20] = 0;
    answer[21] = 0;
    answer[22] = 0;
    answer[23] = 0;

    //12 byte reserved
    for (int var = 24; var < 36; ++var)
        answer[var]=0;

    //Current IP
    int currentIP = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_CURRENT_IP_ADD)->getValueNumb();
    answer[36]=currentIP >> 24;
    answer[37]=currentIP >> 16;
    answer[38]=currentIP >> 8;
    answer[39]=currentIP;

    //12 byte reserved
    for (int var = 40; var < 52; ++var)
        answer[var]=0;

    //Subnet mask
    int subnetMask = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_CURRENT_SUBNET_MASK)->getValueNumb();
    answer[52]=subnetMask >> 24;
    answer[53]=subnetMask >> 16;
    answer[54]=subnetMask >> 8;
    answer[55]=subnetMask;

    //12 byte reserved
    for (int var = 56; var < 68; ++var)
        answer[var]=0;

    //Subnet mask
    int defGateway = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_CURRENT_DEFAULT_GATEWAY)->getValueNumb();
    answer[68]=defGateway >> 24;
    answer[69]=defGateway >> 16;
    answer[70]=defGateway >> 8;
    answer[71]=defGateway;

    //Manufacture name 32 byte
    std::string manName = (dynamic_cast<GVDevice*>(target))->getRegister(REG_MANUFACTURE_NAME)->getValueString();
    for (int var = 0; var < manName.size() && var < 32; ++var)
        answer[var+72] = manName.data()[var];
    for (int var = manName.size(); var < 32; ++var)
        answer[var+72] = 0;

    //Model name 32 byte
    std::string modelName = (dynamic_cast<GVDevice*>(target))->getRegister(REG_MODEL_NAME)->getValueString();
    for (int var = 0; var < modelName.size() && var < 32; ++var)
        answer[var+104] = modelName.data()[var];
    for (int var = modelName.size(); var < 32; ++var)
        answer[var+104] = 0;

    //Device version name 32 byte
    std::string deviceVersion = (dynamic_cast<GVDevice*>(target))->getRegister(REG_DEVICE_VERSION)->getValueString();
    for (int var = 0; var < deviceVersion.size() && var < 32; ++var)
        answer[var+136] = deviceVersion.data()[var];
    for (int var = deviceVersion.size(); var < 32; ++var)
        answer[var+136] = 0;

    //Manufacture specific info 48 byte
    std::string manInfo = (dynamic_cast<GVDevice*>(target))->getRegister(REG_MANUFACTURE_INFO)->getValueString();
    for (int var = 0; var < manInfo.size() && var < 48; ++var)
        answer[var+168] = manInfo.data()[var];
    for (int var = manInfo.size(); var < 48; ++var)
        answer[var+168] = 0;

    //Serial number 16 byte
    std::string serialNumber = (dynamic_cast<GVDevice*>(target))->getRegister(REG_SERIAL_NUMBER)->getValueString();
    for (int var = 0; var < serialNumber.size() && var < 16; ++var)
        answer[var+216] = serialNumber.data()[var];
    for (int var = serialNumber.size(); var < 16; ++var)
        answer[var+216] = 0;

    //User definied name 16 byte
    std::string userDefienedName = (dynamic_cast<GVDevice*>(target))->getRegister(REG_USER_DEFINIED_NAME)->getValueString();
    for (int var = 0; var < userDefienedName.size() && var < 16; ++var)
        answer[var+232] = userDefienedName.data()[var];
    for (int var = userDefienedName.size(); var < 16; ++var)
        answer[var+232] = 0;

    return answer;
}


