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
    return 246;
}

char *DiscoveryMessageHandler::getAckDatagramWithoutHeader()
{
    char* answer = new char[246];

    answer[0]=SPEC_VERSION_MAJOR / 256;
    answer[1]=SPEC_VERSION_MAJOR % 256;

    answer[2]=SPEC_VERSION_MINOR / 256;
    answer[3]=SPEC_VERSION_MINOR % 256;

    int deviceMode = (dynamic_cast<GVDevice*>(target))->getRegister(REG_DEVICE_MODE)->getValueNumb();
    answer[4]=deviceMode;
    answer[5]=deviceMode << 8;
    answer[6]=deviceMode << 16;
    answer[7]=deviceMode << 24;

    answer[8]=0; //RESERVED
    answer[9]=0; //RESERVED

    int MACHigh = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_DEVICE_MAC_ADD_HIGH)->getValueNumb();
    int MACLow = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_DEVICE_MAC_ADD_LOW)->getValueNumb();
    answer[10] = MACHigh;
    answer[11] = MACHigh << 8;
    answer[12] = MACLow;
    answer[13] = MACLow << 8;
    answer[14] = MACLow << 16;
    answer[15] = MACLow << 24;

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
    answer[36]=currentIP;
    answer[35]=currentIP << 8;
    answer[36]=currentIP << 16;
    answer[37]=currentIP << 24;

    //12 byte reserved
    for (int var = 38; var < 50; ++var)
        answer[var]=0;

    //Subnet mask
    int subnetMask = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_CURRENT_SUBNET_MASK)->getValueNumb();
    answer[50]=subnetMask;
    answer[51]=subnetMask << 8;
    answer[52]=subnetMask << 16;
    answer[53]=subnetMask << 24;

    //12 byte reserved
    for (int var = 54; var < 66; ++var)
        answer[var]=0;

    //Subnet mask
    int defGateway = (dynamic_cast<GVDevice*>(target))->getNetworkRegister(0, REG_CURRENT_DEFAULT_GATEWAY)->getValueNumb();
    answer[66]=defGateway;
    answer[67]=defGateway << 8;
    answer[68]=defGateway << 16;
    answer[69]=defGateway << 24;

    //Manufacture name 32 byte
    std::string manName = (dynamic_cast<GVDevice*>(target))->getRegister(REG_MANUFACTURE_NAME)->getValueString();
    for (int var = 0; var < manName.size() && var < 32; ++var)
        answer[var+70] = manName.data()[var];
    for (int var = manName.size(); var < 32; ++var)
        answer[var+70] = 0;

    //Model name 32 byte
    std::string modelName = (dynamic_cast<GVDevice*>(target))->getRegister(REG_MODEL_NAME)->getValueString();
    for (int var = 102; var < modelName.size() && var < 134; ++var)
        answer[var] = modelName.data()[var];
    for (int var = 102+modelName.size(); var < 134; ++var)
        answer[var] = 0;

    //Device version name 32 byte
    std::string deviceVersion = (dynamic_cast<GVDevice*>(target))->getRegister(REG_DEVICE_VERSION)->getValueString();
    for (int var = 0; var < deviceVersion.size() && var < 32; ++var)
        answer[var+134] = deviceVersion.data()[var];
    for (int var = deviceVersion.size(); var < 32; ++var)
        answer[var+134] = 0;

    //Manufacture specific info 48 byte
    std::string manInfo = (dynamic_cast<GVDevice*>(target))->getRegister(REG_MANUFACTURE_INFO)->getValueString();
    for (int var = 0; var < manInfo.size() && var < 48; ++var)
        answer[var+166] = manInfo.data()[var];
    for (int var = manInfo.size(); var < 48; ++var)
        answer[var+166] = 0;

    //Serial number 16 byte
    std::string serialNumber = (dynamic_cast<GVDevice*>(target))->getRegister(REG_SERIAL_NUMBER)->getValueString();
    for (int var = 0; var < serialNumber.size() && var < 16; ++var)
        answer[var+214] = serialNumber.data()[var];
    for (int var = serialNumber.size(); var < 16; ++var)
        answer[var+214] = 0;

    //User definied name 16 byte
    std::string userDefienedName = (dynamic_cast<GVDevice*>(target))->getRegister(REG_USER_DEFINIED_NAME)->getValueString();
    for (int var = 0; var < userDefienedName.size() && var < 16; ++var)
        answer[var+230] = userDefienedName.data()[var];
    for (int var = userDefienedName.size(); var < 16; ++var)
        answer[var+230] = 0;

    return answer;
}


