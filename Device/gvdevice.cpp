#include "gvdevice.h"

GVDevice::GVDevice(string manufacture_name, string model_name, string device_name)
{
    initCommonRegisterMap();
    initNetworkRegisters();

    commonRegisters[REG_MANUFACTURE_NAME]->setValueString(manufacture_name);
    commonRegisters[REG_MODEL_NAME]->setValueString(model_name);
    commonRegisters[REG_DEVICE_VERSION]->setValueString(device_name);

    commonRegisters[REG_GVCP_CAPABILITY]->setBit(31); //Allow multiple read (R-158cd) (R-167cd)
}

GVDevice::~GVDevice()
{
    foreach (auto reg, commonRegisters)
        delete reg.second;
    foreach (auto netReg, networkRegister)
        delete netReg.second;
    foreach (auto streamReg, streamChannels)
        delete streamReg.second;
}

BootstrapRegister *GVDevice::getRegister(int registerCode)
{
    //Network interface registers
    if((registerCode>=0x0008 && registerCode<=0x0044) ||
            (registerCode>=0x064C && registerCode<=0x0900)) {
        int interfaceNumber = DeviceRegisterConverter::getInterfaceNumberFromNetworkRegister(registerCode);
        return networkRegister[interfaceNumber]->getRegisterByAbsoluteRegCode(registerCode);
    }

    //Stream registers
    if(registerCode>=0x0D00) {
        int channelNumber = DeviceRegisterConverter::getChannelNumberFromStreamChannel(registerCode);
        if(channelNumber<streamChannels.size())
            return NULL;
        return streamChannels[channelNumber]->getRegisterByAbsoluteRegCode(registerCode);
    }

    return commonRegisters[registerCode];
}

BootstrapRegister *GVDevice::getNetworkRegister(int interface, int offsetRegisterCode)
{
    return networkRegister[interface]->getRegister(offsetRegisterCode);
}

BootstrapRegister *GVDevice::getStreamChannelRegister(int id, int offsetRegisterCode)
{
    return streamChannels[id]->getRegister(offsetRegisterCode);
}

string GVDevice::getManufactureName()
{
    return commonRegisters[REG_MANUFACTURE_NAME]->getValueString();
}

string GVDevice::getModelName()
{
    return commonRegisters[REG_MODEL_NAME]->getValueString();
}

string GVDevice::getDeviceName()
{
    return commonRegisters[REG_DEVICE_VERSION]->getValueString();
}

Privilege GVDevice::checkChannelPrivilege(QHostAddress senderAddr, quint16 senderPort)
{
    ControlChannelPrivilege ctrlChannelPrivilege;

    bool ctrlAccessSwitchOver = commonRegisters[REG_CONTROL_CHANNEL_PRIVILEGE]->getBit(29);
    bool CtrlAccess = commonRegisters[REG_CONTROL_CHANNEL_PRIVILEGE]->getBit(30);
    bool ExclusiveAccess = commonRegisters[REG_CONTROL_CHANNEL_PRIVILEGE]->getBit(31);
    if(ExclusiveAccess)
        ctrlChannelPrivilege = EXCLUSIVE;
    else {
        if(CtrlAccess && !ctrlAccessSwitchOver)
            ctrlChannelPrivilege = CTRL_ACCESS;
        if(CtrlAccess && ctrlAccessSwitchOver)
            ctrlChannelPrivilege = CTRL_ACCESS_SWITCH_OVER;
        if(!CtrlAccess && !ctrlAccessSwitchOver)
            ctrlChannelPrivilege = MONITOR;
    }


    quint32 applicationAddr = commonRegisters[REG_PRIMARY_APPLICATION_IP_ADDRESS]->getValueNumb();
    quint16 applicationPort = commonRegisters[REG_PRIMARY_APPLICATION_PORT]->getValueNumb();

    switch (ctrlChannelPrivilege) {
        case MONITOR:
            return FULL;
        case CTRL_ACCESS:
            if(senderAddr.toIPv4Address()==applicationAddr && senderPort == applicationPort)
                return FULL;
            else
                return READ;
        case CTRL_ACCESS_SWITCH_OVER:
            if(senderAddr.toIPv4Address()==applicationAddr && senderPort == applicationPort)
                return FULL;
            else
                return READ_SWITCH_OVER;
        case EXCLUSIVE:
            if(senderAddr.toIPv4Address()==applicationAddr && senderPort == applicationPort)
                return FULL;
            else
                return DENIED;
        default:
            break;
    }
    return DENIED;
}

void GVDevice::changeControlChannelPrivilege(int value, QHostAddress primary_address, quint16 primary_port)
{
    commonRegisters[REG_CONTROL_CHANNEL_PRIVILEGE]->setValueNumb(value);
    int addr = primary_address.toIPv4Address();
    commonRegisters[REG_PRIMARY_APPLICATION_IP_ADDRESS]->setValueNumb(addr);
    commonRegisters[REG_PRIMARY_APPLICATION_PORT]->setValueNumb((ushort) primary_port);
}

void GVDevice::closeControlChannelPrivilege()
{
    commonRegisters[REG_CONTROL_CHANNEL_PRIVILEGE]->setValueNumb(0);
    commonRegisters[REG_PRIMARY_APPLICATION_IP_ADDRESS]->setValueNumb(0);
    commonRegisters[REG_PRIMARY_APPLICATION_PORT]->setValueNumb(0);
}

void GVDevice::initCommonRegisterMap()
{
    commonRegisters[REG_VESION]= new  BootstrapRegister(REG_VESION, "Version",RA_READ, 4);
    commonRegisters[REG_DEVICE_MODE] = new  BootstrapRegister(REG_DEVICE_MODE, "Device Mode",RA_READ, 4);

    commonRegisters[REG_MANUFACTURE_NAME] = new  BootstrapRegister(REG_MANUFACTURE_NAME, "Manufacture name",RA_READ, 32);
    commonRegisters[REG_MODEL_NAME] = new  BootstrapRegister(REG_MODEL_NAME, "Model name",RA_READ, 32);
    commonRegisters[REG_DEVICE_VERSION] = new  BootstrapRegister(REG_DEVICE_VERSION, "Device version",RA_READ, 32);
    commonRegisters[REG_MANUFACTURE_INFO] = new  BootstrapRegister(REG_MANUFACTURE_INFO, "Manufacture info",RA_READ, 48);
    commonRegisters[REG_SERIAL_NUMBER] = new  BootstrapRegister(REG_SERIAL_NUMBER, "Serial number",RA_READ, 16);
    commonRegisters[REG_USER_DEFINIED_NAME] = new  BootstrapRegister(REG_USER_DEFINIED_NAME, "User-definied name",RA_READ_WRITE, 16);
    commonRegisters[REG_FIRST_URL] = new  BootstrapRegister(REG_FIRST_URL, "First URL",RA_READ, 512);
    commonRegisters[REG_SECOND_URL] = new  BootstrapRegister(REG_SECOND_URL, "Second URL",RA_READ, 512);

    commonRegisters[REG_NR_NETWORK_INTERFACE] = new  BootstrapRegister(REG_NR_NETWORK_INTERFACE, "Number Of Network Interface",RA_READ, 4);

    commonRegisters[REG_NR_MESSAGE_CHANNELS] = new  BootstrapRegister(REG_NR_MESSAGE_CHANNELS, "Number of Message Channels",RA_READ, 4);
    commonRegisters[REG_NR_STREAM_CHANNELS] = new  BootstrapRegister(REG_NR_STREAM_CHANNELS, "Number of Stream Channels",RA_READ, 4);
    commonRegisters[REG_ACTION_DEVICE_KEY] = new  BootstrapRegister(REG_ACTION_DEVICE_KEY, "Action Device Key", RA_WRITE, 4);
    commonRegisters[REG_NR_ACTIVE_LINKS] = new  BootstrapRegister(REG_NR_ACTIVE_LINKS, "Number of Active Links",RA_READ, 4);
    commonRegisters[REG_GVSP_CAPABILITY] = new  BootstrapRegister(REG_GVSP_CAPABILITY, "GVSP Capability",RA_READ, 4);
    commonRegisters[REG_MESSAGE_CHANNEL_CAPABILITY] = new  BootstrapRegister(REG_MESSAGE_CHANNEL_CAPABILITY, "Message Channel Capability",RA_READ, 4);
    commonRegisters[REG_GVCP_CAPABILITY] = new  BootstrapRegister(REG_GVCP_CAPABILITY, "GVCP Capability",RA_READ, 4);
    commonRegisters[REG_HEARTBEAT_TIMEOUT] = new  BootstrapRegister(REG_HEARTBEAT_TIMEOUT, "Heartbeat Timeout",RA_READ_WRITE, 4);
    commonRegisters[REG_TIMEOUT_TICK_FREQUENCY_HIGH] = new  BootstrapRegister(REG_TIMEOUT_TICK_FREQUENCY_HIGH, "Timestamp Tick Frequency - High",RA_READ, 4);
    commonRegisters[REG_TIMEOUT_TICK_FREQUENCY_LOW] = new  BootstrapRegister(REG_TIMEOUT_TICK_FREQUENCY_LOW, "Timestamp Tick Frequency - Low",RA_READ, 4);
    commonRegisters[REG_TIMEOUT_CONTROL] = new  BootstrapRegister(REG_TIMEOUT_CONTROL, "Timestamp Control",RA_WRITE, 4);
    commonRegisters[REG_DISCOVERY_ACK_DELAY] = new  BootstrapRegister(REG_DISCOVERY_ACK_DELAY, "Discovery ACK Delay",RA_READ_WRITE, 4);
    commonRegisters[REG_GVCP_CONFIGURATION] = new  BootstrapRegister(REG_GVCP_CONFIGURATION, "GVCP Configuration",RA_READ_WRITE, 4);
    commonRegisters[REG_PENDING_TIMEOUT] = new  BootstrapRegister(REG_PENDING_TIMEOUT, "Pending timeout",RA_READ, 4);
    commonRegisters[REG_CONTROL_SWITCHOVER_KEY] = new  BootstrapRegister(REG_CONTROL_SWITCHOVER_KEY, "Control Switchover Key",RA_WRITE, 4);

    commonRegisters[REG_CONTROL_CHANNEL_PRIVILEGE] = new  BootstrapRegister(REG_CONTROL_CHANNEL_PRIVILEGE, "Control Channel Privilege (0-1)",RA_READ_WRITE, 4);
    commonRegisters[REG_PRIMARY_APPLICATION_PORT] = new  BootstrapRegister(REG_PRIMARY_APPLICATION_PORT, "Primary application port",RA_READ, 4);
    commonRegisters[REG_PRIMARY_APPLICATION_IP_ADDRESS] = new  BootstrapRegister(REG_PRIMARY_APPLICATION_IP_ADDRESS, "Primary application address",RA_READ, 4);

}

void GVDevice::initNetworkRegisters()
{
    QList<QNetworkInterface> validNotConnected;
    int interfaceNumber = 0;
    foreach (QNetworkInterface interface, QNetworkInterface::allInterfaces()) {
        if(!(bool)(interface.flags() & QNetworkInterface::IsLoopBack)
                && interface.hardwareAddress().compare("00:00:00:00:00:00")!=0) {
                if(interface.addressEntries().size()!=0) { //Check if interface is connected
                    logger.debugStream()<<"GVDevice adding network connected interface #"<<to_string(interfaceNumber)
                                       <<"; Name:"<<interface.name().toStdString()
                                       <<"; MAC:"<<interface.hardwareAddress().toStdString();
                    networkRegister[interfaceNumber] = new NetworkInterfaceRegisters(interface, interfaceNumber);
                    interfaceNumber++;
                } else {
                    validNotConnected.push_back(interface);
                }
        }
    }

    foreach (QNetworkInterface interface, validNotConnected) {
        logger.debugStream()<<"GVDevice adding network NOT connected interface #"<<to_string(interfaceNumber)
                           <<"; Name:"<<interface.name().toStdString()
                           <<"; MAC:"<<interface.hardwareAddress().toStdString();
        networkRegister[interfaceNumber] = new NetworkInterfaceRegisters(interface, interfaceNumber);
        interfaceNumber++;
    }
}
