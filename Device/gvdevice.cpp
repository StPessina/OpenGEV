#include "gvdevice.h"

GVDevice::GVDevice(string manufacture_name, string model_name, string device_name)
{
    initCommonRegisterMap();
    initNetworkRegisters();

    commonRegisters[REG_MANUFACTURE_NAME]->setValueString(manufacture_name);
    commonRegisters[REG_MODEL_NAME]->setValueString(model_name);
    commonRegisters[REG_DEVICE_VERSION]->setValueString(device_name);
}

GVDevice::~GVDevice()
{

}

BootstrapRegister *GVDevice::getRegister(int registerCode)
{
    return commonRegisters[registerCode];
}

BootstrapRegister *GVDevice::getNetworkRegister(int interface, int offsetRegisterCode)
{
    return networkRegister[interface]->getRegister(offsetRegisterCode);
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
