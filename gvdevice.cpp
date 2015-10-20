#include "gvdevice.h"

GVDevice::GVDevice(string manufacture_name, string model_name, string device_name)
{
     initRegisterMap();

    commonRegisters.at(REG_MANUFACTURE_NAME)->setValueString(manufacture_name);
    commonRegisters.at(REG_MODEL_NAME)->setValueString(model_name);
    commonRegisters.at(REG_DEVICE_VERSION)->setValueString(device_name);
}

BootstrapRegister *GVDevice::getRegister(int registerCode)
{
    return (BootstrapRegister*) commonRegisters.at(registerCode);
}

BootstrapRegister *GVDevice::getNetworkRegister(int interface, int offsetRegisterCode)
{
    return (BootstrapRegister*) networkRegister.at(interface)->getRegister(offsetRegisterCode);
}

string GVDevice::getManufactureName()
{
    return commonRegisters.at(REG_MANUFACTURE_NAME)->getValueString();
}

string GVDevice::getModelName()
{
    return commonRegisters.at(REG_MODEL_NAME)->getValueString();
}

string GVDevice::getDeviceName()
{
    return commonRegisters.at(REG_DEVICE_VERSION)->getValueString();
}

void GVDevice::initRegisterMap()
{
    commonRegisters[REG_VESION]= new  BootstrapRegister(REG_VESION, "Version",RA_READ, 4);
    commonRegisters[REG_DEVICE_MODE] = new  BootstrapRegister(REG_DEVICE_MODE, "Device Mode",RA_READ, 4);

    commonRegisters[REG_MANUFACTURE_NAME] = new  BootstrapRegister(REG_MANUFACTURE_NAME, "Manufacture name",RA_READ, 32);
    commonRegisters[REG_MODEL_NAME] = new  BootstrapRegister(REG_MODEL_NAME, "Model name",RA_READ, 32);
    commonRegisters[REG_DEVICE_VERSION] = new  BootstrapRegister(REG_DEVICE_VERSION, "Device version",RA_READ, 32);
    commonRegisters[0x00A8] = new  BootstrapRegister(0x00A8, "Manufacture info",RA_READ, 48);
    commonRegisters[0x00D8] = new  BootstrapRegister(0x00D8, "Serial number",RA_READ, 16);
    commonRegisters[0x00E8] = new  BootstrapRegister(0x00E8, "User-definied name",RA_READ_WRITE, 16);
    commonRegisters[0x0200] = new  BootstrapRegister(0x0200, "First URL",RA_READ, 512);
    commonRegisters[0x0400] = new  BootstrapRegister(0x0400, "Second URL",RA_READ, 512);

    commonRegisters[0x0600] = new  BootstrapRegister(0x0600, "Number Of Network Interface",RA_READ, 4);

    for (int var = 0; var < QNetworkInterface::allInterfaces().size(); ++var)
        networkRegister[var] = new NetworkInterfaceRegisters(var);

    commonRegisters[0x0900] = new  BootstrapRegister(0x0900, "Number of Message Channels",RA_READ, 4);
    commonRegisters[0x0904] = new  BootstrapRegister(0x0904, "Number of Stream Channels",RA_READ, 4);
    commonRegisters[0x090C] = new  BootstrapRegister(0x090C, "Action Device Key", RA_WRITE, 4);
    commonRegisters[0x0910] = new  BootstrapRegister(0x0910, "Number of Active Links",RA_READ, 4);
    commonRegisters[0x092C] = new  BootstrapRegister(0x092C, "GVSP Capability",RA_READ, 4);
    commonRegisters[0x0930] = new  BootstrapRegister(0x0930, "Message Channel Capability",RA_READ, 4);
    commonRegisters[0x0934] = new  BootstrapRegister(0x0934, "GVCP Capability",RA_READ, 4);
    commonRegisters[0x0938] = new  BootstrapRegister(0x0938, "Heartbeat Timeout",RA_READ_WRITE, 4);
    commonRegisters[0x093C] = new  BootstrapRegister(0x093C, "Timestamp Tick Frequency - High",RA_READ, 4);
    commonRegisters[0x0940] = new  BootstrapRegister(0x0940, "Timestamp Tick Frequency - Low",RA_READ, 4);
    commonRegisters[0x0944] = new  BootstrapRegister(0x0944, "Timestamp Control",RA_WRITE, 4);
    commonRegisters[0x0950] = new  BootstrapRegister(0x0950, "Discovery ACK Delay",RA_READ_WRITE, 4);
    commonRegisters[0x0954] = new  BootstrapRegister(0x0954, "GVCP Configuration",RA_READ_WRITE, 4);
    commonRegisters[0x0958] = new  BootstrapRegister(0x0958, "Pending timeout",RA_READ, 4);
    commonRegisters[0x095C] = new  BootstrapRegister(0x095C, "Control Switchover Key",RA_WRITE, 4);

}
