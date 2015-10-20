#include "gvdevice.h"

GVDevice::GVDevice(string manufacture_name, string model_name, string device_name)
{
    registers.at(REG_MANUFACTURE_NAME)->setValueString(manufacture_name);
    registers.at(REG_MODEL_NAME)->setValueString(model_name);
    registers.at(REG_DEVICE_VERSION)->setValueString(device_name);
}

RegisterAccess* GVDevice::getRegister(int registerCode)
{
    return (RegisterAccess*) registers.at(registerCode);
}

string GVDevice::getManufactureName()
{
    return registers.at(REG_MANUFACTURE_NAME)->getValueString();
}

string GVDevice::getModelName()
{
    return registers.at(REG_MODEL_NAME)->getValueString();
}

string GVDevice::getDeviceName()
{
    return registers.at(REG_DEVICE_VERSION)->getValueString();
}

void GVDevice::initRegisterMap()
{
    registers[REG_VESION]= new  BootstrapRegister(REG_VESION, "Version",RA_READ, 4);
    registers[REG_DEVICE_MODE] = new  BootstrapRegister(REG_DEVICE_MODE, "Device Mode",RA_READ, 4);

    registers[0x0008] = new  BootstrapRegister(0x0008, "Device MAC Address - High (Network interface #0)",RA_READ, 4);
    registers[0x000C] = new  BootstrapRegister(0x000C, "Device MAC Address - Low (Network interface #0)",RA_READ, 4);
    registers[0x0010] = new  BootstrapRegister(0x0010, "Network Interface Capability (Network interface #0)",RA_READ, 4);
    registers[0x0014] = new  BootstrapRegister(0x0014, "Network Interface Configuration (Network interface #0)",RA_READ_WRITE, 4);
    registers[0x0024] = new  BootstrapRegister(0x0024, "Current IP Address (Network interface #0)",RA_READ, 4);
    registers[0x0034] = new  BootstrapRegister(0x0034, "Current Subnet Mask (Network interface #0)",RA_READ, 4);
    registers[0x0044] = new  BootstrapRegister(0x0044, "Current Default Gateway (Network interface #0)",RA_READ, 4);

    registers[0x0048] = new  BootstrapRegister(0x0048, "Manufacture name",RA_READ, 32);
    registers[0x0068] = new  BootstrapRegister(0x0068, "Model name",RA_READ, 32);
    registers[0x0088] = new  BootstrapRegister(0x0088, "Device version",RA_READ, 32);
    registers[0x00A8] = new  BootstrapRegister(0x00A8, "Manufacture info",RA_READ, 48);
    registers[0x00D8] = new  BootstrapRegister(0x00D8, "Serial number",RA_READ, 16);
    registers[0x00E8] = new  BootstrapRegister(0x00E8, "User-definied name",RA_READ_WRITE, 16);
    registers[0x0200] = new  BootstrapRegister(0x0200, "First URL",RA_READ, 512);
    registers[0x0400] = new  BootstrapRegister(0x0400, "Second URL",RA_READ, 512);

    registers[0x0600] = new  BootstrapRegister(0x0600, "Number Of Network Interface",RA_READ, 4);

    registers[0x064C] = new  BootstrapRegister(0x064C, "Persistent IP Address (Network interface #0)",RA_READ_WRITE, 4);
    registers[0x065C] = new  BootstrapRegister(0x065C, "Persistent Subnet Mask (Network interface #0)",RA_READ_WRITE, 4);
    registers[0x066C] = new  BootstrapRegister(0x066C, "Persistent Default Gateway (Network interface #0)",RA_READ_WRITE, 4);
    registers[0x0670] = new  BootstrapRegister(0x0670, "Link Speed (Network interface #0)",RA_READ, 4);

    registers[0x0680] = new  BootstrapRegister(0x0680, "Device MAC Address - High (Network interface #1)",RA_READ, 4);
    registers[0x0684] = new  BootstrapRegister(0x0684, "Device MAC Address - Low (Network interface #1)",RA_READ, 4);
    registers[0x0688] = new  BootstrapRegister(0x0688, "Network Interface Capability (Network interface #1)",RA_READ, 4);
    registers[0x068C] = new  BootstrapRegister(0x068C, "Network Interface Configuration (Network interface #1)",RA_READ_WRITE, 4);
    registers[0x069C] = new  BootstrapRegister(0x069C, "Current IP Address (Network interface #1)",RA_READ, 4);
    registers[0x06AC] = new  BootstrapRegister(0x06AC, "Current Subnet Mask (Network interface #1)",RA_READ, 4);
    registers[0x06BC] = new  BootstrapRegister(0x06BC, "Current Default Gateway (Network interface #1)",RA_READ, 4);
    registers[0x06CC] = new  BootstrapRegister(0x06CC, "Persistent IP Address (Network interface #1)",RA_READ_WRITE, 4);
    registers[0x06DC] = new  BootstrapRegister(0x06DC, "Persistent Subnet Mask (Network interface #1)",RA_READ_WRITE, 4);
    registers[0x06EC] = new  BootstrapRegister(0x06EC, "Persistent Default Gateway (Network interface #1)",RA_READ_WRITE, 4);
    registers[0x06FC] = new  BootstrapRegister(0x06FC, "Link Speed (Network interface #1)",RA_READ, 4);

    registers[0x0700] = new  BootstrapRegister(0x0700, "Device MAC Address - High (Network interface #2)",RA_READ, 4);
    registers[0x0704] = new  BootstrapRegister(0x0704, "Device MAC Address - Low (Network interface #2)",RA_READ, 4);
    registers[0x0708] = new  BootstrapRegister(0x0708, "Network Interface Capability (Network interface #2)",RA_READ, 4);
    registers[0x070C] = new  BootstrapRegister(0x070C, "Network Interface Configuration (Network interface #2)",RA_READ_WRITE, 4);
    registers[0x071C] = new  BootstrapRegister(0x071C, "Current IP Address (Network interface #2)",RA_READ, 4);
    registers[0x072C] = new  BootstrapRegister(0x072C, "Current Subnet Mask (Network interface #2)",RA_READ, 4);
    registers[0x073C] = new  BootstrapRegister(0x073C, "Current Default Gateway (Network interface #2)",RA_READ, 4);
    registers[0x074C] = new  BootstrapRegister(0x074C, "Persistent IP Address (Network interface #2)",RA_READ_WRITE, 4);
    registers[0x075C] = new  BootstrapRegister(0x075C, "Persistent Subnet Mask (Network interface #2)",RA_READ_WRITE, 4);
    registers[0x076C] = new  BootstrapRegister(0x076C, "Persistent Default Gateway (Network interface #2)",RA_READ_WRITE, 4);
    registers[0x0770] = new  BootstrapRegister(0x0770, "Link Speed (Network interface #2)",RA_READ, 4);

    registers[0x0780] = new  BootstrapRegister(0x0780, "Device MAC Address - High (Network interface #3)",RA_READ, 4);
    registers[0x0784] = new  BootstrapRegister(0x0784, "Device MAC Address - Low (Network interface #3)",RA_READ, 4);
    registers[0x0788] = new  BootstrapRegister(0x0788, "Network Interface Capability (Network interface #3)",RA_READ, 4);
    registers[0x078C] = new  BootstrapRegister(0x078C, "Network Interface Configuration (Network interface #3)",RA_READ_WRITE, 4);
    registers[0x079C] = new  BootstrapRegister(0x079C, "Current IP Address (Network interface #3)",RA_READ, 4);
    registers[0x07AC] = new  BootstrapRegister(0x07AC, "Current Subnet Mask (Network interface #3)",RA_READ, 4);
    registers[0x07BC] = new  BootstrapRegister(0x07BC, "Current Default Gateway (Network interface #3)",RA_READ, 4);
    registers[0x07CC] = new  BootstrapRegister(0x07CC, "Persistent IP Address (Network interface #3)",RA_READ_WRITE, 4);
    registers[0x07DC] = new  BootstrapRegister(0x07DC, "Persistent Subnet Mask (Network interface #3)",RA_READ_WRITE, 4);
    registers[0x07EC] = new  BootstrapRegister(0x07EC, "Persistent Default Gateway (Network interface #3)",RA_READ_WRITE, 4);
    registers[0x07FC] = new  BootstrapRegister(0x07FC, "Link Speed (Network interface #3)",RA_READ, 4);

    registers[0x0900] = new  BootstrapRegister(0x0900, "Number of Message Channels",RA_READ, 4);
    registers[0x0904] = new  BootstrapRegister(0x0904, "Number of Stream Channels",RA_READ, 4);
    registers[0x090C] = new  BootstrapRegister(0x090C, "Action Device Key", RA_WRITE, 4);
    registers[0x0910] = new  BootstrapRegister(0x0910, "Number of Active Links",RA_READ, 4);
    registers[0x092C] = new  BootstrapRegister(0x092C, "GVSP Capability",RA_READ, 4);
    registers[0x0930] = new  BootstrapRegister(0x0930, "Message Channel Capability",RA_READ, 4);
    registers[0x0934] = new  BootstrapRegister(0x0934, "GVCP Capability",RA_READ, 4);
    registers[0x0938] = new  BootstrapRegister(0x0938, "Heartbeat Timeout",RA_READ_WRITE, 4);
    registers[0x093C] = new  BootstrapRegister(0x093C, "Timestamp Tick Frequency - High",RA_READ, 4);
    registers[0x0940] = new  BootstrapRegister(0x0940, "Timestamp Tick Frequency - Low",RA_READ, 4);
    registers[0x0944] = new  BootstrapRegister(0x0944, "Timestamp Control",RA_WRITE, 4);
    registers[0x0950] = new  BootstrapRegister(0x0950, "Discovery ACK Delay",RA_READ_WRITE, 4);
    registers[0x0954] = new  BootstrapRegister(0x0954, "GVCP Configuration",RA_READ_WRITE, 4);
    registers[0x0958] = new  BootstrapRegister(0x0958, "Pending timeout",RA_READ, 4);
    registers[0x095C] = new  BootstrapRegister(0x095C, "Control Switchover Key",RA_WRITE, 4);

}
