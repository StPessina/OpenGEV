#ifndef GVDEVICE_H
#define GVDEVICE_H

#include <string>
#include <unordered_map>

#include <vector>

#include "CommonBootstrapRegister/bootstrapregister.h"
#include "Device/deviceregisters.h"

#include "Device/networkinterfaceregisters.h"

#include "CommonComponent/gvcomponent.h"

#include "iostream"


using namespace std;

class GVDevice : public GVComponent
{
public:
    GVDevice(string manufacture_name, string model_name, string device_name);
    virtual ~GVDevice();

    BootstrapRegister *getRegister(int registerCode);

    BootstrapRegister *getNetworkRegister(int interface, int offsetRegisterCode);

    string getManufactureName();

    string getModelName();

    string getDeviceName();

private:

    unordered_map<int,BootstrapRegister*> commonRegisters;

    unordered_map<int,NetworkInterfaceRegisters*> networkRegister;

    void initCommonRegisterMap();

    void initNetworkRegisters();
};

#endif // GVDEVICE_H
