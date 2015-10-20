#ifndef GVDEVICE_H
#define GVDEVICE_H

#include <string>
#include <map>
#include <vector>

#include "bootstrapregister.h"
#include "deviceregisters.h"

#include "networkinterfaceregisters.h"

#include "gvcomponent.h"



using namespace std;

class GVDevice : public GVComponent
{
public:
    GVDevice(string manufacture_name, string model_name, string device_name);

    BootstrapRegister *getRegister(int registerCode);

    BootstrapRegister *getNetworkRegister(int interface, int offsetRegisterCode);

    string getManufactureName();

    string getModelName();

    string getDeviceName();

private:

    map<int,BootstrapRegister*> commonRegisters;

    map<int,NetworkInterfaceRegisters*> networkRegister;

    void initRegisterMap();
};

#endif // GVDEVICE_H
