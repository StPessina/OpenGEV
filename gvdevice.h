#ifndef GVDEVICE_H
#define GVDEVICE_H

#include <string>
#include <map>
#include <vector>

#include "bootstrapregister.h"
#include "deviceregisters.h"

#include "gvcomponent.h"



using namespace std;

class GVDevice : public GVComponent
{
public:
    GVDevice(string manufacture_name, string model_name, string device_name);

    RegisterAccess *getRegister(int registerCode);

    string getManufactureName();

    string getModelName();

    string getDeviceName();

private:

    map<int,BootstrapRegister*> registers;

    void initRegisterMap();
};

#endif // GVDEVICE_H
