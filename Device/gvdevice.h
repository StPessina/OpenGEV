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

/**
 * @brief The GVDevice class a device component class
 */
class GVDevice : public GVComponent
{
public:
    /**
     * @brief GVDevice constructor
     * @param manufacture_name
     * @param model_name
     * @param device_name
     */
    GVDevice(string manufacture_name, string model_name, string device_name);

    /**
     * @brief ~GVDevice deconstructor
     */
    virtual ~GVDevice();

    /**
     * @brief getRegister
     * @param registerCode (use defined code)
     * @return register from map if the code exist
     */
    BootstrapRegister *getRegister(int registerCode);

    /**
     * @brief getNetworkRegister
     * @param interface
     * @param offsetRegisterCode (use defined code)
     * @return network register if exist
     */
    BootstrapRegister *getNetworkRegister(int interface, int offsetRegisterCode);

    /**
     * @brief getManufactureName
     * @return manufacture name
     */
    string getManufactureName();

    /**
     * @brief getModelName
     * @return model name
     */
    string getModelName();

    /**
     * @brief getDeviceName
     * @return device name
     */
    string getDeviceName();

private:

    /**
     * @brief commonRegisters map
     */
    unordered_map<int,BootstrapRegister*> commonRegisters;

    /**
     * @brief networkRegister map
     */
    unordered_map<int,NetworkInterfaceRegisters*> networkRegister;

    /**
     * @brief initCommonRegisterMap
     */
    void initCommonRegisterMap();

    /**
     * @brief initNetworkRegisters
     */
    void initNetworkRegisters();
};

#endif // GVDEVICE_H
