#ifndef GVDEVICE_H
#define GVDEVICE_H

#include <string>
#include <unordered_map>

#include <vector>

#include "CommonBootstrapRegister/bootstrapregister.h"
#include "Device/deviceregisters.h"

#include "Device/networkinterfaceregisters.h"

#include "CommonMessages/privilege.h"

#include "CommonComponent/gvcomponent.h"
#include "CommonControlChannel/controlchannelprivilege.h"

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

    /**
     * @brief checkChannelPrivilege method return the right of the sender
     * @param senderAddr
     * @param senderPort
     * @return right for the sender
     */
    Privilege checkChannelPrivilege(QHostAddress senderAddr, quint16 senderPort);

    /**
     * @brief changeControlChannelPrivilege
     * @param primary_address
     * @param primary_port
     */
    void changeControlChannelPrivilege(int value, QHostAddress primary_address, quint16 primary_port);

    /**
     * @brief closeControlChannelPrivilege
     */
    void closeControlChannelPrivilege();

private:

    /**
     * @brief commonRegisters map
     */
    unordered_map<int,BootstrapRegister*> commonRegisters;

    /**
     * @brief networkRegister map
     */
    unordered_map<int,NetworkInterfaceRegisters*> networkRegister;


    log4cpp::Category &logger = log4cpp::Category::getInstance("ComponentLog");

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
