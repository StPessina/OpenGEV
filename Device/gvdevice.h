#ifndef GVDEVICE_H
#define GVDEVICE_H

#include <string>
#include <unordered_map>

#include "opengev.h"

#include <vector>

#include "DeviceCommandHandler/devicecommandhandlerfactory.h"

#include "CommonBootstrapRegister/bootstrapregister.h"
#include "Device/deviceregisters.h"

#include "Device/networkinterfaceregisters.h"
#include "Device/streamchanneltransmitter.h"

#ifdef USE_QT_SOCKET
    #include "CommonUdpChannel/qtudpchannel.h"
#endif
#ifdef USE_BOOST_SOCKET
    #include "CommonUdpChannel/boostudpchannel.h"
#endif
#ifdef USE_OSAPI_SOCKET
    #include "CommonUdpChannel/osapiudpchannel.h"
#endif

#include "CommonUdpChannel/privilege.h"

#include "CommonComponent/gvcomponent.h"
#include "CommonUdpChannel/controlchannelprivilege.h"

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
     * @brief getStreamChannelRegister
     * @param id
     * @param offsetRegisterCode (use defined code)
     * @return stream channel register if exist
     */
    BootstrapRegister *getStreamChannelRegister(int id, int offsetRegisterCode);

    /**
     * @brief setRegister
     * @param registerCode
     * @param value
     * @param senderAddr
     * @param senderPort
     * @return status code
     */
    Status setRegister(int registerCode, int value, QHostAddress senderAddr, quint16 senderPort);

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

    /**
     * @brief createStreamChannel
     * @return stream channel number (-1 if fail)
     */
    int createStreamChannel();

    /**
     * @brief streamChannelExist
     * @param streamChannelCode
     * @return true if the stream channel exist
     */
    bool streamChannelExist(int streamChannelCode);

    /**
     * @brief getStreamChannel
     * @param streamChannelCode
     * @return stream channel if exist
     */
    StreamChannelTransmitter* getStreamChannel(int streamChannelCode);

    /**
     *  Configure 3D capabities and camera 3D information for depth recostruction
     *
     * @brief configure3DCapabilities
     * @param HFOVDegree
     * @param VFOVDegree
     */
    void configure3DCapabilities(int HFOVDegree, int VFOVDegree);

private:

    UDPChannel* controlChannel;

    /**
     * @brief commonRegisters map
     */
    unordered_map<int,BootstrapRegister*> commonRegisters;

    /**
     * @brief networkRegister map
     */
    unordered_map<int,NetworkInterfaceRegisters*> networkRegister;

    /**
     * @brief stream channel map
     */
    unordered_map<int,StreamChannelTransmitter*> streamChannels;

#ifdef ENABLE_LOG4CPP
    log4cpp::Category &logger = log4cpp::Category::getInstance("ComponentLog");
#endif

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
