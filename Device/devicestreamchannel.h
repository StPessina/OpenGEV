#ifndef DEVICESTREAMCHANNEL_H
#define DEVICESTREAMCHANNEL_H

#include <map>

#include "CommonBootstrapRegister/bootstrapregister.h"

#include "Device/deviceregisters.h"

class DeviceStreamChannel
{
public:
    DeviceStreamChannel(int id);

    ~DeviceStreamChannel();

    BootstrapRegister *getRegister(int regType);

    BootstrapRegister *getRegisterByAbsoluteRegCode(int regCode);

private:
    int id;

    /**
     * @brief map of network register
     */
    map<int,BootstrapRegister*> registers;

    /**
     * @brief initRegisterMap
     */
    void initRegisterMap();

};

#endif // DEVICESTREAMCHANNEL_H
