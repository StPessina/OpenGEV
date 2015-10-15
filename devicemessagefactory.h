#ifndef DEVICEMESSAGEFACTORY_H
#define DEVICEMESSAGEFACTORY_H

#include "abstractmessagefactory.h"

#include "gvdevice.h"

class DeviceMessageFactory : public AbstractMessageFactory
{
public:
    DeviceMessageFactory(GVDevice* target);

    virtual bool isValidCode(int messageCode);

    virtual AbstractMessage createMessageHandler(int messageCode);
};

#endif // DEVICEMESSAGEFACTORY_H
