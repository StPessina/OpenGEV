#include "partnerdevice.h"

PartnerDevice::PartnerDevice()
{
}

PartnerDevice::~PartnerDevice()
{
    foreach (auto channel, controlChannels)
        delete channel.second;
}
