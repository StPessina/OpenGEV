#include "abstractpackethandlerfactory.h"

AbstractPacketHandlerFactory::AbstractPacketHandlerFactory(GVComponent *target)
{
    this->target = target;
}

AbstractPacketHandlerFactory::~AbstractPacketHandlerFactory()
{

}

