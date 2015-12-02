#include "abstractcommandhandlerfactory.h"

AbstractCommandHandlerFactory::AbstractCommandHandlerFactory(GVComponent * const target)
    : AbstractPacketHandlerFactory(target)
{
    this->target = target;
}

AbstractCommandHandlerFactory::~AbstractCommandHandlerFactory()
{

}

int AbstractCommandHandlerFactory::getPacketHandlerIdentifier(const QByteArray &datagram)
{
    if(datagram.size()<5)
        return -1;
    return ConversionUtils::getShortFromQByteArray(datagram,2);
}
