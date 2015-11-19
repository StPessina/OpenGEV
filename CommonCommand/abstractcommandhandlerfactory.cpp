#include "abstractcommandhandlerfactory.h"

AbstractCommandHandlerFactory::AbstractCommandHandlerFactory(GVComponent * const target)
    : AbstractPacketHandlerFactory(target)
{
    this->target = target;
}

AbstractCommandHandlerFactory::~AbstractCommandHandlerFactory()
{

}

int AbstractCommandHandlerFactory::getPacketHandlerIdentifier(QByteArray datagram)
{
    return ConversionUtils::getShortFromQByteArray(datagram,2);
}
