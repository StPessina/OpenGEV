#include "abstractmessagehandlerfactory.h"

AbstractMessageHandlerFactory::AbstractMessageHandlerFactory(GVComponent *target)
{
    this->target = target;
}

AbstractMessageHandlerFactory::~AbstractMessageHandlerFactory()
{

}

