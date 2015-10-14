#include "abstractmessage.h"

AbstractMessage::AbstractMessage(GVComponent* target)
{
    this->target=target;
}

GVComponent* AbstractMessage::getTarget()
{
    return target;
}
