#ifndef ABSTRACTMESSAGEFACTORY_H
#define ABSTRACTMESSAGEFACTORY_H

#include "gvcomponent.h"
#include "abstractmessage.h"

/*!
 * \brief The AbstractMessageFactory class create new message handlers
 */
class AbstractMessageFactory
{
public:
    AbstractMessageFactory(GVComponent* target);

    virtual AbstractMessage createMessageHandler(int messageCode);

private:
    GVComponent* target;
};

#endif // ABSTRACTMESSAGEFACTORY_H
