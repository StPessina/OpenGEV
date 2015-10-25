#ifndef ABSTRACTMESSAGEHANDLERFACTORY_H
#define ABSTRACTMESSAGEHANDLERFACTORY_H

#include "CommonComponent/gvcomponent.h"
#include "CommonMessages/abstractmessagehandler.h"

/*!
 * \brief The AbstractMessageHandlerFactory class create new message handlers
 */
class AbstractMessageHandlerFactory
{
public:
    AbstractMessageHandlerFactory(GVComponent* target);

    virtual ~AbstractMessageHandlerFactory();

    virtual bool isValidCode(int messageCode) = 0;

    virtual AbstractMessageHandler* createMessageHandler(int messageCode,
                                                         QByteArray datagram,
                                                         QHostAddress senderAddress,
                                                         quint16 senderPort) = 0;

protected:
    GVComponent* target;
};

#endif // ABSTRACTMESSAGEHANDLERFACTORY_H
