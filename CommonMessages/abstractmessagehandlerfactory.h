#ifndef ABSTRACTMESSAGEHANDLERFACTORY_H
#define ABSTRACTMESSAGEHANDLERFACTORY_H

#include "CommonComponent/gvcomponent.h"
#include "CommonMessages/abstractmessagehandler.h"

/**
 * @brief The AbstractMessageHandlerFactory class create new message handlers
 */
class AbstractMessageHandlerFactory
{
public:
    /**
     * @brief AbstractMessageHandlerFactory constructor
     * @param target the component where the handler created will be executed
     */
    AbstractMessageHandlerFactory(GVComponent* target);

    /**
     * @brief ~AbstractMessageHandlerFactory deconstructor
     */
    virtual ~AbstractMessageHandlerFactory();

    /**
     * @brief isValidCode check if a message code exist
     * @param messageCode
     * @return true if the code is valid and an handler exist
     */
    virtual bool isValidCode(quint16 messageCode) = 0;

    /**
     * @brief createMessageHandler
     * @param messageCode
     * @param datagram
     * @param senderAddress
     * @param senderPort
     * @return message handler for the requested message code
     */
    virtual AbstractMessageHandler* createMessageHandler(quint16 messageCode,
                                                         QByteArray datagram,
                                                         QHostAddress senderAddress,
                                                         quint16 senderPort) = 0;

protected:
    GVComponent* target;
};

#endif // ABSTRACTMESSAGEHANDLERFACTORY_H
