#ifndef ABSTRACTMESSAGE_H
#define ABSTRACTMESSAGE_H

#include <QString>
#include <QHostAddress>

#include "gvcomponent.h"
#include "privilege.h"

/*!
 * \brief The AbstractMessage class
 */
class AbstractMessage
{
public:
    /*!
     * \brief AbstractMessage
     * \param Target of this command
     */
    AbstractMessage(GVComponent* target);

    /*!
     * \brief getTarget
     * \return target for this command
     */
    GVComponent* getTarget();

    /*!
     * \brief isAllowed
     * \return true if this command is allowed on the target
     */
    virtual bool isAllowed(Privilege ctrlChannelPrivilege);

    /*!
     * \brief execute a command from
     * \param datagram data received
     * \param sender of datagram
     * \param port
     * \return error code
     */
    virtual int execute(QByteArray datagram, QHostAddress sender, quint16 port);

    /*!
     * \brief getAck
     * \return message for acknowledgement
     */
    virtual QByteArray getAck();

private:
    /*!
     * \brief target component
     */
    GVComponent* target;


};

#endif // ABSTRACTMESSAGE_H
