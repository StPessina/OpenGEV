#ifndef DISCOVERYCOMMAND_H
#define DISCOVERYCOMMAND_H

#include "opengv_global.h"

#include "abstractcommand.h"
#include "gvapplication.h"

#include "ApplicationCommandCode.h"
#include "deviceackcode.h"

class DiscoveryCommand : public AbstractCommand
{
public:
    DiscoveryCommand(GVApplication* target);
    virtual ~DiscoveryCommand();

    DiscoveryCommand(GVApplication* target, QHostAddress destinationAddress, quint16 destinationPort);

    int getLengthWithoutHeader();

    char* getCommandDatagramWithoutHeader();

    int executeAnswer(QByteArray answer);
};

#endif // DISCOVERYCOMMAND_H
