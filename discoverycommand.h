#ifndef DISCOVERYCOMMAND_H
#define DISCOVERYCOMMAND_H

#include "opengv_global.h"

#include "abstractcommand.h"

#include "ApplicationCommandCode.h"

class DiscoveryCommand : public AbstractCommand
{
public:
    DiscoveryCommand(int req_id);

    DiscoveryCommand(QHostAddress destinationAddress, quint16 destinationPort, int req_id);

    int getLengthWithoutHeader();

    char* getCommandDatagramWithoutHeader();
};

#endif // DISCOVERYCOMMAND_H
