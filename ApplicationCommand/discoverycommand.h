#ifndef DISCOVERYCOMMAND_H
#define DISCOVERYCOMMAND_H

#include "opengv_global.h"

#include "CommonMessages/abstractcommand.h"

#include "Application/gvapplication.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceMessageHandler/deviceackcode.h"

#include "CommonMessages/conversionutils.h"

using namespace std;

class DiscoveryCommand : public AbstractCommand
{
public:
    DiscoveryCommand(GVComponent* target);
    virtual ~DiscoveryCommand();

    DiscoveryCommand(GVComponent* target, QHostAddress destinationAddress, quint16 destinationPort);

    int getLengthWithoutHeader();

    char* getCommandDatagramWithoutHeader();

    int executeAnswer(QByteArray answer);

};

#endif // DISCOVERYCOMMAND_H
