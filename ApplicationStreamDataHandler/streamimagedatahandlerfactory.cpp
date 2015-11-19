#include "streamimagedatahandlerfactory.h"

StreamImageDataHandlerFactory::StreamImageDataHandlerFactory(GVComponent* const target)
    : AbstractPacketHandlerFactory(target)
{
}

StreamImageDataHandlerFactory::~StreamImageDataHandlerFactory()
{

}

int StreamImageDataHandlerFactory::getPacketHandlerIdentifier(QByteArray datagram)
{
    return AbstractStreamDataHandler::readRequestPacketFormat(&datagram);
}

bool StreamImageDataHandlerFactory::isValidCode(quint16 handlerIdentifier)
{
    switch (handlerIdentifier) {
    case DATA_LEADER_FORMAT:
    case DATA_PAYLOAD_GENIRIC_FORMAT:
    case DATA_TRAILER_FORMAT:
        break;
    default:
        return false;
        break;
    }
    return true;
}

AbstractStreamDataHandler *StreamImageDataHandlerFactory::createPacketHandler(quint16 handlerIdentifier,
                                                                               QByteArray datagram,
                                                                               QHostAddress senderAddress,
                                                                               quint16 senderPort)
{
    switch (handlerIdentifier) {
    case DATA_LEADER_FORMAT:
        return new StreamImageDataLeaderHandler(target,datagram,senderAddress,senderPort);
    //case DATA_PAYLOAD_GENIRIC_FORMAT:
    //case DATA_TRAILER_FORMAT:
    //    break;
    default:
        return NULL;
    }
}
