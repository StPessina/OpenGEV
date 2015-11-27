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
    case DATA_ALLIN_FORMAT:
        return true;
        break;
    default:
        return false;
        break;
    }
}

AbstractStreamDataHandler *StreamImageDataHandlerFactory::createPacketHandler(quint16 handlerIdentifier,
                                                                               QByteArray datagram,
                                                                               QHostAddress senderAddress,
                                                                               quint16 senderPort)
{
    switch (handlerIdentifier) {
    case DATA_LEADER_FORMAT:
        return new StreamImageDataLeaderHandler(target,datagram,senderAddress,senderPort);
    case DATA_PAYLOAD_GENIRIC_FORMAT:
        return new StreamImageDataPayloadHandler(target,datagram, senderAddress, senderPort);
    case DATA_TRAILER_FORMAT:
        return new StreamImageDataTrailerHandler(target,datagram,senderAddress,senderPort);
    case DATA_ALLIN_FORMAT:
        return new StreamImageDataAllInHandler(target,datagram,senderAddress,senderPort);
    default:
        return NULL;
    }
}
