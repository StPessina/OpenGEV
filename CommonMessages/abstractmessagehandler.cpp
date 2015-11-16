#include "abstractmessagehandler.h"

AbstractMessageHandler::AbstractMessageHandler(GVComponent* target, quint16 ackCode, QByteArray datagram,
                                               QHostAddress senderAddress, quint16 senderPort)
{
    this->target=target;
    this->ackCode = ackCode;
    this->datagram = datagram;
    this->sender = senderAddress;
    this->port = senderPort;

    this->requestCommandCode = readRequestCommandCode(&datagram);
    this->requestLength = readRequestRequestLength(&datagram);
    this->reqId = readRequestRequestId(&datagram);
}

AbstractMessageHandler::~AbstractMessageHandler()
{

}

GVComponent* AbstractMessageHandler::getTarget()
{
    return target;
}

bool AbstractMessageHandler::isAckAllowed()
{
    return !ackNotAllowed;
}

quint16 AbstractMessageHandler::readRequestCommandCode(QByteArray *datagram)
{
    uint valueMSB = datagram->at(2);
    uint valueLSB = datagram->at(3);
    return valueMSB*256+valueLSB;
}

quint16 AbstractMessageHandler::readRequestRequestLength(QByteArray *datagram)
{
    uint valueMSB = datagram->at(4);
    uint valueLSB = datagram->at(5);
    return valueMSB*256+valueLSB;
}

quint16 AbstractMessageHandler::readRequestRequestId(QByteArray *datagram)
{
    uint valueMSB = datagram->at(6);
    uint valueLSB = datagram->at(7);
    return valueMSB*256+valueLSB;
}

quint16 AbstractMessageHandler::getRequestCommandCode()
{
    return requestCommandCode;
}

quint16 AbstractMessageHandler::getRequestLength()
{
    return requestLength;
}

quint16 AbstractMessageHandler::getRequestId()
{
    return reqId;
}

quint16 AbstractMessageHandler::getResultStatus()
{
    return resultStatus;
}

QByteArray *AbstractMessageHandler::getAckDatagram()
{
    int datagramSize = 8 + getAckDatagramLengthWithoutHeader(); //8 byte for header
    char* datagramChar = new char[datagramSize];
    char* header = getAckHeader();
    for (int i = 0; i < 8; ++i)
        datagramChar[i]=header[i];
    delete header;
    if(datagramSize>8) {
        char* body = getAckDatagramWithoutHeader();
        for (int i = 8; i < datagramSize; ++i)
            datagramChar[i]=body[i-8];
        delete body;
    }
    QByteArray* datagram = new QByteArray(datagramChar, datagramSize);
    delete datagramChar;
    return datagram;
}

std::string AbstractMessageHandler::toString()
{
    return sender.toString().toStdString() + ":" + std::to_string((int) port) + "/"
            + std::to_string(resultStatus) + "/"
            + std::to_string(ackCode) + "/"
            + std::to_string(getAckDatagramLengthWithoutHeader()) + "/"
            + std::to_string(reqId);
}

char *AbstractMessageHandler::getAckHeader()
{
    char* header = new char[8];

    int resultStatusShifted = resultStatus>>4;

    header[1]=resultStatusShifted >> 8;
    header[2]=resultStatusShifted;

    header[2]=ackCode >> 8;
    header[3]=ackCode;

    int length = getAckDatagramLengthWithoutHeader();
    header[4]=length >> 8;
    header[5]=length;

    header[6]=reqId >> 8;
    header[7]=reqId;

    return header;
}

bool AbstractMessageHandler::checkHeader()
{
    return true;
}
