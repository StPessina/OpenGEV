#include "abstractmessagehandler.h"

AbstractMessageHandler::AbstractMessageHandler(GVComponent* target, int ackCode, QByteArray datagram,
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

int AbstractMessageHandler::readRequestCommandCode(QByteArray *datagram)
{
    int valueMSB = datagram->at(2);
    int valueLSB = datagram->at(3);
    return valueMSB*256+valueLSB;
}

int AbstractMessageHandler::readRequestRequestLength(QByteArray *datagram)
{
    int valueMSB = datagram->at(4);
    int valueLSB = datagram->at(5);
    return valueMSB*256+valueLSB;
}

int AbstractMessageHandler::readRequestRequestId(QByteArray *datagram)
{
    int valueMSB = datagram->at(6);
    int valueLSB = datagram->at(7);
    return valueMSB*256+valueLSB;
}

int AbstractMessageHandler::getRequestCommandCode()
{
    return requestCommandCode;
}

int AbstractMessageHandler::getRequestLength()
{
    return requestLength;
}

int AbstractMessageHandler::getRequestId()
{
    return reqId;
}

int AbstractMessageHandler::getResultStatus()
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
    if(datagramSize>8) {
        char* body = getAckDatagramWithoutHeader();
        for (int i = 8; i < datagramSize; ++i)
            datagramChar[i]=body[i-8];
    }
    QByteArray* datagram = new QByteArray(datagramChar, datagramSize);
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
