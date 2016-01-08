#include "streamimagedataallin.h"

StreamImageDataAllIn::StreamImageDataAllIn(QHostAddress destAddress,
                                             quint16 destPort, quint64 blockId64,
                                             quint32 pixelFormat,
                                             quint32 sizex, quint32 sizey,
                                             quint32 offsetx, quint32 offsety,
                                             quint16 paddingx, quint16 paddingy,
                                             const QByteArray &data)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_ALLIN_FORMAT, blockId64, 0),
      data(data)
{
    this->pixelFormat = pixelFormat;
    this->sizex = sizex;
    this->sizey = sizey;
    this->offsetx = offsetx;
    this->offsety = offsety;
    this->paddingx = paddingx;
    this->paddingy = paddingy;
}

StreamImageDataAllIn::~StreamImageDataAllIn()
{

}

quint16 StreamImageDataAllIn::getPacketBodyLength()
{
    return 36 //Leader body
            + 20 //Trailer body
            + data.size(); //stream data
}

void StreamImageDataAllIn::appendPacketBody(QByteArray &datagram)
{

    StreamImageDataLeader leaderBody(getDestinationAddress(),
                                     getDestionationPort(),
                                     getBlockId64(),
                                     getPacketId32(),
                                     pixelFormat,sizex,sizey,offsetx,offsety,paddingx,paddingy);
    leaderBody.appendPacketBody(datagram);

    StreamImageDataTrailer trailerBody(getDestinationAddress(),
                                       getDestionationPort(),
                                       getBlockId64(),
                                       getPacketId32(),
                                       sizey);
    trailerBody.appendPacketBody(datagram);

    datagram.append(data);
}
