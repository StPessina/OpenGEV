#ifndef STREAMIMAGEDATAALLIN_H
#define STREAMIMAGEDATAALLIN_H

#include "CommonStream/abstractstreamdata.h"
#include "CommonStream/payloadtype.h"
#include "CommonStreamImageFormat/pixelformat.h"

#include "CommonStream/streamimagedataleader.h"
#include "CommonStream/streamimagedatatrailer.h"

/**
 * @brief The StreamImageDataAllIn class implements data leader packet
 * for image stream type (CR-288s)
 */
class StreamImageDataAllIn : public AbstractStreamData
{
public:
    StreamImageDataAllIn(QHostAddress destAddress, quint16 destPort,
                          quint64 blockId64,
                          quint32 pixelFormat, quint32 sizex, quint32 sizey,
                          quint32 offsetx, quint32 offsety,
                          quint16 paddingx, quint16 paddingy,
                          const QByteArray &data);

    virtual ~StreamImageDataAllIn();

protected:

    virtual quint16 getLengthWithoutHeader();

    virtual void appendPacketDatagramWithoutHeader(QByteArray &datagram);

private:

    quint8 fieldId = 0; //4 bits for interlaced data
    quint8 fieldCount = 0; //4 bits for interlaced data

    quint32 pixelFormat;

    quint32 sizex, sizey;
    quint32 offsetx, offsety;
    quint16 paddingx, paddingy;

    const QByteArray &data;
};


#endif // STREAMIMAGEDATAALLIN_H
