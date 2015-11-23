#ifndef STREAMIMAGEDATALEADER_H
#define STREAMIMAGEDATALEADER_H

#include "CommonStream/abstractstreamdata.h"
#include "CommonStream/payloadtype.h"
#include "CommonStreamImageFormat/pixelformat.h"

/**
 * @brief The StreamRawDataLeader class implements data leader packet
 * for image stream type (CR-288s)
 */
class StreamImageDataLeader : public AbstractStreamData
{
public:
    StreamImageDataLeader(QHostAddress destAddress, quint16 destPort,
                          quint64 blockId64, quint32 packetId32,
                          quint32 pixelFormat, quint32 sizex, quint32 sizey,
                          quint32 offsetx, quint32 offsety,
                          quint16 paddingx, quint16 paddingy);

    virtual ~StreamImageDataLeader();

    virtual int executeAnswer(QByteArray answer);

protected:

    virtual quint16 getLengthWithoutHeader();

    virtual QByteArray getPacketDatagramWithoutHeader();

private:

    quint8 fieldId = 0; //4 bits for interlaced data
    quint8 fieldCount = 0; //4 bits for interlaced data

    quint32 pixelFormat;

    quint32 sizex, sizey;
    quint32 offsetx, offsety;
    quint16 paddingx, paddingy;
};

#endif // STREAMIMAGEDATALEADER_H
