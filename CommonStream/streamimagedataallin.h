#ifndef STREAMIMAGEDATAALLIN_H
#define STREAMIMAGEDATAALLIN_H

#include "CommonStream/abstractstreamdata.h"
#include "CommonStream/payloadtype.h"
#include "CommonStreamImageFormat/pixelformat.h"

#include "CommonStream/streamimagedataleader.h"
#include "CommonStream/streamimagedatatrailer.h"

/**
 * @brief The StreamImageDataAllIn class implements data leader packet
 * for image stream type (CR-288s). This kind of packet contains all the stream
 * data in the same packet.
 */
class StreamImageDataAllIn : public AbstractStreamData
{
public:

    /**
     * @brief StreamImageDataAllIn constructor
     * @param destAddress
     * @param destPort
     * @param blockId64
     * @param pixelFormat
     * @param sizex
     * @param sizey
     * @param offsetx
     * @param offsety
     * @param paddingx
     * @param paddingy
     * @param data
     */
    StreamImageDataAllIn(QHostAddress destAddress, quint16 destPort,
                          quint64 blockId64,
                          quint32 pixelFormat, quint32 sizex, quint32 sizey,
                          quint32 offsetx, quint32 offsety,
                          quint16 paddingx, quint16 paddingy,
                          const QByteArray &data);

    /**
     * @brief ~StreamImageDataAllIn deconstructor
     */
    virtual ~StreamImageDataAllIn();

protected:

    /**
     * @brief getLengthWithoutHeader
     * @return datagram length
     */
    virtual quint16 getLengthWithoutHeader();

    /**
     * @brief appendPacketDatagramWithoutHeader
     * @param datagram where data must be appended
     */
    virtual void appendPacketDatagramWithoutHeader(QByteArray &datagram);

private:

    /**
     * @brief fieldId
     */
    quint8 fieldId = 0; //4 bits for interlaced data

    /**
     * @brief fieldCount
     */
    quint8 fieldCount = 0; //4 bits for interlaced data

    /**
     * @brief pixelFormat
     */
    quint32 pixelFormat;

    quint32 sizex, sizey;
    quint32 offsetx, offsety;
    quint16 paddingx, paddingy;

    const QByteArray &data;
};


#endif // STREAMIMAGEDATAALLIN_H
