#ifndef ABSTRACTPIXELFORMAT_H
#define ABSTRACTPIXELFORMAT_H

#include <QByteArray>

class AbstractPixelFormat
{
public:
    AbstractPixelFormat(quint32 pixelFormat, quint32 bytePerPixel);

    virtual quint32 getBytePerPixel() final;

    virtual quint32 getPixelFormat() final;

    virtual char* getCharRapresentation() = 0;

private:

    quint32 pixelFormat;

    quint32 bytePerPixel;
};

#endif // ABSTRACTPIXELFORMAT_H
