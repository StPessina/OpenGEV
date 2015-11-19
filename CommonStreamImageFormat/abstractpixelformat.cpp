#include "abstractpixelformat.h"

AbstractPixelFormat::AbstractPixelFormat(quint32 pixelFormat, quint32 bytePerPixel)
{
    this->bytePerPixel = bytePerPixel;
    this->pixelFormat = pixelFormat;
}

quint32 AbstractPixelFormat::getBytePerPixel()
{
    return bytePerPixel;
}

quint32 AbstractPixelFormat::getPixelFormat()
{
    return pixelFormat;
}
