#ifndef PIXELSMAP_H
#define PIXELSMAP_H

#include <stdint.h>
#include <QByteArray>

#include <vector>

#include "CommonStreamImageFormat/abstractpixelformat.h"

class PixelsMap
{
public:
    PixelsMap(quint32 pixelFormat, quint32 sizex, quint32 sizey,
              quint32 offsetx, quint32 offsety,
              quint16 paddingx, quint16 paddingy);

    virtual ~PixelsMap();

    virtual quint32 getPixelFormat() final;

    virtual void addPixel(AbstractPixelFormat* pixel);

    virtual QByteArray getImagePixelData();

    virtual quint32 getSizeInPixel();

    virtual quint32 getDeclaredSizeInPixel() final;

    virtual quint32 getDataLength() final;

    virtual quint32 getDeclaredDataLength() final;

    virtual quint32 getSizeX() final;

    virtual quint32 getSizeY() final;

    virtual quint32 getOffsetX() final;

    virtual quint32 getOffsetY() final;

    virtual quint16 getPaddingX() final;

    virtual quint16 getPaddingY() final;

private:
    std::vector<AbstractPixelFormat*> pixels;

    quint32 pixelFormat;

    quint32 pixelsSize;

    quint32 bytePerPixel;

    quint32 sizex, sizey;
    quint32 offsetx, offsety;
    quint16 paddingx, paddingy;
};

#endif // PIXELSMAP_H
