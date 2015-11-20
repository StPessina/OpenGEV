#include "pixelsmap.h"

PixelsMap::PixelsMap(quint32 pixelFormat,
                     quint32 sizex, quint32 sizey,
                     quint32 offsetx, quint32 offsety,
                     quint16 paddingx, quint16 paddingy)
{
    this->pixelFormat = pixelFormat;
    this->sizex = sizex;
    this->sizey = sizey;
    this->offsetx = offsetx;
    this->offsety = offsety;
    this->paddingx = paddingx;
    this->paddingy = paddingy;

    pixelsSize = sizex*sizey;

    bytePerPixel = ((pixelFormat & 0x00FF0000) >> 16) / 8; //need byte so divied by 8

    pixels.reserve(pixelsSize);
}

PixelsMap::~PixelsMap()
{
    foreach (AbstractPixelFormat* pixel, pixels)
        delete pixel;

    pixels.clear();
}

quint32 PixelsMap::getPixelFormat()
{
    return pixelFormat;
}

void PixelsMap::addPixel(AbstractPixelFormat *pixel)
{
    pixels.push_back(pixel);
}

char *PixelsMap::getImagePixelData()
{
    if(pixels.size()==0)
        return new char[0];

    char* data = new char[getDataLength()];
    int pointer = 0;
    foreach (AbstractPixelFormat* pixel, pixels) {
        char* pixelData = pixel->getCharRapresentation();
        for (int i = 0; i < bytePerPixel; ++i) {
            data[pointer+i] = pixelData[i];
        }
        delete pixelData;
        pointer += bytePerPixel;
    }

    return data;
}

quint32 PixelsMap::getDataLength() {
    return pixels.size()*bytePerPixel;
}

quint32 PixelsMap::getDeclaredDataLength()
{
    return bytePerPixel*pixelsSize;
}

quint32 PixelsMap::getSizeX()
{
    return sizex;
}

quint32 PixelsMap::getSizeY()
{
    return sizey;
}

quint32 PixelsMap::getOffsetX()
{
    return offsetx;
}

quint32 PixelsMap::getOffsetY()
{
    return offsety;
}

quint16 PixelsMap::getPaddingX()
{
    return paddingx;
}

quint16 PixelsMap::getPaddingY()
{
    return paddingy;
}
