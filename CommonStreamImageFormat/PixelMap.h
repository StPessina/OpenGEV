#ifndef PIXELMAP_H
#define PIXELMAP_H

#include <boost/cstdint.hpp>

#include <stdint.h>
#include <QtGlobal>

#include "CommonStreamImageFormat/pixelformat.h"

#include "CommonPacket/conversionutils.h"

struct PixelMap {

    typedef PixelMap* Ptr;
    PixelMap* prt;

    quint32 pixelFormat;

    quint32 declaredPixelsSize;
    quint32 pixelMapSize = 0;

    quint32 dataLength = 0;

    quint32 bytePerPixel;

    quint32 sizex, sizey;
    quint32 offsetx, offsety;
    quint16 paddingx, paddingy;

    char* data;

    quint32 lastId = 0;

    PixelMap() {}

    PixelMap(quint32 pixelFormat, quint32 sizex, quint32 sizey,
             quint32 offsetx, quint32 offsety,
             quint16 paddingx, quint16 paddingy) :
        pixelFormat(pixelFormat),
        sizex(sizex), sizey(sizey),
        offsetx(offsetx), offsety(offsety),
        paddingx(paddingx), paddingy(paddingy)
    {
        declaredPixelsSize = sizex*sizey;
        pixelMapSize=declaredPixelsSize;

        bytePerPixel = ((pixelFormat & 0x00FF0000) >> 16) / 8; //need byte so divied by 8

        data = (char*) malloc(declaredPixelsSize*bytePerPixel*sizeof(char));
        dataLength = declaredPixelsSize*bytePerPixel;
    }

    void renew(quint32 pixelFormat, quint32 sizex, quint32 sizey,
          quint32 offsetx, quint32 offsety,
          quint16 paddingx, quint16 paddingy)
    {
        declaredPixelsSize = sizex*sizey;
        pixelMapSize=declaredPixelsSize;

        bytePerPixel = ((pixelFormat & 0x00FF0000) >> 16) / 8; //need byte so divied by 8

        if(sizex!=this->sizex ||
                this->sizey!=sizey ||
                this->pixelFormat!=pixelFormat) {
            if(data!=NULL)
                destroyPixelMap();
            data = (char*) malloc(declaredPixelsSize*bytePerPixel*sizeof(char));
            dataLength = declaredPixelsSize*bytePerPixel;
        }

        this->pixelFormat=pixelFormat;
        this->sizex = sizex;
        this->sizey = sizey;
        this->offsetx = offsetx;
        this->offsety = offsety;
        this->paddingx = paddingx;
        this->paddingy = paddingy;
    }

    void destroyPixelMap() {
        free(data);
    }

    const void* getImagePixelData() {
        return data;
    } 
};

template<int gvspFormat>
struct Pixel {
    char* data;
};

template<int gvspFormat>
void setNextPixel(PixelMap::Ptr map, Pixel<gvspFormat> &pixel) {
    memcpy(&(map->data)[map->lastId],pixel.data,map->bytePerPixel);
    map->lastId+=map->bytePerPixel;
}

template<int gvspFormat>
const Pixel<gvspFormat> getPixel(PixelMap::Ptr map, quint32 position) {
    Pixel<gvspFormat> aPixel;
    aPixel.data = &(map->data)[position];
    return aPixel;
}

struct Mono16 : Pixel<GVSP_PIX_MONO16> {

    Mono16(quint16 value) : Pixel() {
        ConversionUtils::setShortToCharArray(data, value, 0);
    }
};

typedef Pixel<3> RGB24;

typedef Pixel<5> Mono40;

/*
struct Mono16 {

};
*/

#endif // PIXELMAP_H
