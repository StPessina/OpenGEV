#ifndef PIXELMAP_H
#define PIXELMAP_H

#include <boost/cstdint.hpp>

#include <stdint.h>
#include <QtGlobal>

#include "CommonStreamImageFormat/pixelformat.h"

#include "CommonPacket/conversionutils.h"

template<int size>
struct Pixel {

};


template<typename T>
struct PixelMap {

    PixelMap<T> *ptr;
    typedef PixelMap<T>* Ptr;

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

    void destroyPixelMap() {
        free(data);
    }

    void setNextPixel(char* pixel) {
        memcpy(&data[lastId],pixel,bytePerPixel);
        /*
        for (int i = 0; i < bytePerPixel; ++i)
            data[lastId+i]=pixel[i];
        */
        lastId+=bytePerPixel;
    }

    char* getImagePixelData() {
        return (char*) data;
    } 
};

/*
struct Mono16 {

};
*/

#endif // PIXELMAP_H
