#ifndef PIXELMAP_H
#define PIXELMAP_H

#include <boost/cstdint.hpp>

#include <stdint.h>
#include <QtGlobal>

#include "CommonStreamImageFormat/pixelformat.h"

#include "CommonPacket/conversionutils.h"

typedef struct {
    boost::uint32_t pixelFormat;
    boost::uint64_t value;
}Pixel;

template<typename T = Pixel>
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

    T *data;
    char* datagram;

    quint32 lastId = -1;

    PixelMap(quint32 pixelFormat, quint32 sizex, quint32 sizey,
             quint32 offsetx, quint32 offsety,
             quint16 paddingx, quint16 paddingy) :
        pixelFormat(pixelFormat),
        sizex(sizex), sizey(sizey),
        offsetx(offsetx), offsety(offsety),
        paddingx(paddingx), paddingy(paddingy)
    {
        //pixelFormat = T.pixelFormat;

        declaredPixelsSize = sizex*sizey;

        bytePerPixel = ((pixelFormat & 0x00FF0000) >> 16) / 8; //need byte so divied by 8

        data = (T*) malloc(declaredPixelsSize*sizeof(T));
        datagram = (char*) malloc(declaredPixelsSize*bytePerPixel*sizeof(char));
    }

    void destroyPixelMap() {
        free(data);
        free(datagram);
    }

    void setNextPixel(T pixel) {
        lastId++;
        data[lastId]=pixel;

        switch (bytePerPixel) {
        case 2:
            ConversionUtils::setShortToCharArray(datagram,pixel.value, lastId*bytePerPixel);
            break;
        default:
            break;
        }

        pixelMapSize++;
        dataLength+=bytePerPixel;
    }

    const char* getImagePixelData() {
        return datagram;
    } 
};

/*
struct Mono16 : public Pixel {

    Mono16(boost::uint16_t value) :
        value(value), format(GVSP_PIX_MONO16) {}

};
*/

#endif // PIXELMAP_H
