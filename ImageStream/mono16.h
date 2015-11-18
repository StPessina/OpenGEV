#ifndef MONO16_H
#define MONO16_H

#include "CommonPacket/conversionutils.h"

#include "ImageStream/abstractpixelformat.h"
#include "ImageStream/pixelformat.h"

class Mono16 : public AbstractPixelFormat
{
public:
    Mono16(quint16 value);

    virtual char* getCharRapresentation();

private:
    quint16 value;
};

#endif // MONO16_H
