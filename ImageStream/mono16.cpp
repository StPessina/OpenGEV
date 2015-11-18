#include "mono16.h"

Mono16::Mono16(quint16 value)
    : AbstractPixelFormat(GVSP_PIX_MONO16, 16)
{
    this->value = value;
}

char *Mono16::getCharRapresentation()
{
    char* data = new char[2];
    ConversionUtils::setShortToCharArray(data, value, 0);
    return data;
}
