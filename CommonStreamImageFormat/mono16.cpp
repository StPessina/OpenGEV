#include "mono16.h"

Mono16::Mono16(quint16 value)
    : AbstractPixelFormat(GVSP_PIX_MONO16, 16)
{
    this->value = value;
}

QByteArray Mono16::getCharRapresentation()
{
    QByteArray data;
    data.reserve(2);

    ConversionUtils::appendShortToQByteArray(&data,value);

    return data;
}
