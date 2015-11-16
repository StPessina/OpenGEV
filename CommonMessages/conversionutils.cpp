#include "conversionutils.h"

ConversionUtils::ConversionUtils()
{
}

bool ConversionUtils::setShortToCharArray(char *array, short value, int start)
{
    array[start] = (value >> 8) & 0xFF;
    array[start+1] = value;
    return true;
}

bool ConversionUtils::setIntToCharArray(char *array, int value, int start)
{
    array[start] = (value >> 24) & 0xFF;
    array[start+1] = (value >> 16) & 0xFF;
    array[start+2] = (value >> 8) & 0xFF;
    array[start+3] = value;
    return true;
}

bool ConversionUtils::sanityCheck(int size, int requiredSize, int start)
{
    if(requiredSize<=0 || start<0) return false;
    if(size<=(start+requiredSize)) return false;
    return true;
}

int ConversionUtils::getIntFromQByteArray(QByteArray array, int start)
{
    if(!sanityCheck(array.size(), 4, start)) return 0;

    int valueMSB = array.at(start) & 0xFF;
    int valueB = array.at(start+1) & 0xFF;
    int valueC = array.at(start+2) & 0xFF;
    int valueLSB = array.at(start+3) & 0xFF;

    return valueLSB | (valueC << 8) | (valueB << 16) | (valueMSB << 24);
}

short ConversionUtils::getShortFromQByteArray(QByteArray array, int start)
{
    if(!sanityCheck(array.size(),2,start)) return 0;

    short valueMSB = array.at(start) & 0xFF;
    short valueLSB = array.at(start+1) & 0xFF;

    return valueLSB | (valueMSB << 8);
}

QString ConversionUtils::getStringFromQByteArray(QByteArray array, int size, int start)
{
    if(!sanityCheck(array.size(), size, start)) return QString("");

    int realSize = 0;

    for (int i = 0; i < size; ++i)
        if(array.at(i+start)!=0 || array.at(i+start)!='\n')
            realSize++;

    char* charString = new char[realSize];

    for (int i = 0; i < realSize; ++i)
        if(array.at(i+start)!=0 || array.at(i+start)!='\n')
            charString[i]=array.at(i+start);

    return QString(charString);
}
