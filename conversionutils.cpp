#include "conversionutils.h"

ConversionUtils::ConversionUtils()
{
}

bool ConversionUtils::setIntToCharArray(char *array, int value, int start)
{
    return false;
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

    int valueMSB = array.at(start);
    int valueB = array.at(start+1);
    int valueC = array.at(start+2);
    int valueLSB = array.at(start+3);

    return valueLSB | (valueB << 8) | (valueC << 16) | (valueMSB << 24);
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
