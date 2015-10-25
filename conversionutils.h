#ifndef CONVERSIONUTILS_H
#define CONVERSIONUTILS_H

#include <QString>

class ConversionUtils
{
public:
    ConversionUtils();

    static QString getStringFromQByteArray(QByteArray array, int size, int start=0);

    static bool setShortToCharArray(char *array, short value, int start);

    static bool setIntToCharArray(char* array, int value, int start);

    static int getIntFromQByteArray(QByteArray array, int start=0);

private:
    static bool sanityCheck(int size, int requiredSize, int start);
};

#endif // CONVERSIONUTILS_H
