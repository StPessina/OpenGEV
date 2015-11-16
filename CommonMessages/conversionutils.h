#ifndef CONVERSIONUTILS_H
#define CONVERSIONUTILS_H

#include <QString>

/**
 * @brief The ConversionUtils class
 */
class ConversionUtils
{
public:
    /**
     * @brief ConversionUtils
     */
    ConversionUtils();

    /**
     * @brief getStringFromQByteArray read string from QByteArray
     * @param array
     * @param size
     * @param start
     * @return string value if no out of bound
     */
    static QString getStringFromQByteArray(QByteArray array, int size, int start=0);

    /**
     * @brief setShortToCharArray write short value in char array
     * @param array
     * @param value
     * @param start
     * @return true if the value is set
     */
    static bool setShortToCharArray(char *array, short value, int start);

    /**
     * @brief setIntToCharArray write integer value from a char array
     * @param array
     * @param value
     * @param start
     * @return true if no out of bound
     */
    static bool setIntToCharArray(char* array, int value, int start);

    /**
     * @brief getIntFromQByteArray read integer value from a QByteArray
     * @param array
     * @param start
     * @return the integer value if no out of bound
     */
    static int getIntFromQByteArray(QByteArray array, int start=0);

    /**
     * @brief getIntFromQByteArray read integer value from a QByteArray
     * @param array
     * @param start
     * @return the unsigned short value if no out of bound
     */
    static short getShortFromQByteArray(QByteArray array, int start=0);


private:
    /**
     * @brief sanityCheck check if a it's possible read/write value on a array
     * @param size of the array maximum size of the data structure
     * @param requiredSize value size
     * @param start position where the value is read/write in the array
     * @return true if the space is enaugh
     */
    static bool sanityCheck(int size, int requiredSize, int start);
};

#endif // CONVERSIONUTILS_H
