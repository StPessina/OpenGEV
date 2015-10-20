#ifndef BOOTSTRAPREGISTER_H
#define BOOTSTRAPREGISTER_H

#include <string>

#include "registeraccess.h"

class BootstrapRegister
{
public:
    BootstrapRegister(int address, std::string name, RegisterAccess accessType, int length);

    int getAddress();

    std::string getName();

    RegisterAccess getAccessType();

    int getLength();

    bool isStringValue();

    std::string setValueString(std::string valueString);

    std::string getValueString();

    void setValueNumb(int valueNumb);

    int getValueNumb();

private:
    int address;

    std::string name;

    RegisterAccess accessType;

    int length;

    bool stringValue;

    std::string valueString;

    int valueNumb;
};

#endif // BOOTSTRAPREGISTER_H
