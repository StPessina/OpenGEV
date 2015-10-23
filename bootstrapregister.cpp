#include "bootstrapregister.h"

BootstrapRegister::BootstrapRegister(int address, std::string name, RegisterAccess accessType, int byteLength)
{
    this->address = address;
    this->name = name;
    this->accessType = accessType;
    this->length = byteLength;
    this->valueString="";

    switch (byteLength) {
    case 4:
        valueType=INT;
        break;
    case 8:
        valueType=LONG;
        break;
    default:
        valueType=STRING;
    }
}

BootstrapRegister::~BootstrapRegister()
{

}

int BootstrapRegister::getAddress()
{
    return address;
}

std::string BootstrapRegister::getName()
{
    return name;
}

RegisterAccess BootstrapRegister::getAccessType()
{
    return accessType;
}

int BootstrapRegister::getLength()
{
    return length;
}

bool BootstrapRegister::isStringValue()
{
    return valueType==STRING;
}

void BootstrapRegister::setValueString(std::string valueString)
{
    this->valueString = valueString;
}

std::string BootstrapRegister::getValueString()
{
    return valueString;
}

void BootstrapRegister::setValueNumb(int valueNumb)
{
    this->valueNumb = valueNumb;
}

void BootstrapRegister::setValueNumb(long valueNumb)
{
    this->valueNumb = valueNumb;
}

long BootstrapRegister::getValueNumb()
{
    return valueNumb;
}
