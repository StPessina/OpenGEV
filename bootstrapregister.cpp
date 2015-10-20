#include "bootstrapregister.h"

BootstrapRegister::BootstrapRegister(int address, std::string name, RegisterAccess accessType, int length)
{
    this->address = address;
    this->name = name;
    this->accessType = accessType;
    this->length = length;

    if(length==4)
        stringValue=false;
    else
        stringValue=true;
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
    return stringValue;
}

std::string BootstrapRegister::setValueString(std::string valueString)
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

int BootstrapRegister::getValueNumb()
{
    return valueNumb;
}
