#include "bootstrapregister.h"

BootstrapRegister::BootstrapRegister(int address, string name, RegisterAccess accessType, int byteLength)
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

string BootstrapRegister::getName()
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

void BootstrapRegister::setValueString(string valueString)
{
    this->valueString = valueString;
}

string BootstrapRegister::getValueString()
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
