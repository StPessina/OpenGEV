#ifndef BOOTSTRAPREGISTER_H
#define BOOTSTRAPREGISTER_H

#include <string>

#include "CommonBootstrapRegister/registeraccess.h"
#include "CommonBootstrapRegister/bootstrapregistertype.h"

class BootstrapRegister
{
public:
    BootstrapRegister(int address, std::string name, RegisterAccess accessType, int byteLength);

    virtual ~BootstrapRegister();

    int getAddress();

    std::string getName();

    RegisterAccess getAccessType();

    int getLength();

    bool isStringValue();

    void setValueString(std::string valueString);

    std::string getValueString();

    void setValueNumb(int valueNumb);

    void setValueNumb(long valueNumb);

    long getValueNumb();

private:
    int address;

    std::string name;

    RegisterAccess accessType;

    int length;

    BootstrapRegisterType valueType;

    std::string valueString = "";

    long valueNumb = 0;
};

#endif // BOOTSTRAPREGISTER_H
