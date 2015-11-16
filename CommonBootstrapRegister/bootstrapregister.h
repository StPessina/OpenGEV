#ifndef BOOTSTRAPREGISTER_H
#define BOOTSTRAPREGISTER_H

#include <string>
#include <math.h>

#include "CommonBootstrapRegister/registeraccess.h"
#include "CommonBootstrapRegister/bootstrapregistertype.h"

using namespace std;

/**
 * @brief The BootstrapRegister class provide defition for a bootstrap register. Bootstrap register is used to store
 * information in GV components (like applications or devices)
 */
class BootstrapRegister
{
public:
    /**
     * @brief BootstrapRegister constructor
     * @param address id address
     * @param name
     * @param accessType allowed access from the GigE network
     * @param byteLength number of byte for the register
     */
    BootstrapRegister(int address, string name, RegisterAccess accessType, int byteLength);

    /**
     * @brief ~BootstrapRegister constructor
     */
    virtual ~BootstrapRegister();

    /**
     * @brief getAddress method
     * @return the id address of the register
     */
    int getAddress();

    /**
     * @brief getName method
     * @return the name of the register
     */
    string getName();

    /**
     * @brief getAccessType method
     * @return access type allowed for this register from the network
     */
    RegisterAccess getAccessType();

    /**
     * @brief getLength method
     * @return the length in bytes
     */
    int getLength();

    /**
     * @brief isStringValue method
     * @return true if the stored value is a string
     */
    bool isStringValue();

    /**
     * @brief setValueString method assign new string value for the register, if it's a string
     * @param valueString new value
     */
    void setValueString(string valueString);

    /**
     * @brief getValueString method
     * @return the string value for the register, if it's a string
     */
    string getValueString();

    /**
     * @brief setValueNumb method
     * @param valueNumb number value for the register, if it's a number
     */
    void setValueNumb(int valueNumb);

    /**
     * @brief setValueNumb method assign new number value for the register, if it's a number
     * @param valueNumb new value
     */
    void setValueNumb(long valueNumb);

    /**
     * @brief setBit method set bit value in a number value for the register, if it's a number
     * @param bitPosition from 0 to 31
     */
    void setBit(int bitPosition);

    /**
     * @brief resetBit reset bit value in a number value for the register, if it's a number
     * @param bitPosition from 0 to 31
     */
    void resetBit(int bitPosition);

    /**
     * @brief getMask
     * @param bitPosition to reset or set
     * @param direct true for reset and false for set
     * @return mask for set/reset bit
     */
    int getMask(int bitPosition, bool reset);

    /**
     * @brief getValueNumb method
     * @return the number value for this register, if it's a number
     */
    long getValueNumb();

private:
    /**
     * @brief address id of the register
     */
    int address;

    /**
     * @brief name of the register
     */
    string name;

    /**
     * @brief accessType allowed to the register from the network
     */
    RegisterAccess accessType;

    /**
     * @brief length in bytes
     */
    int length;

    /**
     * @brief valueType type of the value
     */
    BootstrapRegisterType valueType;

    /**
     * @brief valueString value string for the register
     */
    string valueString = "";

    /**
     * @brief valueNumb value number for the register
     */
    long valueNumb = 0;
};

#endif // BOOTSTRAPREGISTER_H
