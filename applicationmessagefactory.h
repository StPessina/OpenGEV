#ifndef APPLICATIONMESSAGEFACTORY_H
#define APPLICATIONMESSAGEFACTORY_H

#include "abstractmessagefactory.h"

#include "gvapplication.h"

class ApplicationMessageFactory : public AbstractMessageFactory
{
public:
    ApplicationMessageFactory(GVApplication* target);

    virtual bool isValidCode(int messageCode);

    virtual AbstractMessage createMessageHandler(int messageCode);
};

#endif // APPLICATIONMESSAGEFACTORY_H
