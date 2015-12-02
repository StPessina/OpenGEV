#include "opengv.h"


OpenGV::OpenGV()
{
}

void OpenGV::configure()
{
#ifdef ENABLE_LOG4CPP
    initLog4cpp();
#endif
}

#ifdef ENABLE_LOG4CPP
void OpenGV::initLog4cpp()
{
    const char *file_log4cpp_init = "opengv_log4cpp.properties";
    try
    {
        log4cpp::PropertyConfigurator::configure( file_log4cpp_init );
    } catch( log4cpp::ConfigureFailure &e ) {
        std::cout
                << e.what()
                << " [log4cpp::ConfigureFailure catched] while reading "
                << file_log4cpp_init
                << std::endl;
        exit(1);
     }

}
#endif
