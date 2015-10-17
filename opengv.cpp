#include "opengv.h"


OpenGV::OpenGV()
{
}

void OpenGV::configure()
{
    initLog4cpp();
}

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
