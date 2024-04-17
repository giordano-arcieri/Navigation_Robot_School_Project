#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>

namespace Logger
{
    enum LogLevel
    {
        LOG,
        WARN,
        ERROR
    };

    void log(const std::string &message);
    void warn(const std::string &message);
    void error(const std::string &message);
    void logMessage(LogLevel level, const std::string &message);
}

#endif // LOGGER_H