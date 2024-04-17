#include "Logger.h"

void Logger::log(const std::string &message)
{
    logMessage(LOG, message);
}

void Logger::warn(const std::string &message)
{
    logMessage(WARN, message);
}

void Logger::error(const std::string &message)
{
    logMessage(ERROR, message);
}

void Logger::logMessage(LogLevel level, const std::string &message)
{
    std::string levelStr;
    switch (level)
    {
    case LOG:
        levelStr = "LOG";
        break;
    case WARN:
        levelStr = "WARN";
        break;
    case ERROR:
        levelStr = "ERROR";
        break;
    }

    std::cout << "[" << levelStr << "] " << message << std::endl;
}