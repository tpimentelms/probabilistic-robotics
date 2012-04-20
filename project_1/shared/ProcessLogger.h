/**
 * @file      ProcessLogger.h
 * @author    George Andrew Brindeiro
 * @date      06/14/2010
 *
 * @attention Copyright (C) 2010
 * @attention Carnegie Mellon University
 */

#ifndef PROCESSLOGGER_H
#define PROCESSLOGGER_H


#include <boost/thread.hpp>
#include <iostream>


#define LOG(level) \
        if ((level > ProcessLogger::getLogLevel())&&(level > ProcessLogger::getPrintLevel())) ; \
        else ProcessLogger(__PRETTY_FUNCTION__).getLogStream(level)

#define LOG_INIT(log_dir,proc_name,log_level,print_level) \
        ProcessLogger::open(log_dir,proc_name);\
        ProcessLogger::setLogLevel(log_level);\
        ProcessLogger::setPrintLevel(print_level);

#define LOG_INIT2(filename,log_level,print_level) \
        ProcessLogger::open(filename);\
        ProcessLogger::setLogLevel(log_level);\
        ProcessLogger::setPrintLevel(print_level);
        
#define ENTER()		LOG(LEVEL_VERBOSE) << "Enter " << __PRETTY_FUNCTION__;
#define EXIT()     	LOG(LEVEL_VERBOSE) << "Exit " << __PRETTY_FUNCTION__;

enum LogLevel {LEVEL_FATAL, LEVEL_ERROR, LEVEL_WARN, LEVEL_INFO, LEVEL_DEBUG, LEVEL_VERBOSE};

/**
 * @brief Management of process logs
 *
 * ProcessLogger is the class responsible for all log output, be it a
 * simple terminal message to be printed or something that should be
 * written to a log file. Filters allow the user to decide how verbose
 * they want their log files or terminal output to be.
 *
 */
class ProcessLogger
{
    public:
        ProcessLogger() {}

        /**
         * @brief Constructor which should be used when tracing function using logger (debug/verbose level)
         */
        ProcessLogger(const std::string& callerFunction) : callerFunction_(callerFunction) {}

        /**
         * @brief Destructor writes log entry to file
         */
        ~ProcessLogger();

        /**
         * @brief Return stream to which log messages are written, before file write
         * @return String stream to which apply the insertion operator
         * @param level Filtering level to be applied to the message being written
         */
        std::ostringstream& getLogStream(LogLevel level = LEVEL_INFO);

        /**
         * @brief Find out version for process log, based on pre-existing process logs
         * @return String containing version number
         * @param logDir Directory storing logs
         * @param procName Current process name
         */
        static std::string getLogVersion(const std::string& logDir, const std::string& procName);

        /**
         * @brief Open a log for writing, using process-based filename
         * @return True if successful
         * @param logDir Directory storing logs
         * @param procName Current process name
         */
        static bool open(const std::string& logDir, const std::string& procName);

        /**
         * @brief Open a log for writing
         * @return True if successful
         * @param filename Filename to open
         */
        static bool open(const std::string& filename);

        /**
         * @brief Close existing log
         * @return True if successful
         */
        static bool close();

        static LogLevel& getLogLevel()                           { return logLevel_; }
        static void setLogLevel(const LogLevel& logLevel)        { logLevel_ = logLevel; }

        static LogLevel& getPrintLevel()                         { return printLevel_; }
        static void setPrintLevel(const LogLevel& printLevel)    { printLevel_ = printLevel; }

    private:
        ProcessLogger(const ProcessLogger&);
        ProcessLogger& operator=(const ProcessLogger&);

        static LogLevel logLevel_;      // Max level to log to file
        static LogLevel printLevel_;    // Max level to print to terminal
        LogLevel messageLevel_;         // Input message level, defined for each record logged

        std::ostringstream logStream_;  // Stream where information will be written before logging to file or terminal

        static FILE* logFile_;          // File where log will be written to
        boost::mutex logFileMutex_;     // Mutex for file write

        std::string logPrefix_;         // Information (time, level) that will be prefix to the log message
        std::string callerFunction_;    // Caller function, for debugging purposes

        /**
         * @brief Convert from a LogLevel value to the corresponding string
         * @return String correspondent to LogLevel value
         * @param level LogLevel value to be converted
         */
        static inline std::string ToString(LogLevel level)
        {
            switch(level)
            {
                case LEVEL_FATAL:       { return "LEVEL_FATAL"; }
                case LEVEL_ERROR:       { return "LEVEL_ERROR"; }
                case LEVEL_WARN:        { return "LEVEL_WARN"; }
                case LEVEL_INFO:        { return "LEVEL_INFO"; }
                case LEVEL_DEBUG:       { return "LEVEL_DEBUG"; }
                case LEVEL_VERBOSE:     { return "LEVEL_VERBOSE"; }
            }

            std::cerr << "Could not parse level to string. Refer to " << __PRETTY_FUNCTION__ << std::endl;
            return "LEVEL_UNKNOWN";
        }

        static inline const char* msgColor(LogLevel level)
        {
            switch(level)
            {
                case LEVEL_FATAL:       { return "\033[31m\033[1m"; }  	//red
                case LEVEL_ERROR:       { return "\033[33m\033[1m"; }  	//orange
                case LEVEL_WARN:        { return "\033[32m\033[1m"; }  	//green
                case LEVEL_INFO:        { return "\033[0;0m\033[1m"; }  //standard
                case LEVEL_DEBUG:       { return "\033[36m\033[1m"; }  	//cyan
                case LEVEL_VERBOSE:     { return "\033[34m\033[1m"; }  	//blue
                default:                { return "\033[35m\033[1m"; }  	//white
            }
        }

};

#endif // PROCESSLOGGER_H
