/**
 * @file      ProcessLogger.cpp
 * @author    George Andrew Brindeiro
 * @date      06/14/2010
 *
 * @attention Copyright (C) 2010
 * @attention Carnegie Mellon University
 */

#include <ProcessLogger.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <sys/time.h>


using namespace std;
namespace bfs = boost::filesystem;

LogLevel ProcessLogger::logLevel_ = LEVEL_VERBOSE;
LogLevel ProcessLogger::printLevel_ = LEVEL_INFO;

FILE* ProcessLogger::logFile_;

ProcessLogger::~ProcessLogger()
{
    string logMsg = logStream_.str();
    string logPostfix("");

    if((messageLevel_ == LEVEL_FATAL) || (messageLevel_ == LEVEL_ERROR))
    {
        logPostfix += "\n\t\t\t\t\t\tIn function:\t";
        logPostfix += callerFunction_;
    }

    if(logFile_ != NULL)
        if(messageLevel_ <= logLevel_)
        {
            boost::lock_guard<boost::mutex> lock(logFileMutex_);
            fprintf(logFile_, "%s\t%s%s\n", logPrefix_.c_str(),
                                            logStream_.str().c_str(),
                                            logPostfix.c_str());
        }

    if(messageLevel_ <= printLevel_)
    {
        fprintf(stderr, "%s%s\t%s%s\033[0;0m\n",
                                      msgColor(messageLevel_),
                                      logPrefix_.c_str(),
                                      logStream_.str().c_str(),
                                      logPostfix.c_str());
    }

    fflush(stderr);
}

ostringstream& ProcessLogger::getLogStream(LogLevel level)
{
    struct timeval tv;
    time_t nowtime;
    struct tm *nowtm;
    char tmbuf[64], timestamp[64];

    gettimeofday(&tv, NULL);
    nowtime = tv.tv_sec;
    nowtm = localtime(&nowtime);
    strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
    snprintf(timestamp, sizeof timestamp, "%s.%06ld", tmbuf, tv.tv_usec);

    //logPrefix_ = msgColor(messageLevel_);
    logPrefix_ = timestamp;
    logPrefix_ += "\t";
    logPrefix_ += ToString(level);

    messageLevel_ = level;

    return logStream_;
}

string ProcessLogger::getLogVersion(const string& logDir, const string& procName)
{
    int versionInt = 0;
    string version("");

    // Look for process logs for procName within logDir, figure out latest version
    if(bfs::exists(logDir))
    {
        bfs::directory_iterator end;
        for(bfs::directory_iterator itr(logDir); itr != end; ++itr)
        {
            if(bfs::is_directory(itr->status()))
                continue;

            const boost::regex processLogFilter(procName+"-([0-9]+)\\.process\\.log");
            boost::smatch processLogMatch;

            if(!boost::regex_match(itr->filename(), processLogMatch, processLogFilter))
                continue;

            LOG(LEVEL_DEBUG) << "process log found: " << itr->filename();

            try
            {
                //processLogMatch[0] = process log complete filename
                //processLogMatch[1] = marked subexpression for version number
                versionInt =  std::max(versionInt, boost::lexical_cast<int>(processLogMatch[1]));
            }
            catch(const boost::bad_lexical_cast &e)
            {
                // Could not perform cast, return invalid version zero
                LOG(LEVEL_ERROR) << "Failed to convert process log version number!";
                return "0";
            }

        }

        try
        {
            version = boost::lexical_cast<std::string>(versionInt+1);
        } catch(const boost::bad_lexical_cast &e) {
            // Could not perform cast, return invalid version zero
            LOG(LEVEL_ERROR) << "Failed to convert process log version number!";
            return "0";
        }

        // Return extracted version
        return version;
    }
    else
    {
        // Log directory doesn't exist, return invalid version zero
        LOG(LEVEL_ERROR) << "logDir " << logDir << " doesn't exist in ProcessLogger::getLogVersion";
        return "0";
    }
}

bool ProcessLogger::open(const string& logDir, const string& procName)
{
    string version = getLogVersion(logDir,procName);
    logFile_ = fopen((logDir + "/" + procName + "-" + version + ".process.log").c_str(),"w");

    if(logFile_ != NULL)
        return true;
    else
    {
        LOG(LEVEL_ERROR) << "Error: Could not open ProcessLogger file "
                         << (logDir + "/" + procName + "-" + version + ".process.log") << endl;
        return false;
    }
}

bool ProcessLogger::open(const string& filename)
{
    logFile_ = fopen(filename.c_str(),"w");

    if(logFile_ != NULL)
        return true;
    else
    {
        LOG(LEVEL_ERROR) << "Error: Could not open ProcessLogger file " << filename << endl;
        return false;
    }
}

bool ProcessLogger::close()
{
    int fileNotClosed = fclose(logFile_);

    logFile_ = NULL;

    if(fileNotClosed)
    {
        LOG(LEVEL_ERROR) << "Error: Could not close ProcessLogger file" << endl;
        return false;
    }
    else
        return true;
}
