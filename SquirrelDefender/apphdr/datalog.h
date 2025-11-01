#pragma once

/********************************************************************************
 * @file    datalog.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef DATALOG_H
#define DATALOG_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

#include "mcap_logger.h"
#include "Common.pb.h"
#include "Target.pb.h"
#include "Mavlink.pb.h"
#include "System.pb.h"
#include "Detection.pb.h"
#include "Path.pb.h"
#include "ImageAnnotations.pb.h"

#endif // logger options

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

class DataLogger
{
public:
    DataLogger();
    ~DataLogger();

    static bool init();
    static void loop();
    static void shutdown();
    static void log_data();

    void publishAnnotations(uint64_t ts_ns, const os::logger::Objects &objs);

    static inline void logTime(os::logger::Time *t, uint64_t ts_ns)
    {
        t->set_timestamp_ns(ts_ns);
        t->set_timestamp_sec(static_cast<double>(ts_ns) * 1e-9);
    }

    template <typename ProtoMsg>
    static bool publish(const std::string &topic, const ProtoMsg &msg, uint64_t timestamp)
    {
        if (!mMCAPLogger)
            return false; // not initialized yet
        std::string bytes;
        if (!msg.SerializeToString(&bytes))
            return false;

        std::lock_guard<std::mutex> lk(s_mtx); // safe if called from multiple threads
        return mMCAPLogger->logMessage(topic, bytes, timestamp);
    }

private:
    static std::unique_ptr<MCAPLogger> mMCAPLogger;
    static std::mutex s_mtx;
};

#else // default to old logger

class DataLogger
{
public:
    DataLogger();
    ~DataLogger();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // logger options

#endif // DATALOG_H
