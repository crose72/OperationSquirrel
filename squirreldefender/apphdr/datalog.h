#pragma once

/********************************************************************************
 * @file    datalog.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Data logging interface for MCAP-based or legacy logging backends.
 *
 *          This header defines the DataLogger class, which provides a unified
 *          logging API across platforms. On Jetson Orin Nano and WSL builds,
 *          MCAP is used for structured protobuf logging. Otherwise, the legacy
 *          logger is used.
 ********************************************************************************/
#ifndef DATALOG_H
#define DATALOG_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#if defined(BLD_JETSON_ORIN) || defined(BLD_WSL)

#include "mcap_logger.h"
#include "Common.pb.h"
#include "Target.pb.h"
#include "Mavlink.pb.h"
#include "System.pb.h"
#include "Detection.pb.h"
#include "Control.pb.h"
#include "ImageAnnotations.pb.h"

#endif // logger options

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

#if defined(BLD_JETSON_ORIN) || defined(BLD_WSL)

class DataLogger
{
public:
    DataLogger();
    ~DataLogger();

    static bool init();
    static void loop();
    static void shutdown();
    static void log_data();

    void publish_annotations(uint64_t ts_ns, const os::logger::Objects &objs);

    static inline void logTime(os::logger::Time *t, uint64_t ts_ns)
    {
        t->set_timestamp_ns(ts_ns);
        t->set_timestamp_sec(static_cast<double>(ts_ns) * 1e-9);
    }

    template <typename ProtoMsg>
    static bool publish(const std::string &topic, const ProtoMsg &msg, uint64_t timestamp)
    {
        if (!mcap_logger)
            return false; // not initialized yet
        std::string bytes;
        if (!msg.SerializeToString(&bytes))
            return false;

        std::lock_guard<std::mutex> lk(s_mtx); // safe if called from multiple threads
        return mcap_logger->logMessage(topic, bytes, timestamp);
    }

private:
    static std::unique_ptr<MCAPLogger> mcap_logger;
    static std::mutex s_mtx;
};

#else // Legacy logger fallback

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
