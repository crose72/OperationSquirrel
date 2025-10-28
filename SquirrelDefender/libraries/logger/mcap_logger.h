/********************************************************************************
 * @file    mcap_logger.h
 * @author  Cameron Rose
 * @date    9/26/2025
 ********************************************************************************/
#ifndef MCAP_LOGGER_H
#define MCAP_LOGGER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <mcap/internal.hpp> // important to include this before write.hpp and writer.inl
#include <mcap/writer.hpp>
#include <mcap/types.hpp>
#include <fstream>
#include <unordered_map>

/********************************************************************************
 * Typedefs / Enums / Structs
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class MCAPLogger
{
public:
    MCAPLogger(const std::string &filename, const std::string &profile = "");
    ~MCAPLogger();

    // Register a protobuf schema and channel
    uint16_t addChannel(const std::string &schemaName, const std::string &protoSchemaText, const std::string &topic);

    // Log any protobuf message (as bytes)
    bool logMessage(const std::string &topic, const std::string &data, uint64_t timestamp);
    bool logMessage(const std::string &topic, const std::string &data, float timestamp);

    void close();

private:
    std::ofstream mOutfile;
    mcap::McapWriter mWriter;
    uint16_t mNextSchemaId = 1;
    uint16_t mNextChannelId = 1;
    uint32_t mSeq = 0;
    bool mClosed = false;

    // Map from topic string to channel ID
    std::unordered_map<std::string, uint16_t> mChannelMap;
};

#endif // MCAP_LOGGER_H