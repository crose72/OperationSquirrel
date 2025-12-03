/********************************************************************************
 * @file    mcap_logger.cpp
 * @author  Cameron Rose
 * @date    9/26/2025
 * @brief   A C++ class to enable logging into MCAP files.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mcap_logger.h"
#include <vector>
#include <cstddef> // std::byte
#include <cstring> // std::memcpy
#include <spdlog/spdlog.h>

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

static std::vector<std::byte> readAllBytes(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return {};

    // Slurp into a temporary char buffer
    std::vector<char> tmp((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());

    // Copy into std::byte vector
    std::vector<std::byte> out(tmp.size());
    if (!tmp.empty())
        std::memcpy(out.data(), tmp.data(), tmp.size());
    return out;
}

/********************************************************************************
 * Function:
 * Description:
 ********************************************************************************/
MCAPLogger::MCAPLogger(const std::string &filename, const std::string &profile)
    : mOutfile(filename, std::ios::binary | std::ios::out | std::ios::trunc)
{
    mcap::McapWriterOptions opts(profile);
    mWriter.open(mOutfile, opts);
}

MCAPLogger::~MCAPLogger()
{
    close();
}

uint16_t MCAPLogger::addChannel(const std::string &topic,
                                const std::string &schemaName,
                                const std::string & /*encoding_ignored*/)
{
    // Load the FileDescriptorSet that CMake generated.
    const std::string descPath =
#ifdef SCHEMA_DESC_PATH
        SCHEMA_DESC_PATH;
#else
        "schemas.desc"; // fallback if not defined by CMake
#endif

    const auto descBytes = readAllBytes(descPath);
    if (descBytes.empty())
    {
        spdlog::error("[MCAP] ERROR: Could not read descriptor set at {} — Foxglove will show 'no such type: {}",
                      descPath, schemaName);
        return 0;
    }

    // Add schema: name must be "package.Message", encoding must be "protobuf".
    mcap::Schema schema;
    schema.name = schemaName; // e.g., "logger.TargetInfo"
    schema.encoding = "protobuf";
    schema.data = descBytes;

    mWriter.addSchema(schema); // void; sets schema.id internally

    // Create channel that references the schema. You must assign channel.id.
    mcap::Channel ch;
    ch.id = mNextChannelId++; // your class already has this counter
    ch.topic = topic;         // e.g., "/target_info"
    ch.messageEncoding = "protobuf";
    ch.schemaId = schema.id; // use the id set by addSchema()

    mWriter.addChannel(ch); // void

    mChannelMap[topic] = static_cast<uint16_t>(ch.id);

    spdlog::info("[MCAP] Added channel: {} schema={} chId={}",
                 topic, schemaName, ch.id);

    return static_cast<uint16_t>(ch.id);
}

bool MCAPLogger::logMessage(const std::string &topic, const std::string &data, uint64_t timestamp)
{
    auto it = mChannelMap.find(topic);
    if (it == mChannelMap.end())
        return false;

    mcap::Message msg;
    msg.channelId = it->second;
    msg.sequence = mSeq++;
    msg.logTime = timestamp;
    msg.publishTime = timestamp;
    msg.data = reinterpret_cast<const std::byte *>(data.data());
    msg.dataSize = static_cast<uint64_t>(data.size());

    auto s = mWriter.write(msg);
    return s.ok();
}

bool MCAPLogger::logMessage(const std::string &topic, const std::string &data, float timestamp)
{
    auto it = mChannelMap.find(topic);
    if (it == mChannelMap.end())
        return false;

    mcap::Message msg;
    msg.channelId = it->second;
    msg.sequence = mSeq++;
    msg.logTime = timestamp;
    msg.publishTime = timestamp;
    msg.data = reinterpret_cast<const std::byte *>(data.data());
    msg.dataSize = static_cast<uint64_t>(data.size());

    auto s = mWriter.write(msg);
    return s.ok();
}

void MCAPLogger::close()
{
    if (!mClosed)
    {
        mWriter.close();
        mOutfile.close();
        mClosed = true;
    }
}
