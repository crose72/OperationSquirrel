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

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

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

uint16_t MCAPLogger::addChannel(const std::string &schemaName,
                                const std::string &protoSchemaText,
                                const std::string &topic)
{
    mcap::Schema schema{schemaName, "protobuf", protoSchemaText};
    mWriter.addSchema(schema);
    // If your MCAP version throws on error, this is fine.

    mcap::Channel channel{topic, "protobuf", schema.id, {}};
    channel.id = mNextChannelId++;
    mWriter.addChannel(channel);
    return channel.id;
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

void MCAPLogger::close()
{
    if (!mClosed)
    {
        mWriter.close();
        mOutfile.close();
        mClosed = true;
    }
}
