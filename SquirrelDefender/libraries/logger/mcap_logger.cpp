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
#include <iostream>

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

uint16_t MCAPLogger::addChannel(const std::string &topic,
                                const std::string &schemaName,
                                const std::string &encoding)
{
    std::string protoSchemaText = ""; // You could add your schema text if you want

    mcap::Schema schema{schemaName, encoding, protoSchemaText};
    mWriter.addSchema(schema);

    mcap::Channel channel{topic, encoding, schema.id, {}};
    channel.id = mNextChannelId++;
    mWriter.addChannel(channel);

    mChannelMap[topic] = channel.id;

    std::cout << "[MCAP] Added channel: " << topic << " id=" << channel.id << std::endl;
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
