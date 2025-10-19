// io_map.cpp
#include "test_io.h"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <stdexcept>

namespace
{
    std::vector<std::string> readStringArray(const Json::Value &root, const char *key)
    {
        if (!root.isMember(key) || !root[key].isArray())
        {
            throw std::runtime_error(std::string("io_map.json missing array '") + key + "'");
        }
        std::vector<std::string> out;
        out.reserve(root[key].size());
        for (const auto &v : root[key])
        {
            if (!v.isString())
            {
                throw std::runtime_error(std::string("io_map.json '") + key + "' must contain strings");
            }
            out.emplace_back(v.asString());
        }
        return out;
    }
}

IOMap IOMap::load(const std::string &json_path)
{
    std::ifstream in(json_path, std::ifstream::binary);
    if (!in)
    {
        throw std::runtime_error("Failed to open io map json: " + json_path);
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    std::string errs;
    if (!Json::parseFromStream(builder, in, &root, &errs))
    {
        throw std::runtime_error("Failed to parse io map json: " + errs);
    }

    IOMap io;
    io.inputs = readStringArray(root, "inputs");
    io.outputs = readStringArray(root, "outputs");
    return io;
}
