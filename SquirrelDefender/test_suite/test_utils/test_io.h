// io_map.h
#pragma once
#include <string>
#include <vector>

struct IOMap
{
    std::vector<std::string> inputs;
    std::vector<std::string> outputs;

    // Load from a JSON file path; throws std::runtime_error on error
    static IOMap load(const std::string &json_path);
};
