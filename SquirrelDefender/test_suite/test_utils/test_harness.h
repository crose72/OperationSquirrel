/********************************************************************************
 * @file    test_harness.h
 * @author  Cameron Rose
 * @date    10/19/2025
 ********************************************************************************/
#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <vector>
#include <string>
#include <ostream>
#include <type_traits>
#include "csv_reader.h"
#include <memory>
#include <fstream>
#include <iostream>

/********************************************************************************
 * Function prototypes
 ********************************************************************************/

// Minimal, header-only harness.
// Usage from your test file:
//   1) Define INPUT_VARS(X) and OUTPUT_VARS(X)
//   2) #include "test_utils/test_harness.h"
//   3) Create TestHarness g_h; g_h.attachCSV(*g_csv);
//   4) INPUT_VARS(TESTHARNESS_REGISTER_INPUT);
//      OUTPUT_VARS(TESTHARNESS_REGISTER_OUTPUT);
//   5) g_h.validateInputs(); g_h.writeHeader(out); g_h.setInputs(row); g_h.writeRow(out);

class TestHarness
{
public:
    void attachCSV(CSVReader &r) { csv_ = &r; }

    // Register an input (set from CSV + written)
    template <typename T>
    void regInput(const char *name, T &var)
    {
        input_names_.emplace_back(name);
        if (seen_.insert(name).second)
            header_names_.emplace_back(name);

        // setter (typed)
        setters_[name] = [this, name, &var](size_t row)
        {
            if (!csv_)
                throw std::runtime_error("CSVReader not attached");
            if constexpr (std::is_same_v<T, int16_t>)
            {
                var = static_cast<int16_t>((int)(*csv_)(name, row));
            }
            else if constexpr (std::is_same_v<T, bool>)
            {
                var = (bool)(*csv_)(name, row); // accepts 0/1/true/false
            }
            else if constexpr (std::is_enum_v<T>)
            {
                using U = std::underlying_type_t<T>;
                var = static_cast<T>((U)(*csv_)(name, row));
            }
            else
            {
                var = (T)(*csv_)(name, row);
            }
        };

        // writer (typed)
        writers_[name] = [&var](std::ostream &os)
        {
            if constexpr (std::is_same_v<T, bool>)
            {
                os << (var ? 1 : 0);
            }
            else if constexpr (std::is_same_v<T, int16_t>)
            {
                os << static_cast<int>(var);
            }
            else if constexpr (std::is_enum_v<T>)
            {
                os << static_cast<std::underlying_type_t<T>>(var);
            }
            else
            {
                os << var;
            }
        };
    }

    // Register an output-only (written, NOT set)
    template <typename T>
    void regOutput(const char *name, T &var)
    {
        if (seen_.insert(name).second)
            header_names_.emplace_back(name);
        writers_[name] = [&var](std::ostream &os)
        {
            if constexpr (std::is_same_v<T, bool>)
            {
                os << (var ? 1 : 0);
            }
            else if constexpr (std::is_same_v<T, int16_t>)
            {
                os << static_cast<int>(var);
            }
            else if constexpr (std::is_enum_v<T>)
            {
                os << static_cast<std::underlying_type_t<T>>(var);
            }
            else
            {
                os << var;
            }
        };
    }

    // Validate CSV columns for inputs and that writers exist for all headers
    void validateInputs() const
    {
        if (!csv_)
            throw std::runtime_error("CSVReader not attached");
        for (const auto &col : input_names_)
        {
            if (!csv_->hasColumn(col.c_str()))
                throw std::runtime_error("Missing required input column in CSV: " + col);
            if (!setters_.count(col))
                throw std::runtime_error("No setter registered for: " + col);
        }
        for (const auto &name : header_names_)
        {
            if (!writers_.count(name))
                throw std::runtime_error("No writer registered for: " + name);
        }
    }

    // Write CSV header in the order: inputs first, then outputs-not-in-inputs
    void writeHeader(std::ostream &os) const
    {
        for (size_t i = 0; i < header_names_.size(); ++i)
        {
            os << header_names_[i];
            if (i + 1 < header_names_.size())
                os << ",";
        }
        os << "\n";
    }

    // Set all inputs for a given row
    void setInputs(size_t row) const
    {
        for (const auto &n : input_names_)
            setters_.at(n)(row);
    }

    // Write one data row (all header_names_)
    void writeRow(std::ostream &os) const
    {
        for (size_t i = 0; i < header_names_.size(); ++i)
        {
            writers_.at(header_names_[i])(os);
            if (i + 1 < header_names_.size())
                os << ",";
        }
        os << "\n";
    }

    size_t rows() const { return csv_ ? csv_->rows() : 0; }

private:
    CSVReader *csv_ = nullptr;
    std::vector<std::string> input_names_;
    std::vector<std::string> header_names_;
    std::unordered_set<std::string> seen_;
    std::unordered_map<std::string, std::function<void(size_t)>> setters_;
    std::unordered_map<std::string, std::function<void(std::ostream &)>> writers_;
};

// Convenience macros you’ll call with your lists.
#undef TESTHARNESS_REGISTER_INPUT
#undef TESTHARNESS_REGISTER_OUTPUT
#define TESTHARNESS_REGISTER_INPUT(name) g_h.regInput(#name, name);
#define TESTHARNESS_REGISTER_OUTPUT(name) g_h.regOutput(#name, name);

#endif // TEST_HARNESS_H