#pragma once

/********************************************************************************
 * @file    csv_reader.h
 * @author  Cameron Rose
 * @date    10/19/2025
 ********************************************************************************/
#ifndef CSV_READER_H
#define CSV_READER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <stdexcept>
#include <type_traits>
#include <charconv>
#include <cstdlib>
#include <limits>
#include <algorithm>

/********************************************************************************
 * Function prototypes
 ********************************************************************************/

class CSVReader
{
public:
    explicit CSVReader(const std::string &path, char delimiter = ',')
        : delim_(delimiter)
    {
        std::ifstream in(path);
        if (!in)
            throw std::runtime_error("CSVReader: failed to open file: " + path);

        // Read header
        std::string line;
        if (!std::getline(in, line))
            throw std::runtime_error("CSVReader: empty file: " + path);

        headers_ = splitCSV(line, delim_);
        if (headers_.empty())
            throw std::runtime_error("CSVReader: no headers found");

        // Ensure unique header names (simple check)
        {
            std::unordered_map<std::string, int> seen;
            for (const auto &h : headers_)
            {
                if (++seen[h] > 1)
                    throw std::runtime_error("CSVReader: duplicate header: " + h);
            }
        }

        // Prepare columns
        for (const auto &h : headers_)
        {
            columns_.emplace(h, std::vector<std::string>{});
        }

        // Read rows
        std::vector<std::string> fields;
        while (std::getline(in, line))
        {
            fields = splitCSV(line, delim_);
            // Grow all columns to same row count; pad missing cells with empty
            for (size_t c = 0; c < headers_.size(); ++c)
            {
                if (c < fields.size())
                {
                    columns_[headers_[c]].push_back(std::move(fields[c]));
                }
                else
                {
                    columns_[headers_[c]].push_back(std::string{});
                }
            }
            ++nrows_;
        }
    }

    // Number of data rows (excluding header)
    size_t rows() const noexcept { return nrows_; }

    bool hasColumn(const std::string &header) const noexcept
    {
        return columns_.find(header) != columns_.end();
    }

    // ----- Value access: csv("header", index) -----
    class ValueProxy
    {
    public:
        ValueProxy(const std::vector<std::string> *col, size_t idx)
            : col_(col), idx_(idx) {}

        // String conversion
        operator std::string() const { return at(); }

        // Arithmetic conversions
        template <typename T,
                  typename = std::enable_if_t<std::is_arithmetic_v<T>>>
        operator T() const
        {
            return CSVReader::convertArithmetic<T>(at());
        }

        // Enum conversions
        template <typename T,
                  typename = std::enable_if_t<std::is_enum_v<T>>,
                  typename = void>
        operator T() const
        {
            using U = std::underlying_type_t<T>;
            U v = CSVReader::convertArithmetic<U>(at());
            return static_cast<T>(v);
        }

    private:
        const std::vector<std::string> *col_;
        size_t idx_;

        const std::string &at() const
        {
            if (!col_)
                throw std::runtime_error("CSVReader: null column");
            if (idx_ >= col_->size())
                throw std::out_of_range("CSVReader: row index out of range");
            return (*col_)[idx_];
        }
    };

    // Matches: (int)csv("header", idx)
    ValueProxy operator()(const std::string &header, size_t index) const
    {
        auto it = columns_.find(header);
        if (it == columns_.end())
            throw std::runtime_error("CSVReader: missing column: " + header);
        return ValueProxy(&it->second, index);
    }

    // Fill a std::vector<T> starting from start_index (clamped to available rows)
    template <typename T>
    void operator()(std::vector<T> &out, const std::string &header, size_t start_index = 0) const
    {
        const auto &col = requireColumn(header);
        if (start_index > col.size())
            throw std::out_of_range("CSVReader: start_index past end of column for '" + header + "'");
        const size_t n = std::min(out.size(), col.size() - start_index);
        for (size_t i = 0; i < n; ++i)
        {
            out[i] = convertMaybeEnum<T>(col[start_index + i]);
        }
        // if out is larger than n, we leave trailing elements untouched
    }

    // Fill a C array T out[N] starting from start_index
    template <typename T, size_t N>
    void operator()(T (&out)[N], const std::string &header, size_t start_index = 0) const
    {
        const auto &col = requireColumn(header);
        if (start_index > col.size())
            throw std::out_of_range("CSVReader: start_index past end of column for '" + header + "'");
        const size_t n = std::min(N, col.size() - start_index);
        for (size_t i = 0; i < n; ++i)
        {
            out[i] = convertMaybeEnum<T>(col[start_index + i]);
        }
        // trailing untouched if N > n
    }

    // Direct access to the raw string column (if you need it)
    const std::vector<std::string> &columnRaw(const std::string &header) const
    {
        return requireColumn(header);
    }

    void eraseRow(size_t i)
    {
        if (i >= nrows_)
            return; // or throw, if you prefer

        for (auto &kv : columns_)
        {
            auto &col = kv.second;
            if (i < col.size())
            {
                col.erase(col.begin() + static_cast<std::ptrdiff_t>(i));
            }
        }
        --nrows_;
    }

    bool dropFirstRowIfDuplicateZero(const std::string &timeHeader)
    {
        if (nrows_ < 2)
            return false;
        float t0 = static_cast<float>((*this)(timeHeader, 0));
        float t1 = static_cast<float>((*this)(timeHeader, 1));
        if (t0 == 0.0f && t1 == 0.0f)
        {
            eraseRow(0);
            return true;
        }
        return false;
    }

private:
    char delim_;
    size_t nrows_ = 0;
    std::vector<std::string> headers_;
    std::unordered_map<std::string, std::vector<std::string>> columns_;

    const std::vector<std::string> &requireColumn(const std::string &header) const
    {
        auto it = columns_.find(header);
        if (it == columns_.end())
            throw std::runtime_error("CSVReader: missing column: " + header);
        return it->second;
    }

    static std::vector<std::string> splitCSV(const std::string &line, char delim)
    {
        std::vector<std::string> out;
        std::string cur;
        cur.reserve(32);

        bool in_quotes = false;
        for (size_t i = 0; i < line.size(); ++i)
        {
            char ch = line[i];
            if (in_quotes)
            {
                if (ch == '"')
                {
                    // double quote inside quotes -> escaped quote
                    if (i + 1 < line.size() && line[i + 1] == '"')
                    {
                        cur.push_back('"');
                        ++i;
                    }
                    else
                    {
                        in_quotes = false;
                    }
                }
                else
                {
                    cur.push_back(ch);
                }
            }
            else
            {
                if (ch == '"')
                {
                    in_quotes = true;
                }
                else if (ch == delim)
                {
                    out.push_back(std::move(cur));
                    cur.clear();
                }
                else
                {
                    cur.push_back(ch);
                }
            }
        }
        out.push_back(std::move(cur));
        return out;
    }

    // Convert arithmetic types (ints/floats/bools)
    template <typename T>
    static T convertArithmetic(const std::string &s)
    {
        if constexpr (std::is_same_v<T, bool>)
        {
            // Accept 1/0/true/false (case-insensitive for words)
            if (s == "1")
                return true;
            if (s == "0")
                return false;
            if (ieq(s, "true"))
                return true;
            if (ieq(s, "false"))
                return false;
            throw std::runtime_error("CSVReader: cannot parse bool from '" + s + "'");
        }
        else if constexpr (std::is_integral_v<T>)
        {
            T val{};
            auto *b = s.data();
            auto *e = s.data() + s.size();
            std::from_chars_result res = std::from_chars(b, e, val, 10);
            if (res.ec != std::errc())
                throw std::runtime_error("CSVReader: integer parse error for '" + s + "'");
            return val;
        }
        else if constexpr (std::is_floating_point_v<T>)
        {
            // Use strtod/strtof; from_chars(float) not widely available pre-C++20 on some libs
            char *end = nullptr;
            errno = 0;
            long double ld = std::strtold(s.c_str(), &end);
            if (end == s.c_str() || errno == ERANGE)
                throw std::runtime_error("CSVReader: float parse error for '" + s + "'");
            // clamp/convert
            long double lo = std::numeric_limits<T>::lowest();
            long double hi = std::numeric_limits<T>::max();
            if (ld < lo)
                ld = lo;
            if (ld > hi)
                ld = hi;
            return static_cast<T>(ld);
        }
        else
        {
            static_assert(std::is_arithmetic_v<T>, "Unsupported arithmetic type");
        }
    }

    template <typename T>
    static T convertMaybeEnum(const std::string &s)
    {
        if constexpr (std::is_enum_v<T>)
        {
            using U = std::underlying_type_t<T>;
            U v = convertArithmetic<U>(s);
            return static_cast<T>(v);
        }
        else if constexpr (std::is_same_v<T, std::string>)
        {
            return s;
        }
        else
        {
            return convertArithmetic<T>(s);
        }
    }

    static bool ieq(const std::string &a, const std::string &b)
    {
        if (a.size() != b.size())
            return false;
        for (size_t i = 0; i < a.size(); ++i)
        {
            char ca = a[i], cb = b[i];
            if (ca >= 'A' && ca <= 'Z')
                ca = static_cast<char>(ca - 'A' + 'a');
            if (cb >= 'A' && cb <= 'Z')
                cb = static_cast<char>(cb - 'A' + 'a');
            if (ca != cb)
                return false;
        }
        return true;
    }
};

#endif // CSV_READER_H
