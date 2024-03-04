/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : iras_behaviortree_ros2
 * Purpose : Provides conversion functions between datatypes
 *
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <cmath>

#include <behaviortree_cpp_v3/basic_types.h>

#include <iras_behaviortree_ros2/default.h>

/** NECESSARY!
 * This is called from the BT-framework when reading input ports.
 * Provides conversion from .xml-String to float.
 */
template <>
inline float BT::convertFromString<float>(BT::StringView str)
{
    return std::stof(str.data());
}

template <>
inline std::vector<int> BT::convertFromString<std::vector<int>>(BT::StringView str)
{
    std::vector<int> result;
    std::vector<std::string> parts;
    boost::split(parts, str.data(), boost::is_any_of(";, "), boost::token_compress_on);
    for (const auto &part : parts)
    {
        try
        {
            result.push_back(std::stoi(part));
        }
        catch (const std::exception &e)
        {
            throw BT::RuntimeError("Failed to convert string to vector<int>: " + std::string(e.what()));
        }
    }
    return result;
}

class Converter
{
public:
    static std::string ftos(std::string float_str)
    {
        float_str.erase(float_str.find_last_not_of('0') + 1, std::string::npos);
        float_str.erase(float_str.find_last_not_of('.') + 1, std::string::npos);

        return float_str;
    }

    static std::string ftos(float value, int digits = 2)
    {
        int factor = std::pow(10, digits);

        return ftos(std::to_string(std::roundf(value * factor) / factor));
    }
};