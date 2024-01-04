#pragma once

#include <iras_behaviortree_ros2/default.h>

#include <memory>

class CommandLine
{
public:
    static std::string execute(const std::string &command)
    {
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);

        if (!pipe)
        {
            throw std::runtime_error("popen() failed!");
        }

        std::array<char, 128> buffer;
        std::string result;

        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        {
            result += buffer.data();
        }

        return result;
    }

    static std::vector<std::string> split_string(const std::string &str,
                                                 const std::string &delimiter)
    {
        std::vector<std::string> strings;

        std::string::size_type pos = 0;
        std::string::size_type prev = 0;

        while ((pos = str.find(delimiter, prev)) != std::string::npos)
        {
            strings.push_back(str.substr(prev, pos - prev));
            prev = pos + 1;
        }

        strings.push_back(str.substr(prev));

        return strings;
    }
};
