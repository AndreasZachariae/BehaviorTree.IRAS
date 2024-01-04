#pragma once

#include <iras_behaviortree_ros2/default.h>

class LogLevel
{
public:
    enum Value : uint8_t
    {
        Debug = 0,
        Info = 1,
        Warn = 2,
        Error = 3
    };

    LogLevel(Value value) : value_(value) {}
    LogLevel() : LogLevel(Info) {}

    operator Value() const { return value_; }
    explicit operator bool() = delete;

    std::string to_string() const;
    
private:
    Value value_;
};