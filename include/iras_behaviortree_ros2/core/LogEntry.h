#pragma once

#include <iras_behaviortree_ros2/default.h>
#include <iras_behaviortree_ros2/core/LogLevel.h>
#include <iras_behaviortree_ros2/core/ChronoTime.h>

struct LogEntry
{
    std::string message;
    unsigned int hierarchy;
    LogLevel level;
    ChronoTime time;

    std::string to_string() const { return "[" + level.to_string() + " | " + time.to_string("%H:%M:%S") + "]: " + message; }
};
