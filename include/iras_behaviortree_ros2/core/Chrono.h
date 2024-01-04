#pragma once

#include <iras_behaviortree_ros2/default.h>
#include <iras_behaviortree_ros2/core/ChronoEntry.h>

class Chrono
{
public:
    static Chrono global_instance;

    Chrono() {}
    ~Chrono() {}

    std::string start();
    std::string start(std::string key);
    int64_t stop(std::string key);

private:
    std::map<std::string, ChronoEntry> entries;
};
