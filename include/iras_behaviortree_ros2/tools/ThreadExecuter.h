#pragma once

#include <thread>

#include <iras_behaviortree_ros2/default.h>

class ThreadExecuter
{
public:
    ThreadExecuter(const std::string &filepath) : filepath_(filepath) {}
    ~ThreadExecuter() {}

    std::string filepath(){ return filepath_; }
    bool is_running() { return thread_handle_ > 0; }

    bool run();
    bool cancel();

private:
    std::string filepath_;

    pthread_t thread_handle_ = 0;
};