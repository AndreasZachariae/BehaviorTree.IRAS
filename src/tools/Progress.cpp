#include <behaviortree_ros/tools/Progress.h>

#include <cpp_core/core/Logger.h>

void Progress::next_step(const std::string &message)
{
    this->message = message;

    ++current_step;

    Logger::global_instance.log(to_string());
}

void Progress::set_step(int step, const std::string &message)
{
    this->message = message;

    if (step >= 0 && step <= steps)
    {
        current_step = step;
        Logger::global_instance.log(to_string());
    }
    else
    {
        Logger::global_instance.log("Step " + std::to_string(current_step) + " is out of range 0 to " + std::to_string(step));
    }
}

void Progress::set_fail(const std::string &message)
{
    this->message = message;

    current_step = FAIL_STEP;

    Logger::global_instance.log("Progress failed: " + message, LogLevel::Error);
}

std::string Progress::to_string()
{
    return "Step " + std::to_string(current_step) + "/" + std::to_string(steps) + ": " + message;
}