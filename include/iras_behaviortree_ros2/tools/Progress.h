/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : iras_behaviortree_ros2
 * Purpose : Provides tracking of the progress for ROS action and service protocol
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <iras_behaviortree_ros2/default.h>

class Progress
{
public:
    static const int FAIL_STEP = -1;

    Progress(int steps = 0) : steps(steps) {}

    int steps = 0;
    int current_step = 0;
    std::string message = "";

    void next_step(const std::string &message = "");

    void set_step(int step, const std::string &message = "");
    void set_fail(const std::string &message = "");

    std::string to_string();
};