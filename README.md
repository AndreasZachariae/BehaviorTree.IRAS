# BehaviorTree.ROS

This package is a wrapper around the [BehaviorTree.cpp](https://www.behaviortree.dev/) package.

It provides an easy to use API for ROS2 services and actions inside a BehaviorTree node.

![example_tree](https://d33wubrfki0l68.cloudfront.net/f8b2bac65168251a46ec25232f20db7961327ffc/88ad1/images/readthedocs.png)

## Access token
repository | user:token
-----------|-----------
behaviortree_ros | `project_240_bot:glpat-stK1tgiDxr44VV8X95r7`


## Deprecated ros_core Package

`ros_core` will be removed because behaviortree functionalities were moved to `common/behaviortree_ros` and the cpp library is here in `cpp_core`

Before `cpp_core` was integrated as submodule in `ros_core` and to use it only a `ros_core` dependecy was necessary.

Now `cpp_core` is its own ros package

### Example change from ros_core to cpp_core

Change all Headers from ~~`#inlcude <ros_core/...>`~~ to `#include <behaviortree_ros/...>`

Headers with `#inlcude <cpp_core/...>` can stay the same

in `CMakeLists.txt`:

~~find_package(ros_core REQUIRED)~~

~~set(DEPENDENCIES ros_core)~~

~~target_link_libraries(TaskPlanner ros_core::ros_core_lib)~~

~~include_directories(${ros_core_INCLUDE_DIRS}~~

```
find_package(behaviortree_ros REQUIRED)
find_package(cpp_core REQUIRED)

include_directories(
  ${cpp_core_INCLUDE_DIRS}
  ${behaviortree_ros_INCLUDE_DIRS}
)

set(DEPENDENCIES 
  behaviortree_ros
  cpp_core
)

target_link_libraries(TaskPlanner
  behaviortree_ros::behaviortree_ros_lib
  cpp_core::cpp_core_lib
)
```

in `package.xml`:

~~`<depend>ros_core</depend>`~~

```
<depend>behaviortree_ros</depend>
<depend>cpp_core</depend>
```
