# FrankaController-tips
In this repository, I summarized the problems I ran into and tips I got while creating ROS c++ packages for Franka arm. For sample controller codes, you can check my other repositories: [Franka Feedback Controller](https://github.com/hh2564/franka_feedback_controller) and [Franka Feedback Linearization Controller](https://github.com/hh2564/franka_feedback_linearization_controller). 

# Useful Libraries 
In order to create a controller package from Franka arm, you will need to install [ros](http://wiki.ros.org), [Libfranka](https://frankaemika.github.io/docs/libfranka.html), [franka_ros](https://frankaemika.github.io/docs/franka_ros.html), and [ros_control](http://wiki.ros.org/ros_control). I would also recommand using [pinocchio](https://github.com/stack-of-tasks/pinocchio) for high speed computing robot arm dynamic. 

# Creating Package 
After successfully installed required and recommanded ROS libraries, you can start to create the controller catkin package.
While creating package, we can add our desired dependencies: 
```
catkin_create_pkg <your_controller_pkg_name> <dependencies> 
```
Some useful dependencies for Franka arm controller can be:
```
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_gripper
  franka_hw
  geometry_msgs
  hardware_interface
  joint_limits_interface
  pluginlib
  realtime_tools
  roscpp
  tf
  tf_conversions
  urdf
  visualization_msgs 
  ```

# Additional Folders 
In order to maintain functionality of the Franka arm controller package, I added the following folders to collect corresponding files: 
* config 
* launch 
* urdf
* src 
* include/<controller_name> 

In the config folder, it stores a yaml file with required controller parameters. In the launch folder, it stores launch files to spawn your robot model and controller into Gazebo simulation environment. In urdf folder, it stores a urdf file that need to be loaded if you choose to use [pinocchio](https://github.com/stack-of-tasks/pinocchio). In src folder, it stores your controller source c++ file. In include folder, it stores your header file corresponding to your c++ source file. 

# Managing CMakeList 
In order to ensure your controller package compile correctly, we need to take a look at the CMakeLists.txt.

**In order to makesure your controller support Eigen library, we need to use C++14 by adding the following lines in the beginning of your CMake file:**

```
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```
To load the Franka, Eigen, and Pinocchio Libraries properly, we need: 
```
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()
...
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio ${catkin_LIBRARIES})
```

We also need to makesure your source file and header file are loaded properly: 
```
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

...
add_library(franka_feedback_linearization_controller src/franka_feedback_linearization_controller.cpp)
```
# Loading Controller 
In order to ensure your controller is loaded by ros controller manager, you can follow the instruction on the [Franka arm website](https://frankaemika.github.io/docs/franka_ros.html#writing-your-own-controller) with detailed description. 

# Controller Script 
## Init 
In the init part of the controller, you need to load the the majority of the parameters, and here's a sample code for loading parameter and notifing user if there's something wrong: 
```
std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("FeedbackLinearizationController: Could not read parameter arm_id");
        return false;
    }
```
Some essential parameters to consider are: 
* arm_id and joint_names (to  use in hardware handles)
* publish rate 
* joint_handles_
* state_handle_
* model_handle_

You will also need to have corresponding interfaces to make sure the handles work. 

In addition to getting paramters, if you need to use [pinocchio](https://github.com/stack-of-tasks/pinocchio), you will need to build model and get data in this section: 
```
pinocchio::urdf::buildModel(<model_urdf_file_name>, model);
data = pinocchio::Data(model);
controlled_frame = model.getFrameId(<ehttp://docs.ros.org/en/kinetic/api/franka_hw/html/classfranka__hw_1_1FrankaModelHandle.htmlnd_effector_frame_id>); 
```
## Starting 
This section is not required for creating a controller, but you can choose to add something in the beginning of your control. For example, in my implementation of [Franka Feedback Linearization Controller](https://github.com/hh2564/franka_feedback_linearization_controller), I choose to calculate my desired trajectory in this section. 

## Update 
Something need to keep in mind is while using pinocchio, you need to update model and data everytime: 
```
pinocchio::forwardKinematics(model, data, q, dq);
pinocchio::updateFramePlacements(model, data);
```
Something useful to keep in mind are some useful hardware interfaces: 
* [state_handle_](http://docs.ros.org/en/kinetic/api/libfranka/html/structfranka_1_1RobotState.html)
* [model_handle_](http://docs.ros.org/en/kinetic/api/franka_hw/html/classfranka__hw_1_1FrankaModelHandle.html)

You can use these tools to find the crucial information regarding Franka arm to help you design the controller. 

#
