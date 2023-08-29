# How to control drone OFFBOARD using MAVROS, C++ example COMPLETE GUIDE

Assuming you have Ubuntu 20.04 installed, [ROS Noetic installed](https://youtu.be/8AeTQ8Ew0hc), and [PX4 development environment set](https://youtu.be/9Mb-aV3lmZ0)

## Create a package with rospy and roscpp dependencies

```bash
cd ~/catkin_ws/src
catkin_create_pkg offb roscpp rospy
```

## Create C++ file inside *src* folder of the package

Copy the below C++ code into the *src* folder of the created package

```c
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```

Add the C++ file to the executables section in the CMakeLists.txt

```
add_executable(${PROJECT_NAME}_node src/offb_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
```

## Create a launch file

Create a launch file inside the *launch* folder and include the PX4 MAVROS SITL node

```xml
<launch>
	
	<!-- Include the MAVROS node with SITL and Gazebo -->
	<include file="$(find px4)/launch/mavros_posix_sitl.launch">
	
	</include>

	<!-- Our node to control the drone -->
	<node pkg="offb" type="offb_node" name="offb_node" required="true" output="screen" />

</launch>
```

## Build the catkin workspace

```bash
catkin build
source ~/.bashrc
```

## Run the package

```bash
roslaunch offb main.launch
```

## Code explanation can be found at Official PX4 documentation website
[MAVROS Offboard control example | PX4 User Guide](https://docs.px4.io/master/en/ros/mavros_offboard.html)