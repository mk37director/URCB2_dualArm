#pragma once

/* Basic headers */
#include "yur_ros2_driver/ur_driver.h"
#include "yur_ros2_driver/do_output.h"
#include <stdio.h>
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>

/* ROS2 headers */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class YRos2Wrapper : public rclcpp::Node {
protected:
    /* UR driver instances & parameters */
    UrDriver robot_;
    std::condition_variable rt_msg_cond_;
    std::condition_variable msg_cond_;
    double io_flag_delay_;
    std::vector<double> joint_offsets_;

    /* ROS2 Subscribers */
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Yjointcmd_sub_;

    /* ROS2 Publishers */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    /* Thread instances */
    std::thread* mb_publish_thread_;
    std::thread* rt_publish_thread_;

    /* Robot name */
    std::string robot_name_;

    /* read_only_ = true  : Leader 모드 (joint_states publish 만)
       read_only_ = false : Follower 모드 (uploadProg + servoj) */
    bool read_only_;

public:
    YRos2Wrapper(std::string host, int reverse_port,
                 std::string robot_name,
                 bool read_only = false);

    ~YRos2Wrapper();

    void halt();

private:
    std::mutex RT_msg_lock;
    std::mutex Mb_msg_lock;

    void jointcmdCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void publishRTMsg();
    void publishMbMsg();
};
