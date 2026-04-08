#include "yur_ros2_driver/yur_ros_wrapper.hpp"

YRos2Wrapper::YRos2Wrapper(std::string host, int reverse_port,
                            std::string robot_name,
                            bool read_only)
    : Node(robot_name),
      robot_(rt_msg_cond_, msg_cond_, host, reverse_port, 0.03, 300),
      io_flag_delay_(0.05),
      joint_offsets_(6, 0.0),
      robot_name_(robot_name),
      read_only_(read_only),
      mb_publish_thread_(nullptr),
      rt_publish_thread_(nullptr)
{
    /* Joint state publisher (공통) */
    std::string joint_state_topic = robot_name_ + "/joint_states";
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic, 1);

    if (read_only_) {
        /* ============================================================
         * Leader 모드 (read_only = true)
         *   uploadProg 없음 -> 티치펜던트/Freedrive 자유롭게 사용
         *   RT 포트(30003) 만 연결 -> joint_states publish
         *   Broken pipe 없음
         * ============================================================ */
        RCLCPP_INFO(this->get_logger(),
            "\033[33m[Leader] read_only 모드: %s\033[0m",
            robot_name_.c_str());

        /* sec_interface 시작 (펌웨어 버전 확인용) */
        if (!robot_.sec_interface_->start()) {
            RCLCPP_ERROR(this->get_logger(), "Secondary interface 연결 실패");
            return;
        }

        double fw = robot_.sec_interface_->robot_state_->getVersion();
        robot_.rt_interface_->robot_state_->setVersion(fw);
        RCLCPP_INFO(this->get_logger(), "Firmware version: %.7f", fw);

        /* RT 포트만 연결 */
        if (!robot_.rt_interface_->start()) {
            RCLCPP_ERROR(this->get_logger(), "RT interface 연결 실패");
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "\033[32m[Leader] joint_states publish: \"%s\"\033[0m",
            joint_state_topic.c_str());
        RCLCPP_INFO(this->get_logger(),
            "\033[32m[Leader] 티치펜던트/Freedrive 사용 가능\033[0m");

        rt_publish_thread_ = new std::thread(
            std::bind(&YRos2Wrapper::publishRTMsg, this));
        mb_publish_thread_ = new std::thread(
            std::bind(&YRos2Wrapper::publishMbMsg, this));

    } else {
        /* ============================================================
         * Follower 모드 (read_only = false) - 기존 동작
         *   uploadProg + reverse connection + servoj
         * ============================================================ */
        if (robot_.start()) {
            RCLCPP_INFO(this->get_logger(), "Robot started successfully");

            std::string joint_target_topic = robot_name_ + "/targetJ";
            Yjointcmd_sub_ = this->create_subscription<
                std_msgs::msg::Float64MultiArray>(
                joint_target_topic, 1,
                std::bind(&YRos2Wrapper::jointcmdCB, this,
                          std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(),
                "\033[32mit can be executed by the topic: \"%s\"\033[0m",
                joint_target_topic.c_str());
            RCLCPP_INFO(this->get_logger(),
                "\033[32mReal-time joint state is published by the topic: \"%s\"\033[0m",
                joint_state_topic.c_str());

            rt_publish_thread_ = new std::thread(
                std::bind(&YRos2Wrapper::publishRTMsg, this));
            mb_publish_thread_ = new std::thread(
                std::bind(&YRos2Wrapper::publishMbMsg, this));

            RCLCPP_INFO(this->get_logger(),
                "\033[31mReal-time state publishing thread was generated\033[0m");
        }
    }
}

YRos2Wrapper::~YRos2Wrapper() {
    halt();
}

void YRos2Wrapper::halt() {
    robot_.halt();
    if (rt_publish_thread_) {
        rt_publish_thread_->join();
        delete rt_publish_thread_;
        rt_publish_thread_ = nullptr;
    }
    if (mb_publish_thread_) {
        mb_publish_thread_->join();
        delete mb_publish_thread_;
        mb_publish_thread_ = nullptr;
    }
}

void YRos2Wrapper::jointcmdCB(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!read_only_) {
        robot_.servoj(msg->data);
    }
}

void YRos2Wrapper::publishRTMsg()
{
    sensor_msgs::msg::JointState joint_msg;

    while (rclcpp::ok()) {
        joint_msg.name = robot_.getJointNames();
        std::unique_lock<std::mutex> locker(RT_msg_lock);

        while (!robot_.rt_interface_->robot_state_->getDataPublished()) {
            rt_msg_cond_.wait(locker);
        }

        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.position =
            robot_.rt_interface_->robot_state_->getQActual();

        for (unsigned int i = 0; i < joint_msg.position.size(); i++) {
            joint_msg.position[i] += joint_offsets_[i];
        }

        joint_msg.velocity =
            robot_.rt_interface_->robot_state_->getQdActual();
        joint_msg.effort =
            robot_.rt_interface_->robot_state_->getIActual();

        joint_pub_->publish(joint_msg);
        robot_.rt_interface_->robot_state_->setDataPublished();
    }
}

void YRos2Wrapper::publishMbMsg()
{
    bool warned = false;
    printf("hello Mb~~");
    while (rclcpp::ok()) {
        std::unique_lock<std::mutex> locker(Mb_msg_lock);
        while (!robot_.sec_interface_->robot_state_->getNewDataAvailable()) {
            msg_cond_.wait(locker);
        }

        if (robot_.sec_interface_->robot_state_->isEmergencyStopped()
            || robot_.sec_interface_->robot_state_->isProtectiveStopped()) {
            if (robot_.sec_interface_->robot_state_->isEmergencyStopped()
                && !warned) {
                print_error("Emergency stop pressed!");
            } else if (
                robot_.sec_interface_->robot_state_->isProtectiveStopped()
                && !warned) {
                print_error("Robot is protective stopped!");
            }
            warned = true;
        } else {
            warned = false;
        }
        robot_.sec_interface_->robot_state_->finishedReading();
    }
}
