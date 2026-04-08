#include "yur_ros2_driver/yur_ros_wrapper.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string RightArm_ip   = "192.168.1.120";
    std::string RightArm_name = "UR10_right";
    std::string LeftArm_ip    = "192.168.1.121";
    std::string LeftArm_name  = "UR10_left";
    int RightArm_reverse_port = 50001;
    int LeftArm_reverse_port  = 50002;

    /* Right: Follower 모드 (read_only=false)
       - uploadProg + servoj
       - /UR10_right/targetJ 수신 → 로봇 실행 */
    auto RightArm = std::make_shared<YRos2Wrapper>(
        RightArm_ip, RightArm_reverse_port, RightArm_name,
        false);  // read_only = false

    /* Left: Leader 모드 (read_only=true)
       - uploadProg 없음 → 티치펜던트/Freedrive 자유롭게 사용
       - joint_states 만 publish
       - Broken pipe 없음 */
    auto LeftArm = std::make_shared<YRos2Wrapper>(
        LeftArm_ip, LeftArm_reverse_port, LeftArm_name,
        true);   // read_only = true

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(RightArm);
    executor.add_node(LeftArm);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
