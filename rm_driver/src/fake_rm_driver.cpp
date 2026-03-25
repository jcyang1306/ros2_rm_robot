// SPDX-License-Identifier: Apache-2.0
// Minimal RM trajectory simulator: mirrors movej CANFD commands into joint_states (no hardware / libapi).

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/jointpos.hpp"
#include "rm_ros_interfaces/msg/jointposcustom.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class FakeRmDriver : public rclcpp::Node
{
public:
  FakeRmDriver()
  : rclcpp::Node("fake_rm_driver")
  {

    this->declare_parameter<int>("arm_dof", arm_dof_);
    this->get_parameter<int>("arm_dof", arm_dof_);

    this->declare_parameter<std::vector<std::string>>("arm_joints", arm_joints_);

    // set initial joint positions and velocities to 0
    joint_pos_.assign(arm_dof_, 0.0);
    joint_vel_.assign(arm_dof_, 0.0);
    joint_state_.name.resize(arm_dof_);

    if (this->get_parameter("arm_joints", arm_joints_))
    {
        for (int i = 0; i < arm_dof_; i++)
        {
          joint_state_.name[i] = arm_joints_[i];
        }
    }

    rclcpp::QoS qos(10);
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

    sub_canfd_ = this->create_subscription<rm_ros_interfaces::msg::Jointpos>(
      "rm_driver/movej_canfd_cmd",
      rclcpp::ParametersQoS(),
      std::bind(&FakeRmDriver::on_movej_canfd, this, std::placeholders::_1));

    sub_canfd_custom_ = this->create_subscription<rm_ros_interfaces::msg::Jointposcustom>(
      "rm_driver/movej_canfd_custom_cmd",
      rclcpp::ParametersQoS(),
      std::bind(&FakeRmDriver::on_movej_canfd_custom, this, std::placeholders::_1));


    double publish_rate = 50.0;
    auto period = std::chrono::duration<double>(1.0 / std::max(publish_rate, 1.0));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FakeRmDriver::publish_joint_state, this));

    RCLCPP_INFO(
      get_logger(),
      "fake_rm_driver running (%zu DOF); listening on rm_driver/movej_canfd_cmd", arm_dof_);
  }

private:
  static void copy_joints(
    const std::vector<float> & joint_in, uint8_t dof_in, size_t arm_dof,
    std::vector<double> & joint_out)
  {
    size_t n = std::min(arm_dof, joint_in.size());
    if (dof_in > 0) {
      n = std::min(n, static_cast<size_t>(dof_in));
    }
    for (size_t i = 0; i < n; ++i) {
      joint_out[i] = static_cast<double>(joint_in[i]);
    }
  }

  void on_movej_canfd(const rm_ros_interfaces::msg::Jointpos::SharedPtr msg)
  {
    copy_joints(msg->joint, msg->dof, arm_dof_, joint_pos_);
  }

  void on_movej_canfd_custom(const rm_ros_interfaces::msg::Jointposcustom::SharedPtr msg)
  {
    copy_joints(msg->joint, msg->dof, arm_dof_, joint_pos_);
  }

  void publish_joint_state()
  {
    sensor_msgs::msg::JointState out;
    joint_state_.name = arm_joints_;
    joint_state_.position.assign(joint_pos_.begin(), joint_pos_.end());
    joint_state_.velocity.assign(joint_vel_.begin(), joint_vel_.end());
    joint_state_.header.stamp = this->now();
    pub_->publish(joint_state_);
  }

  int arm_dof_{6};
  std::vector<std::string> arm_joints_;
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  sensor_msgs::msg::JointState joint_state_;


  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<rm_ros_interfaces::msg::Jointpos>::SharedPtr sub_canfd_;
  rclcpp::Subscription<rm_ros_interfaces::msg::Jointposcustom>::SharedPtr sub_canfd_custom_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeRmDriver>());
  rclcpp::shutdown();
  return 0;
}
