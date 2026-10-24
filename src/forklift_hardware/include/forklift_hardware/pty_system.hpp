// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include "hardware_interface/system_interface.hpp"
#include "forklift_hardware/protocol.hpp"

namespace forklift_hardware {
class PtySystem : public hardware_interface::SystemInterface {
 public:
  ~PtySystem() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &,const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &,const rclcpp::Duration &) override;
 private:
  using Clock=std::chrono::steady_clock;
  void worker(); void stop();
  std::array<double,5> position_{},velocity_{},command_{},outgoing_{};
  std::array<std::string,5> names_{{"front_left_wheel_joint","front_right_wheel_joint","rear_left_wheel_joint","rear_right_wheel_joint","lift_joint"}};
  std::string port_file_;
  double cpr_=4096., lift_steps_per_m_=10000.;
  State received_{};
  bool have_state_=false;
  Clock::time_point received_at_{}, written_at_{};
  uint64_t consumed_seq_=0, consumed_us_=0;
  std::array<int64_t,4> previous_ticks_{};
  int64_t previous_lift_=0;
  std::atomic<bool> running_{false}, enabled_{false}, fault_{false};
  std::thread thread_;
  std::mutex mutex_;
  int fd_=-1;
};
}  // namespace forklift_hardware
