// SPDX-License-Identifier: Apache-2.0
// Fortress Contact publishes every physics step. Bound the ROS-facing stream
// without fabricating heartbeats when the simulator has stopped producing data.
#include <ignition/transport/Node.hh>
#include <ignition/msgs/contacts.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <chrono>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto ros = std::make_shared<rclcpp::Node>("contact_throttle");
  ignition::transport::Node transport;
  auto pub = transport.Advertise<ignition::msgs::Contacts>("/p6/contacts");
  std::mutex mutex;
  ignition::msgs::Contacts latest;
  bool fresh = false;
  std::function<void(const ignition::msgs::Contacts &)> callback =
      [&](const ignition::msgs::Contacts &msg) {
        std::lock_guard<std::mutex> lock(mutex);
        latest = msg;
        fresh = true;
      };
  if (!transport.Subscribe("/p6/contacts_raw", callback)) return 1;
  auto timer = ros->create_wall_timer(std::chrono::milliseconds(20), [&]() {
    ignition::msgs::Contacts msg;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!fresh) return;
      msg = latest;
      fresh = false;
    }
    pub.Publish(msg);
  });
  rclcpp::spin(ros);
  rclcpp::shutdown();
  return 0;
}
