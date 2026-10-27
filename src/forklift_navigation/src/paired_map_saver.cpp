// SPDX-License-Identifier: Apache-2.0
// Lifecycle frontend for the corrected-pose snapshot saver in ProbabilitySlam.
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include "nav2_util/lifecycle_node.hpp"
#include "slam_toolbox/srv/save_map.hpp"

class PairedMapSaver : public nav2_util::LifecycleNode
{
  using SaveMap = slam_toolbox::srv::SaveMap;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Clock = std::chrono::steady_clock;

public:
  PairedMapSaver() : nav2_util::LifecycleNode("map_saver_server")
  {
    declare_parameter("snapshot_service", "/slam_toolbox/save_maps");
    declare_parameter("save_timeout", 30.0);
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    timeout_ = get_parameter("save_timeout").as_double();
    if (!std::isfinite(timeout_) || timeout_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "save_timeout must be finite and positive");
      return CallbackReturn::FAILURE;
    }
    try {
      client_ = create_client<SaveMap>(get_parameter("snapshot_service").as_string());
      // Deferred response keeps heartbeat and lifecycle callbacks responsive
      // even while the SLAM backend builds/writes a large map.
      service_ = create_service<SaveMap>("/slam_toolbox/save_map", [this](
          std::shared_ptr<rclcpp::Service<SaveMap>> service,
          std::shared_ptr<rmw_request_id_t> header,
          std::shared_ptr<SaveMap::Request> request) {
          if (!active_ || pending_ || !client_->service_is_ready()) {
            SaveMap::Response response;
            response.result = SaveMap::Response::RESULT_UNDEFINED_FAILURE;
            service->send_response(*header, response);
            RCLCPP_WARN(get_logger(), "Save rejected: saver inactive, busy, or SLAM unavailable");
            return;
          }
          try {
            pending_.emplace(client_->async_send_request(request));
            response_service_ = service;
            response_header_ = header;
            deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(timeout_));
          } catch (const std::exception & error) {
            SaveMap::Response response;
            response.result = SaveMap::Response::RESULT_UNDEFINED_FAILURE;
            service->send_response(*header, response);
            RCLCPP_ERROR(get_logger(), "Save request failed: %s", error.what());
          }
        });
      timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {checkSave();});
      return CallbackReturn::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Configure failed: %s", error.what());
      release();
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    active_ = true;
    createBond();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    if (bond_) {destroyBond();}
    // Previously accepted saves may finish; new requests are rejected.
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    release();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    release();
    return CallbackReturn::SUCCESS;
  }

private:
  void finish(uint8_t result)
  {
    SaveMap::Response response;
    response.result = result;
    try {
      response_service_->send_response(*response_header_, response);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Could not deliver save result: %s", error.what());
    }
    pending_.reset();
    response_service_.reset();
    response_header_.reset();
  }

  void checkSave()
  {
    if (!pending_) {return;}
    if (pending_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto result = pending_->get()->result;
        finish(result);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Save Map failed: %s", error.what());
        finish(SaveMap::Response::RESULT_UNDEFINED_FAILURE);
      }
    } else if (Clock::now() >= deadline_) {
      client_->remove_pending_request(*pending_);
      RCLCPP_ERROR(get_logger(), "Save timed out; backend may still write files. Check SLAM logs.");
      finish(SaveMap::Response::RESULT_UNDEFINED_FAILURE);
    }
  }

  void release()
  {
    active_ = false;
    if (bond_) {destroyBond();}
    if (pending_) {
      client_->remove_pending_request(*pending_);
      finish(SaveMap::Response::RESULT_UNDEFINED_FAILURE);
    }
    timer_.reset();
    service_.reset();
    client_.reset();
  }

  bool active_{false};
  double timeout_{30.0};
  Clock::time_point deadline_;
  std::optional<rclcpp::Client<SaveMap>::FutureAndRequestId> pending_;
  rclcpp::Client<SaveMap>::SharedPtr client_;
  rclcpp::Service<SaveMap>::SharedPtr service_, response_service_;
  std::shared_ptr<rmw_request_id_t> response_header_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PairedMapSaver>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
