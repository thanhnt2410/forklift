// SPDX-License-Identifier: Apache-2.0
// Extend the installed SLAM Toolbox; no fork of its scan matching or solver.
#include <chrono>
#include <memory>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include "slam_toolbox/slam_toolbox_async.hpp"
#include "slam_toolbox/visualization_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "forklift_navigation/probability_grid.hpp"
#include "nav2_map_server/map_io.hpp"

class ProbabilitySlam : public slam_toolbox::AsynchronousSlamToolbox
{
public:
  explicit ProbabilitySlam(const rclcpp::NodeOptions & options)
  : AsynchronousSlamToolbox(options) {}

  void configure() override
  {
    AsynchronousSlamToolbox::configure();
    // Lifecycle frontend owns the public endpoint used by RViz. This backend
    // owns corrected scans, so both maps share one snapshot under mapper lock.
    map_saver_.reset();
    save_service_ = create_service<slam_toolbox::srv::SaveMap>(
      "~/save_maps", [this](
        const std::shared_ptr<slam_toolbox::srv::SaveMap::Request> request,
        std::shared_ptr<slam_toolbox::srv::SaveMap::Response> response) {
        saveMaps(request->name.data, *response);
      });
    const double period = declare_parameter("probability_update_interval", 2.0);
    if (!std::isfinite(period) || period <= 0.0) {
      throw std::invalid_argument("probability_update_interval must be positive");
    }
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    probability_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map_probability", qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/mapping/trajectory", qos);
    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() {publishProbability();});
  }

private:
  void publishProbability()
  {
    nav_msgs::msg::OccupancyGrid navigation, probability;
    nav_msgs::msg::Path path;
    if (!buildSnapshot(navigation, probability, path)) {return;}
    probability_pub_->publish(probability);
    path_pub_->publish(path);
  }

  bool buildSnapshot(nav_msgs::msg::OccupancyGrid & navigation,
    nav_msgs::msg::OccupancyGrid & map, nav_msgs::msg::Path & path)
  {
    {
      // Same lock as upstream scan processing and map building. Rebuild from
      // corrected scans every time, including after loop closure/deserialization.
      boost::mutex::scoped_lock lock(smapper_mutex_);
      const auto scans = smapper_->getMapper()->GetAllProcessedScans();
      if (scans.empty()) {return false;}
      int minimum = 2;
      double threshold = 0.1;
      get_parameter_or("min_pass_through", minimum, 2);
      get_parameter_or("occupancy_threshold", threshold, 0.1);
      auto grid = forklift_navigation::ProbabilityGrid::build(
        scans, resolution_, minimum, threshold);
      if (!grid) {return false;}
      vis_utils::toNavMap(grid.get(), map);
      map.header.frame_id = map_frame_;
      map.header.stamp = scan_header.stamp;
      map.info.resolution = resolution_;
      map.info.map_load_time = map.header.stamp;
      map.info.origin.orientation.w = 1.0;
      navigation = map;
      // Karto rows can contain padding: use its index rather than ROS width.
      for (uint32_t y = 0; y < map.info.height; ++y) {
        for (uint32_t x = 0; x < map.info.width; ++x) {
          map.data[y * map.info.width + x] = grid->probability(x, y);
        }
      }
      path.header = map.header;
      path.poses.reserve(scans.size());
      for (const auto * scan : scans) {
        const auto & corrected = scan->GetCorrectedPose();
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = corrected.GetX();
        pose.pose.position.y = corrected.GetY();
        pose.pose.orientation.z = std::sin(corrected.GetHeading() / 2.0);
        pose.pose.orientation.w = std::cos(corrected.GetHeading() / 2.0);
        path.poses.push_back(pose);
      }
    }
    return true;
  }

  void saveMaps(const std::string & requested, slam_toolbox::srv::SaveMap::Response & response)
  {
    using Response = slam_toolbox::srv::SaveMap::Response;
    response.result = Response::RESULT_UNDEFINED_FAILURE;
    try {
      if (requested.empty()) {throw std::invalid_argument("Enter a map filename prefix");}
      const char * home = std::getenv("HOME");
      const auto prefix = std::filesystem::absolute(
        nav2_map_server::expand_user_home_dir_if_needed(requested, home ? home : "")).string();
      nav_msgs::msg::OccupancyGrid navigation, probability;
      nav_msgs::msg::Path path;
      if (!buildSnapshot(navigation, probability, path)) {
        response.result = Response::RESULT_NO_MAP_RECEIEVD;
        return;
      }
      std::filesystem::create_directories(std::filesystem::path(prefix).parent_path());
      nav2_map_server::SaveParameters options;
      options.map_file_name = prefix;
      options.image_format = "pgm";
      options.free_thresh = 0.25;
      options.occupied_thresh = 0.65;
      options.mode = nav2_map_server::MapMode::Trinary;
      if (!nav2_map_server::saveMapToFile(navigation, options)) {
        throw std::runtime_error("Could not save navigation map");
      }
      options.map_file_name = prefix + "_probability";
      options.image_format = "png";
      options.free_thresh = 0.01;
      options.occupied_thresh = 0.99;
      options.mode = nav2_map_server::MapMode::Scale;
      if (!nav2_map_server::saveMapToFile(probability, options)) {
        throw std::runtime_error("Navigation map saved, but probability image failed");
      }
      // Keep exact integer probabilities too: scale PNG quantizes gray values.
      std::ofstream raw(prefix + "_probability.grid.json");
      raw.exceptions(std::ios::failbit | std::ios::badbit);
      raw << std::setprecision(17)
          << "{\"frame_id\":" << std::quoted(probability.header.frame_id)
          << ",\"stamp\":{\"sec\":" << probability.header.stamp.sec
          << ",\"nanosec\":" << probability.header.stamp.nanosec << '}'
          << ",\"width\":" << probability.info.width
          << ",\"height\":" << probability.info.height
          << ",\"resolution\":" << probability.info.resolution
          << ",\"origin\":[" << probability.info.origin.position.x << ','
          << probability.info.origin.position.y << ",0],\"data\":[";
      for (size_t i = 0; i < probability.data.size(); ++i) {
        if (i) {raw << ',';}
        raw << static_cast<int>(probability.data[i]);
      }
      raw << "]}\n";
      raw.close();
      response.result = Response::RESULT_SUCCESS;
      RCLCPP_INFO(get_logger(), "Saved navigation + probability maps: %s", prefix.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Save Map failed (some files may already exist): %s", error.what());
    }
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr probability_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<slam_toolbox::srv::SaveMap>::SharedPtr save_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProbabilitySlam>(rclcpp::NodeOptions());
  node->configure();
  node->loadPoseGraphByParams();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
