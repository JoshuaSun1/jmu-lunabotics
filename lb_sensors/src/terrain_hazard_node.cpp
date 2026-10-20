// Copyright 2026 JMU NASA Lunabotics Team
// SPDX-License-Identifier: MIT

#include "lb_sensors/terrain_hazard_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>
#include <string>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <utility>
#include <vector>

namespace lb_sensors {
namespace {

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

void AddDiagnosticValue(DiagnosticStatus* status, const std::string& key,
                        const std::string& value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status->values.push_back(std::move(item));
}

PointCloud2 MakeCloud(const std::vector<TerrainPoint>& points,
                      const std_msgs::msg::Header& header) {
  PointCloud2 cloud;
  cloud.header = header;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  for (const auto& name : {"x", "y", "z"}) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = static_cast<std::uint32_t>(cloud.fields.size() * 4U);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(std::move(field));
  }
  cloud.data.resize(cloud.row_step);
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto offset = index * cloud.point_step;
    std::memcpy(cloud.data.data() + offset, &points[index].x, 4);
    std::memcpy(cloud.data.data() + offset + 4, &points[index].y, 4);
    std::memcpy(cloud.data.data() + offset + 8, &points[index].z, 4);
  }
  return cloud;
}

struct RigidTransform {
  double r00, r01, r02;
  double r10, r11, r12;
  double r20, r21, r22;
  double tx, ty, tz;

  explicit RigidTransform(const geometry_msgs::msg::TransformStamped& stamped) {
    const auto& q = stamped.transform.rotation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < 0.9 || norm > 1.1) {
      throw std::invalid_argument("camera TF has an invalid rotation");
    }
    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    const double w = q.w / norm;
    r00 = 1.0 - 2.0 * (y * y + z * z);
    r01 = 2.0 * (x * y - z * w);
    r02 = 2.0 * (x * z + y * w);
    r10 = 2.0 * (x * y + z * w);
    r11 = 1.0 - 2.0 * (x * x + z * z);
    r12 = 2.0 * (y * z - x * w);
    r20 = 2.0 * (x * z - y * w);
    r21 = 2.0 * (y * z + x * w);
    r22 = 1.0 - 2.0 * (x * x + y * y);
    tx = stamped.transform.translation.x;
    ty = stamped.transform.translation.y;
    tz = stamped.transform.translation.z;
    if (!std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(tz)) {
      throw std::invalid_argument("camera TF has an invalid translation");
    }
  }

  TerrainPoint Apply(double x, double y, double z) const {
    return {static_cast<float>(r00 * x + r01 * y + r02 * z + tx),
            static_cast<float>(r10 * x + r11 * y + r12 * z + ty),
            static_cast<float>(r20 * x + r21 * y + r22 * z + tz)};
  }
};

float ReadDepth(const Image& image, std::size_t offset) {
  if (image.encoding == "32FC1") {
    std::uint32_t bits = 0;
    if (image.is_bigendian) {
      for (int index = 0; index < 4; ++index) {
        bits = (bits << 8U) | image.data[offset + index];
      }
    } else {
      for (int index = 3; index >= 0; --index) {
        bits = (bits << 8U) | image.data[offset + index];
      }
    }
    float depth;
    std::memcpy(&depth, &bits, sizeof(depth));
    return depth;
  }
  const std::uint16_t millimeters =
      image.is_bigendian
          ? static_cast<std::uint16_t>((image.data[offset] << 8U) | image.data[offset + 1])
          : static_cast<std::uint16_t>(image.data[offset] | (image.data[offset + 1] << 8U));
  return static_cast<float>(millimeters) * 0.001F;
}

} // namespace

class TerrainHazardNode final : public rclcpp::Node {
public:
  TerrainHazardNode() : Node("terrain_hazard_node") {
    const std::string calibration_id = declare_parameter<std::string>("mount_calibration_id", "");
    config_.rock_height_m = declare_parameter<double>("rock_height_m", -1.0);
    config_.drop_depth_m = declare_parameter<double>("drop_depth_m", -1.0);
    config_.max_slope_deg = declare_parameter<double>("max_slope_deg", -1.0);
    config_.min_forward_m = declare_parameter<double>("min_forward_m", 0.3);
    config_.max_forward_m = declare_parameter<double>("max_forward_m", 4.0);
    config_.lateral_limit_m = declare_parameter<double>("lateral_limit_m", 1.5);
    config_.cell_size_m = declare_parameter<double>("cell_size_m", 0.05);
    config_.ground_inlier_tolerance_m =
        declare_parameter<double>("ground_inlier_tolerance_m", 0.04);
    config_.min_cluster_cells = declare_parameter<int>("min_cluster_cells", 3);
    config_.min_ground_cells = declare_parameter<int>("min_ground_cells", 30);
    pixel_stride_ = declare_parameter<int>("pixel_stride", 4);
    max_rate_hz_ = declare_parameter<double>("max_rate_hz", 10.0);
    max_image_age_s_ = declare_parameter<double>("max_image_age_s", 0.3);
    min_valid_depth_fraction_ = declare_parameter<double>("min_valid_depth_fraction", 0.15);
    const std::string camera_name = declare_parameter<std::string>("camera_name", "zed");
    const std::string depth_topic = declare_parameter<std::string>(
        "depth_topic", "/" + camera_name + "/zed_node/depth/depth_registered");
    const std::string camera_info_topic = declare_parameter<std::string>(
        "camera_info_topic", "/" + camera_name + "/zed_node/depth/camera_info");
    if (calibration_id.empty() || camera_name.empty()) {
      throw std::invalid_argument("measured mount_calibration_id and camera_name are required");
    }
    ValidateTerrainConfig(config_);
    if (pixel_stride_ < 1 || !std::isfinite(max_rate_hz_) || max_rate_hz_ <= 0.0 ||
        max_rate_hz_ > 10.0 || !std::isfinite(max_image_age_s_) || max_image_age_s_ <= 0.0 ||
        !std::isfinite(min_valid_depth_fraction_) || min_valid_depth_fraction_ <= 0.0 ||
        min_valid_depth_fraction_ > 1.0) {
      throw std::invalid_argument("depth sampling, rate or coverage parameter is invalid");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    obstacle_publisher_ =
        create_publisher<PointCloud2>("/perception/depth_obstacles", rclcpp::SensorDataQoS());
    hazard_publisher_ =
        create_publisher<PointCloud2>("/perception/terrain_hazards", rclcpp::SensorDataQoS());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10).reliable());
    camera_info_subscription_ = create_subscription<CameraInfo>(
        camera_info_topic, rclcpp::SensorDataQoS(),
        [this](CameraInfo::ConstSharedPtr info) { latest_camera_info_ = std::move(info); });
    depth_subscription_ = create_subscription<Image>(
        depth_topic, rclcpp::SensorDataQoS(),
        [this](Image::ConstSharedPtr image) { ProcessDepth(std::move(image)); });
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      const auto age =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - last_valid_frame_)
              .count();
      if (age > max_image_age_s_) {
        PublishDiagnostics(DiagnosticStatus::STALE, "no recent valid depth frame");
      }
    });
    RCLCPP_INFO(get_logger(), "terrain hazard node ready for camera %s; calibration %s",
                camera_name.c_str(), calibration_id.c_str());
  }

private:
  void ProcessDepth(Image::ConstSharedPtr image) {
    const auto start = std::chrono::steady_clock::now();
    const rclcpp::Time stamp(image->header.stamp);
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() == 0 || !IsFreshDepthFrame(age, max_image_age_s_)) {
      PublishDiagnostics(DiagnosticStatus::STALE, "depth timestamp is stale or invalid");
      return;
    }
    if (last_processed_stamp_.nanoseconds() != 0 &&
        (stamp - last_processed_stamp_).seconds() < 1.0 / max_rate_hz_ - 0.001) {
      return;
    }
    last_processed_stamp_ = stamp;
    const auto& info = latest_camera_info_;
    if (!info || info->width != image->width || info->height != image->height ||
        info->header.frame_id != image->header.frame_id ||
        std::abs((rclcpp::Time(info->header.stamp) - stamp).seconds()) > 0.1 ||
        !std::isfinite(info->k[0]) || !std::isfinite(info->k[4]) || info->k[0] <= 0.0 ||
        info->k[4] <= 0.0) {
      PublishDiagnostics(DiagnosticStatus::ERROR, "depth CameraInfo is missing or mismatched");
      return;
    }
    const std::size_t bytes_per_pixel = image->encoding == "32FC1"   ? 4U
                                        : image->encoding == "16UC1" ? 2U
                                                                     : 0U;
    if (bytes_per_pixel == 0 || image->width == 0 || image->height == 0 ||
        image->step < image->width * bytes_per_pixel ||
        image->data.size() < static_cast<std::size_t>(image->step) * image->height) {
      PublishDiagnostics(DiagnosticStatus::ERROR, "depth image encoding or layout is invalid");
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("base_link", image->header.frame_id, stamp,
                                              rclcpp::Duration::from_seconds(0.03));
    } catch (const tf2::TransformException&) {
      PublishDiagnostics(DiagnosticStatus::ERROR, "measured camera-to-base_link TF unavailable");
      return;
    }
    std::vector<TerrainPoint> points;
    std::size_t sampled = 0;
    std::size_t valid = 0;
    try {
      const RigidTransform base_from_camera(transform);
      points.reserve(static_cast<std::size_t>(image->width / pixel_stride_ + 1) *
                     static_cast<std::size_t>(image->height / pixel_stride_ + 1));
      for (std::uint32_t row = 0; row < image->height; row += pixel_stride_) {
        for (std::uint32_t column = 0; column < image->width; column += pixel_stride_) {
          ++sampled;
          const auto offset = static_cast<std::size_t>(row) * image->step +
                              static_cast<std::size_t>(column) * bytes_per_pixel;
          const float depth = ReadDepth(*image, offset);
          if (!std::isfinite(depth) || depth <= 0.0F) {
            continue;
          }
          ++valid;
          const double x = (static_cast<double>(column) - info->k[2]) * depth / info->k[0];
          const double y = (static_cast<double>(row) - info->k[5]) * depth / info->k[4];
          points.push_back(base_from_camera.Apply(x, y, depth));
        }
      }
    } catch (const std::invalid_argument& error) {
      PublishDiagnostics(DiagnosticStatus::ERROR, error.what());
      return;
    }
    coverage_fraction_ = sampled == 0 ? 0.0 : static_cast<double>(valid) / sampled;
    if (coverage_fraction_ < min_valid_depth_fraction_) {
      PublishDiagnostics(DiagnosticStatus::ERROR, "insufficient valid depth coverage");
      return;
    }
    const TerrainResult result = DetectTerrain(points, config_);
    unobserved_cells_ = result.unobserved_ground_cells;
    ground_inlier_cells_ = result.ground_inlier_cells;
    if (!result.ground_valid) {
      PublishDiagnostics(DiagnosticStatus::ERROR,
                         "insufficient observed ground for hazard detection");
      return;
    }
    obstacle_publisher_->publish(MakeCloud(result.raised_obstacles, image->header));
    hazard_publisher_->publish(MakeCloud(result.terrain_hazards, image->header));
    last_valid_frame_ = std::chrono::steady_clock::now();
    processing_ms_ = std::chrono::duration<double, std::milli>(last_valid_frame_ - start).count();
    obstacle_count_ = result.raised_obstacles.size();
    hazard_count_ = result.terrain_hazards.size();
    PublishDiagnostics(unobserved_cells_ > 0 ? DiagnosticStatus::WARN : DiagnosticStatus::OK,
                       unobserved_cells_ > 0 ? "unobserved ground between returns"
                                             : "valid terrain frame");
  }

  void PublishDiagnostics(std::uint8_t level, const std::string& message) {
    if (level == DiagnosticStatus::ERROR || level == DiagnosticStatus::STALE) {
      obstacle_count_ = 0;
      hazard_count_ = 0;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    DiagnosticStatus status;
    status.level = level;
    status.name = "terrain_hazard_node";
    status.hardware_id = "zed_mini";
    status.message = message;
    AddDiagnosticValue(&status, "depth_coverage_fraction", std::to_string(coverage_fraction_));
    AddDiagnosticValue(&status, "unobserved_ground_cells", std::to_string(unobserved_cells_));
    AddDiagnosticValue(&status, "ground_inlier_cells", std::to_string(ground_inlier_cells_));
    AddDiagnosticValue(&status, "raised_cells", std::to_string(obstacle_count_));
    AddDiagnosticValue(&status, "hazard_cells", std::to_string(hazard_count_));
    AddDiagnosticValue(&status, "processing_ms", std::to_string(processing_ms_));
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  TerrainConfig config_;
  int pixel_stride_ = 4;
  double max_rate_hz_ = 10.0;
  double max_image_age_s_ = 0.3;
  double min_valid_depth_fraction_ = 0.15;
  double coverage_fraction_ = 0.0;
  double processing_ms_ = 0.0;
  std::size_t unobserved_cells_ = 0;
  std::size_t ground_inlier_cells_ = 0;
  std::size_t obstacle_count_ = 0;
  std::size_t hazard_count_ = 0;
  rclcpp::Time last_processed_stamp_{0, 0, RCL_SYSTEM_TIME};
  std::chrono::steady_clock::time_point last_valid_frame_ = std::chrono::steady_clock::now();
  CameraInfo::ConstSharedPtr latest_camera_info_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<PointCloud2>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<PointCloud2>::SharedPtr hazard_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<Image>::SharedPtr depth_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

} // namespace lb_sensors

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<lb_sensors::TerrainHazardNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("terrain_hazard_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
