// Copyright 2026 JMU NASA Lunabotics Team
// SPDX-License-Identifier: MIT

#include "lb_localization/tag_localizer_core.hpp"

#include <algorithm>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <functional>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <utility>
#include <vector>

namespace lb_localization {
namespace {

using DetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;

double ToSeconds(const rclcpp::Time& time) {
  return static_cast<double>(time.nanoseconds()) * 1.0e-9;
}

double MeanTagEdgeLengthPx(const apriltag_msgs::msg::AprilTagDetection& detection) {
  double total = 0.0;
  for (std::size_t index = 0; index < detection.corners.size(); ++index) {
    const auto& first = detection.corners[index];
    const auto& second = detection.corners[(index + 1U) % detection.corners.size()];
    if (!std::isfinite(first.x) || !std::isfinite(first.y) || !std::isfinite(second.x) ||
        !std::isfinite(second.y)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    total += std::hypot(second.x - first.x, second.y - first.y);
  }
  return total / static_cast<double>(detection.corners.size());
}

double CandidateScore(const PoseCandidate& candidate) {
  return candidate.covariance[0] + candidate.covariance[7] + candidate.covariance[35];
}

} // namespace

class TagLocalizer final : public rclcpp::Node {
public:
  explicit TagLocalizer(const rclcpp::NodeOptions& options) : Node("tag_localizer", options) {
    const std::string tag_map_file = declare_parameter<std::string>("tag_map_file", "");
    const std::string localizer_config_file =
        declare_parameter<std::string>("localizer_config_file", "");
    const bool allow_synthetic_test_data =
        declare_parameter<bool>("allow_synthetic_test_data", false);
    const std::string pose_topic =
        declare_parameter<std::string>("pose_topic", "/localization/apriltag_pose");
    const std::string diagnostics_topic =
        declare_parameter<std::string>("diagnostics_topic", "/diagnostics");

    if (tag_map_file.empty() || localizer_config_file.empty()) {
      throw std::runtime_error("tag_map_file and localizer_config_file must be explicit files; "
                               "the tag_localizer has no production defaults.");
    }
    tag_map_ = TagMap::LoadFromFile(tag_map_file);
    config_ = LocalizerConfig::LoadFromFile(localizer_config_file);
    ValidateRuntimeInputs(tag_map_, config_, allow_synthetic_test_data);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // A dedicated listener thread is required for nonzero tf2 lookup timeouts
    // to work on both the locked Humble target and the Jazzy development host.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, rclcpp::QoS(10).reliable());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        diagnostics_topic, rclcpp::QoS(10).reliable());

    for (const SourceConfig& source : config_.sources) {
      detection_subscriptions_.push_back(create_subscription<DetectionArray>(
          source.detections_topic, rclcpp::SensorDataQoS(),
          [this, source_id = source.source_id](DetectionArray::ConstSharedPtr message) {
            HandleDetections(source_id, std::move(message));
          }));
    }
    diagnostics_timer_ =
        create_wall_timer(std::chrono::seconds(1), [this]() { PublishDiagnostics("periodic"); });

    RCLCPP_INFO(
        get_logger(),
        "tag_localizer initialized with %zu source(s); publishes only absolute pose measurements "
        "and diagnostics, never map->odom TF.",
        config_.sources.size());
  }

private:
  void HandleDetections(const std::string& source_id, DetectionArray::ConstSharedPtr message) {
    const SourceConfig* source = config_.FindSource(source_id);
    if (source == nullptr) {
      RejectObservation("unknown_source_callback");
      return;
    }

    const rclcpp::Time now = this->now();
    HandleClockJump(now);
    PruneCandidates(now);
    const rclcpp::Time detection_stamp(message->header.stamp, get_clock()->get_clock_type());
    if (message->header.frame_id != source->expected_camera_frame) {
      RejectMessageOrDetections(message->detections.size(), "source_header_frame_mismatch");
      PublishDiagnostics("source_header_frame_mismatch");
      return;
    }
    if (!IsCurrentDetectionStamp(detection_stamp, now)) {
      RejectMessageOrDetections(message->detections.size(), "stale_or_future_detection_stamp");
      PublishDiagnostics("stale_or_future_detection_stamp");
      return;
    }

    bool accepted_in_callback = false;
    for (const apriltag_msgs::msg::AprilTagDetection& detection : message->detections) {
      const TagKey key{CanonicalTagFamily(detection.family), detection.id};
      const TagDefinition* known_tag = tag_map_.Find(key);
      if (known_tag == nullptr) {
        RejectObservation("unknown_family_or_id");
        continue;
      }

      const std::string observation_frame = ObservationFrameFor(*source, detection.id);
      geometry_msgs::msg::TransformStamped camera_to_tag_message;
      geometry_msgs::msg::TransformStamped base_to_camera_message;
      try {
        camera_to_tag_message = tf_buffer_->lookupTransform(
            source->expected_camera_frame, observation_frame, detection_stamp,
            rclcpp::Duration::from_seconds(config_.tf_lookup_timeout_s));
        base_to_camera_message = tf_buffer_->lookupTransform(
            source->base_frame, source->expected_camera_frame, detection_stamp,
            rclcpp::Duration::from_seconds(config_.tf_lookup_timeout_s));
      } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Rejecting tag observation because exact-time TF lookup failed: %s",
                             error.what());
        RejectObservation("missing_exact_time_tf");
        continue;
      }

      if (camera_to_tag_message.header.frame_id != source->expected_camera_frame ||
          camera_to_tag_message.child_frame_id != observation_frame) {
        RejectObservation("observation_tf_frame_mismatch");
        continue;
      }
      if (base_to_camera_message.header.frame_id != source->base_frame ||
          base_to_camera_message.child_frame_id != source->expected_camera_frame) {
        RejectObservation("camera_extrinsic_tf_frame_mismatch");
        continue;
      }
      if (!IsFreshObservationTransform(camera_to_tag_message, detection_stamp)) {
        RejectObservation("stale_observation_tf");
        continue;
      }
      if (!IsFreshOrStaticExtrinsic(base_to_camera_message, detection_stamp)) {
        RejectObservation("stale_camera_extrinsic_tf");
        continue;
      }

      tf2::Transform camera_to_tag;
      tf2::Transform base_to_camera;
      tf2::fromMsg(camera_to_tag_message.transform, camera_to_tag);
      tf2::fromMsg(base_to_camera_message.transform, base_to_camera);
      const DetectionQuality quality{detection.hamming,
                                     static_cast<double>(detection.decision_margin),
                                     MeanTagEdgeLengthPx(detection)};
      std::string reject_reason;
      std::optional<PoseCandidate> candidate =
          MakePoseCandidate(*known_tag, source->source_id, camera_to_tag, base_to_camera, quality,
                            config_.gates, config_.covariance_model, &reject_reason);
      if (!candidate.has_value()) {
        RejectObservation(reject_reason);
        continue;
      }
      candidate->stamp_s = ToSeconds(detection_stamp);
      candidate->stamp_nanoseconds = detection_stamp.nanoseconds();
      StoreCandidate(*candidate);
      ++accepted_observation_count_;
      accepted_in_callback = true;
    }

    if (accepted_in_callback) {
      TryPublishFusedPose();
    } else if (message->detections.empty()) {
      last_fusion_status_ = "no_detections";
      PublishDiagnostics(last_fusion_status_);
    }
  }

  bool IsCurrentDetectionStamp(const rclcpp::Time& stamp, const rclcpp::Time& now) const {
    const double age_s = (now - stamp).seconds();
    return age_s <= config_.gates.max_detection_age_s && age_s >= -config_.gates.max_future_skew_s;
  }

  bool IsFreshObservationTransform(const geometry_msgs::msg::TransformStamped& transform,
                                   const rclcpp::Time& detection_stamp) const {
    const rclcpp::Time transform_stamp(transform.header.stamp, get_clock()->get_clock_type());
    return transform_stamp.nanoseconds() != 0 &&
           std::abs((transform_stamp - detection_stamp).seconds()) <=
               config_.gates.transform_timestamp_tolerance_s;
  }

  bool IsFreshOrStaticExtrinsic(const geometry_msgs::msg::TransformStamped& transform,
                                const rclcpp::Time& detection_stamp) const {
    const rclcpp::Time transform_stamp(transform.header.stamp, get_clock()->get_clock_type());
    return transform_stamp.nanoseconds() == 0 ||
           std::abs((transform_stamp - detection_stamp).seconds()) <=
               config_.gates.transform_timestamp_tolerance_s;
  }

  void StoreCandidate(const PoseCandidate& candidate) {
    const auto iterator = candidate_cache_.find(candidate.tag);
    if (iterator == candidate_cache_.end()) {
      candidate_cache_.emplace(candidate.tag, candidate);
      return;
    }
    const PoseCandidate& incumbent = iterator->second;
    const bool is_newer =
        candidate.stamp_s > incumbent.stamp_s + config_.gates.transform_timestamp_tolerance_s;
    if (is_newer || CandidateScore(candidate) < CandidateScore(incumbent)) {
      iterator->second = candidate;
    }
  }

  void PruneCandidates(const rclcpp::Time& now) {
    const double now_s = ToSeconds(now);
    for (auto iterator = candidate_cache_.begin(); iterator != candidate_cache_.end();) {
      if (now_s - iterator->second.stamp_s > config_.gates.fusion_window_s) {
        iterator = candidate_cache_.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }

  void TryPublishFusedPose() {
    std::vector<PoseCandidate> candidates;
    candidates.reserve(candidate_cache_.size());
    for (const auto& key_and_candidate : candidate_cache_) {
      candidates.push_back(key_and_candidate.second);
    }
    std::string reject_reason;
    std::optional<PoseEstimate> estimate =
        FuseCandidates(candidates, config_.gates, config_.covariance_model, &reject_reason);
    if (!estimate.has_value()) {
      RejectFusion(reject_reason);
      PublishDiagnostics(reject_reason);
      return;
    }
    if (!IsWithinJumpGate(*estimate, last_accepted_pose_, config_.gates, &reject_reason)) {
      RejectFusion(reject_reason);
      PublishDiagnostics(reject_reason);
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = rclcpp::Time(estimate->stamp_nanoseconds, get_clock()->get_clock_type());
    pose.header.frame_id = tag_map_.map_frame;
    tf2::toMsg(estimate->map_to_base, pose.pose.pose);
    pose.pose.covariance = estimate->covariance;
    pose_publisher_->publish(pose);
    last_accepted_pose_ = *estimate;
    last_fusion_status_ = "published";
    PublishDiagnostics(last_fusion_status_);
  }

  void HandleClockJump(const rclcpp::Time& now) {
    if (last_clock_time_.has_value() && now < *last_clock_time_) {
      candidate_cache_.clear();
      last_accepted_pose_.reset();
      last_fusion_status_ = "ros_time_moved_backward_reset";
      RCLCPP_WARN(get_logger(),
                  "ROS time moved backward; cleared pending tag observations and jump reference.");
    }
    last_clock_time_ = now;
  }

  void RejectMessageOrDetections(const std::size_t detection_count, const std::string& reason) {
    const std::size_t count = std::max<std::size_t>(1U, detection_count);
    for (std::size_t index = 0; index < count; ++index) {
      RejectObservation(reason);
    }
  }

  void RejectObservation(const std::string& reason) {
    ++rejected_observation_count_;
    ++rejection_counts_[reason.empty() ? "unspecified_observation_rejection" : reason];
    last_fusion_status_ = reason.empty() ? "unspecified_observation_rejection" : reason;
  }

  void RejectFusion(const std::string& reason) {
    ++rejected_fusion_count_;
    const std::string normalized_reason = reason.empty() ? "unspecified_fusion_rejection" : reason;
    ++rejection_counts_["fusion." + normalized_reason];
    last_fusion_status_ = normalized_reason;
  }

  void PublishDiagnostics(const std::string& event) {
    if (!diagnostics_publisher_) {
      return;
    }
    const rclcpp::Time now = this->now();
    HandleClockJump(now);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "lb_localization/tag_localizer";
    status.hardware_id = "lb_localization";
    status.message = event;

    const double last_valid_age_s =
        last_accepted_pose_.has_value()
            ? std::max(0.0, ToSeconds(now) - last_accepted_pose_->stamp_s)
            : std::numeric_limits<double>::infinity();
    if (!last_accepted_pose_.has_value() || last_valid_age_s > config_.gates.max_detection_age_s) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }

    AddDiagnosticValue(&status, "accepted_observations", accepted_observation_count_);
    AddDiagnosticValue(&status, "rejected_observations", rejected_observation_count_);
    AddDiagnosticValue(&status, "rejected_fusions", rejected_fusion_count_);
    AddDiagnosticValue(&status, "cached_unique_tags",
                       static_cast<std::uint64_t>(candidate_cache_.size()));
    AddDiagnosticValue(&status, "last_valid_correction_age_s",
                       std::isfinite(last_valid_age_s) ? ToString(last_valid_age_s)
                                                       : std::string("never"));
    AddDiagnosticValue(&status, "last_fusion_status", last_fusion_status_);
    for (const auto& reason_and_count : rejection_counts_) {
      AddDiagnosticValue(&status, "rejections." + reason_and_count.first, reason_and_count.second);
    }
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  static std::string ToString(const std::uint64_t value) {
    return std::to_string(value);
  }

  static std::string ToString(const double value) {
    std::ostringstream stream;
    stream.precision(6);
    stream << std::fixed << value;
    return stream.str();
  }

  static void AddDiagnosticValue(diagnostic_msgs::msg::DiagnosticStatus* status,
                                 const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status->values.push_back(std::move(item));
  }

  static void AddDiagnosticValue(diagnostic_msgs::msg::DiagnosticStatus* status,
                                 const std::string& key, const std::uint64_t value) {
    AddDiagnosticValue(status, key, ToString(value));
  }

  TagMap tag_map_;
  LocalizerConfig config_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  std::vector<rclcpp::Subscription<DetectionArray>::SharedPtr> detection_subscriptions_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::map<TagKey, PoseCandidate> candidate_cache_;
  std::optional<PoseEstimate> last_accepted_pose_;
  std::optional<rclcpp::Time> last_clock_time_;
  std::map<std::string, std::uint64_t> rejection_counts_;
  std::uint64_t accepted_observation_count_{};
  std::uint64_t rejected_observation_count_{};
  std::uint64_t rejected_fusion_count_{};
  std::string last_fusion_status_{"never_received_detection"};
};

} // namespace lb_localization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<lb_localization::TagLocalizer>(rclcpp::NodeOptions{}));
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("tag_localizer"), "Failing closed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
