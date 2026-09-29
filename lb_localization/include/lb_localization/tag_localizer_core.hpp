// Copyright 2026 JMU NASA Lunabotics Team
// SPDX-License-Identifier: MIT

#ifndef LB_LOCALIZATION__TAG_LOCALIZER_CORE_HPP_
#define LB_LOCALIZATION__TAG_LOCALIZER_CORE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <tf2/LinearMath/Transform.hpp>
#include <vector>

namespace lb_localization {

/// A detector-family/ID pair that identifies one physical AprilTag.
struct TagKey {
  std::string family;
  int id{};

  bool operator==(const TagKey& other) const;
  bool operator<(const TagKey& other) const;
};

/// A surveyed (or explicitly synthetic-test-only) tag pose in the map frame.
struct TagDefinition {
  TagKey key;
  double size_m{};
  std::string frame_id;
  tf2::Transform map_to_tag;
};

/// Parsed tag-map data.  Production use requires survey_status == "surveyed".
struct TagMap {
  std::string map_frame;
  std::string survey_status;
  std::string coordinate_convention;
  std::map<TagKey, TagDefinition> tags;

  static TagMap LoadFromFile(const std::string& path);
  const TagDefinition* Find(const TagKey& key) const;
};

/// Required quality gates from LUNA-SW-SPEC-001 section 6.3 plus explicit support gates.
struct QualityGates {
  double max_tag_distance_m{};
  double max_view_angle_rad{};
  double min_decision_margin{};
  int max_hamming{};
  double max_pose_jump_m{};
  double max_yaw_jump_rad{};
  double max_detection_age_s{};
  std::size_t minimum_valid_tags{};

  double min_tag_pixel_size_px{};
  double transform_timestamp_tolerance_s{};
  double max_future_skew_s{};
  double fusion_window_s{};
  double jump_reference_timeout_s{};
  double max_intertag_position_spread_m{};
  double max_intertag_yaw_spread_rad{};
};

/// Explicit, configurable covariance model.  Values are never hidden in code.
struct CovarianceModel {
  double position_stddev_floor_m{};
  double position_stddev_per_m{};
  double yaw_stddev_floor_rad{};
  double yaw_stddev_per_m{};
  double roll_pitch_stddev_rad{};
  double z_stddev_scale{};
  double distance_quality_gain{};
  double view_angle_quality_gain{};
  double reference_decision_margin{};
  double reference_tag_pixel_size_px{};
  /// Deliberately large covariance used for unobserved roll/pitch in the 2-D fused output.
  double unobserved_roll_pitch_variance_rad2{};
};

/// The source-specific detector and camera-TF contract.
struct SourceConfig {
  std::string source_id;
  std::string detections_topic;
  std::string expected_camera_frame;
  std::string observation_frame_prefix;
  std::string base_frame;
  std::string extrinsic_status;
  std::string extrinsic_authority;
};

/// All runtime configuration other than the tag map itself.
struct LocalizerConfig {
  std::string configuration_status;
  std::string map_frame;
  std::string base_frame;
  double tf_lookup_timeout_s{};
  QualityGates gates;
  CovarianceModel covariance_model;
  std::vector<SourceConfig> sources;

  static LocalizerConfig LoadFromFile(const std::string& path);
  const SourceConfig* FindSource(const std::string& source_id) const;
};

/// Reject unmeasured/synthetic configuration unless an explicit test-only opt-in is present.
void ValidateRuntimeInputs(const TagMap& tag_map, const LocalizerConfig& config,
                           bool allow_synthetic_test_data);

/// Canonicalize detector names such as "36h11" and "tag36h11" to "tag36h11".
std::string CanonicalTagFamily(const std::string& family);

/// Return a camera-specific detector observation-frame name for a tag ID.
std::string ObservationFrameFor(const SourceConfig& source, int tag_id);

/// Return true only for finite translation and normalized, finite quaternion data.
bool IsFiniteTransform(const tf2::Transform& transform);

/// Normalize an angle into [-pi, pi].
double WrapAngle(double angle_rad);

/// Extract REP-103 yaw from a transform rotation.
double YawOf(const tf2::Transform& transform);

/// Quality information derivable from an AprilTag detection message and its observation TF.
struct DetectionQuality {
  int hamming{};
  double decision_margin{};
  double tag_pixel_size_px{};
};

/// A map-frame pose candidate from one distinct physical-tag observation.
struct PoseCandidate {
  TagKey tag;
  std::string source_id;
  tf2::Transform map_to_base;
  double stamp_s{};
  std::int64_t stamp_nanoseconds{};
  double distance_m{};
  double view_angle_rad{};
  double decision_margin{};
  double tag_pixel_size_px{};
  std::array<double, 36> covariance{};
};

/// A fused two-dimensional absolute robot-pose measurement for the global EKF.
struct PoseEstimate {
  tf2::Transform map_to_base;
  std::array<double, 36> covariance{};
  double stamp_s{};
  std::int64_t stamp_nanoseconds{};
  std::size_t distinct_tag_count{};
  double max_position_residual_m{};
  double max_yaw_residual_rad{};
};

/// Build a candidate using map_T_base = map_T_tag * inverse(camera_T_tag) * inverse(base_T_camera).
/// Returns nullopt and fills reject_reason when a configured quality gate fails.
std::optional<PoseCandidate>
MakePoseCandidate(const TagDefinition& known_tag, const std::string& source_id,
                  const tf2::Transform& camera_to_tag, const tf2::Transform& base_to_camera,
                  const DetectionQuality& quality, const QualityGates& gates,
                  const CovarianceModel& covariance_model, std::string* reject_reason);

/// Fuse unique tags by inverse covariance.  Duplicate physical-tag observations are deduplicated.
/// Returns nullopt and fills reject_reason for minimum-tag/disagreement failures.
std::optional<PoseEstimate> FuseCandidates(const std::vector<PoseCandidate>& candidates,
                                           const QualityGates& gates,
                                           const CovarianceModel& covariance_model,
                                           std::string* reject_reason);

/// Check the configured jump gates against a recent previous accepted correction.
bool IsWithinJumpGate(const PoseEstimate& estimate, const std::optional<PoseEstimate>& previous,
                      const QualityGates& gates, std::string* reject_reason);

} // namespace lb_localization

#endif // LB_LOCALIZATION__TAG_LOCALIZER_CORE_HPP_
