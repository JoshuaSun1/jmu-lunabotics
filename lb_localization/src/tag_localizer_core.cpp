// Copyright 2026 JMU NASA Lunabotics Team
// SPDX-License-Identifier: MIT

#include "lb_localization/tag_localizer_core.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tf2/utils.h>
#include <utility>
#include <yaml-cpp/yaml.h>

namespace lb_localization {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumVariance = 1.0e-12;

[[noreturn]] void ConfigurationError(const std::string& message) {
  throw std::runtime_error("tag_localizer configuration error: " + message);
}

void Require(const bool condition, const std::string& message) {
  if (!condition) {
    ConfigurationError(message);
  }
}

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool IsRecognizedStatus(const std::string& status) {
  return status == "surveyed" || status == "synthetic_test_only" || status == "unverified";
}

bool IsSupportedCoordinateConvention(const std::string& convention) {
  return convention == "REP-103";
}

YAML::Node RequiredNode(const YAML::Node& parent, const std::string& name) {
  const YAML::Node child = parent[name];
  Require(child.IsDefined(), "missing required field '" + name + "'");
  return child;
}

std::string ReadString(const YAML::Node& parent, const std::string& name) {
  const YAML::Node node = RequiredNode(parent, name);
  Require(node.IsScalar(), "field '" + name + "' must be a scalar string");
  try {
    const std::string value = node.as<std::string>();
    Require(!value.empty(), "field '" + name + "' must not be empty");
    Require(value != "TBD", "field '" + name + "' must not be TBD");
    return value;
  } catch (const YAML::Exception& error) {
    ConfigurationError("field '" + name + "' is not a string: " + error.what());
  }
}

double ReadFiniteDouble(const YAML::Node& parent, const std::string& name) {
  const YAML::Node node = RequiredNode(parent, name);
  Require(node.IsScalar(), "field '" + name + "' must be a finite number");
  try {
    const double value = node.as<double>();
    Require(IsFinite(value), "field '" + name + "' must be finite");
    return value;
  } catch (const YAML::Exception& error) {
    ConfigurationError("field '" + name + "' is not numeric: " + error.what());
  }
}

int ReadInteger(const YAML::Node& parent, const std::string& name) {
  const YAML::Node node = RequiredNode(parent, name);
  Require(node.IsScalar(), "field '" + name + "' must be an integer");
  try {
    return node.as<int>();
  } catch (const YAML::Exception& error) {
    ConfigurationError("field '" + name + "' is not an integer: " + error.what());
  }
}

std::size_t ReadPositiveSize(const YAML::Node& parent, const std::string& name) {
  const int value = ReadInteger(parent, name);
  Require(value > 0, "field '" + name + "' must be greater than zero");
  return static_cast<std::size_t>(value);
}

tf2::Transform ReadTransform(const YAML::Node& parent, const std::string& name) {
  const YAML::Node pose = RequiredNode(parent, name);
  Require(pose.IsMap(), "field '" + name + "' must be a pose mapping");
  const double x = ReadFiniteDouble(pose, "x");
  const double y = ReadFiniteDouble(pose, "y");
  const double z = ReadFiniteDouble(pose, "z");
  const double roll = ReadFiniteDouble(pose, "roll");
  const double pitch = ReadFiniteDouble(pose, "pitch");
  const double yaw = ReadFiniteDouble(pose, "yaw");

  tf2::Quaternion rotation;
  rotation.setRPY(roll, pitch, yaw);
  rotation.normalize();
  return tf2::Transform(rotation, tf2::Vector3(x, y, z));
}

tf2::Transform NormalizedTransform(const tf2::Transform& transform) {
  tf2::Quaternion rotation = transform.getRotation();
  rotation.normalize();
  return tf2::Transform(rotation, transform.getOrigin());
}

double InverseVariance(const double variance) {
  return 1.0 / std::max(variance, kMinimumVariance);
}

double WeightedVariance(const std::vector<std::pair<double, double>>& values_and_weights,
                        const double weighted_mean, const bool circular) {
  double numerator = 0.0;
  double denominator = 0.0;
  for (const auto& value_and_weight : values_and_weights) {
    const double residual = circular ? WrapAngle(value_and_weight.first - weighted_mean)
                                     : value_and_weight.first - weighted_mean;
    numerator += value_and_weight.second * residual * residual;
    denominator += value_and_weight.second;
  }
  return denominator > 0.0 ? numerator / denominator : 0.0;
}

bool IsBetterDuplicate(const PoseCandidate& candidate, const PoseCandidate& incumbent) {
  const double candidate_variance =
      candidate.covariance[0] + candidate.covariance[7] + candidate.covariance[35];
  const double incumbent_variance =
      incumbent.covariance[0] + incumbent.covariance[7] + incumbent.covariance[35];
  if (candidate_variance != incumbent_variance) {
    return candidate_variance < incumbent_variance;
  }
  return candidate.decision_margin > incumbent.decision_margin;
}

void SetDiagonalCovariance(std::array<double, 36>* covariance, const double x_variance,
                           const double y_variance, const double z_variance,
                           const double roll_variance, const double pitch_variance,
                           const double yaw_variance) {
  covariance->fill(0.0);
  (*covariance)[0] = x_variance;
  (*covariance)[7] = y_variance;
  (*covariance)[14] = z_variance;
  (*covariance)[21] = roll_variance;
  (*covariance)[28] = pitch_variance;
  (*covariance)[35] = yaw_variance;
}

} // namespace

bool TagKey::operator==(const TagKey& other) const {
  return family == other.family && id == other.id;
}

bool TagKey::operator<(const TagKey& other) const {
  return family == other.family ? id < other.id : family < other.family;
}

std::string CanonicalTagFamily(const std::string& family) {
  std::string normalized;
  normalized.reserve(family.size() + 3U);
  for (const char character : family) {
    if (character != ' ' && character != '_' && character != '-') {
      normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
    }
  }
  if (normalized.rfind("tag", 0) != 0U) {
    normalized = "tag" + normalized;
  }
  return normalized;
}

TagMap TagMap::LoadFromFile(const std::string& path) {
  try {
    const YAML::Node root = YAML::LoadFile(path);
    Require(root.IsMap(), "tag map root must be a mapping");
    Require(ReadInteger(root, "schema_version") == 1, "unsupported tag-map schema_version");

    const YAML::Node map_node = RequiredNode(root, "map");
    Require(map_node.IsMap(), "field 'map' must be a mapping");
    TagMap tag_map;
    tag_map.map_frame = ReadString(map_node, "frame_id");
    tag_map.survey_status = ReadString(map_node, "survey_status");
    tag_map.coordinate_convention = ReadString(map_node, "coordinate_convention");
    Require(IsRecognizedStatus(tag_map.survey_status), "unrecognized tag-map survey_status");
    Require(tag_map.map_frame == "map", "tag-map frame_id must be 'map'");
    Require(IsSupportedCoordinateConvention(tag_map.coordinate_convention),
            "tag-map coordinate_convention must be 'REP-103'");

    const YAML::Node tags = RequiredNode(root, "tags");
    Require(tags.IsSequence() && tags.size() > 0U, "field 'tags' must be a nonempty sequence");
    std::set<std::string> frame_ids;
    for (const YAML::Node& node : tags) {
      Require(node.IsMap(), "each tag record must be a mapping");
      TagDefinition definition;
      definition.key.family = CanonicalTagFamily(ReadString(node, "family"));
      Require(definition.key.family != "tag", "tag family must not be empty");
      Require(definition.key.family == "tag36h11",
              "only the selected tag36h11 field family is accepted");
      definition.key.id = ReadInteger(node, "id");
      Require(definition.key.id > 0,
              "field tag id must be positive; bench-only ID 0 is not a field landmark");
      definition.size_m = ReadFiniteDouble(node, "size_m");
      Require(definition.size_m > 0.0, "tag size_m must be greater than zero");
      definition.frame_id = ReadString(node, "frame_id");
      definition.map_to_tag = ReadTransform(node, "pose_map");
      Require(tag_map.tags.emplace(definition.key, definition).second,
              "tag map contains a duplicate family/id record");
      Require(frame_ids.emplace(definition.frame_id).second,
              "tag map contains a duplicate frame_id");
    }
    return tag_map;
  } catch (const YAML::Exception& error) {
    ConfigurationError("could not load tag map '" + path + "': " + error.what());
  }
}

const TagDefinition* TagMap::Find(const TagKey& key) const {
  const auto iterator = tags.find(key);
  return iterator == tags.end() ? nullptr : &iterator->second;
}

LocalizerConfig LocalizerConfig::LoadFromFile(const std::string& path) {
  try {
    const YAML::Node root = YAML::LoadFile(path);
    Require(root.IsMap(), "localizer configuration root must be a mapping");
    Require(ReadInteger(root, "schema_version") == 1, "unsupported localizer schema_version");

    LocalizerConfig config;
    config.configuration_status = ReadString(root, "configuration_status");
    Require(IsRecognizedStatus(config.configuration_status), "unrecognized configuration_status");
    config.map_frame = ReadString(root, "map_frame");
    config.base_frame = ReadString(root, "base_frame");
    config.tf_lookup_timeout_s = ReadFiniteDouble(root, "tf_lookup_timeout_s");
    Require(config.tf_lookup_timeout_s > 0.0, "tf_lookup_timeout_s must be greater than zero");

    const YAML::Node gates = RequiredNode(root, "quality_gates");
    Require(gates.IsMap(), "field 'quality_gates' must be a mapping");
    config.gates.max_tag_distance_m = ReadFiniteDouble(gates, "max_tag_distance_m");
    const double max_view_angle_deg = ReadFiniteDouble(gates, "max_view_angle_deg");
    config.gates.max_view_angle_rad = max_view_angle_deg * kPi / 180.0;
    config.gates.min_decision_margin = ReadFiniteDouble(gates, "min_decision_margin");
    config.gates.max_hamming = ReadInteger(gates, "max_hamming");
    config.gates.max_pose_jump_m = ReadFiniteDouble(gates, "max_pose_jump_m");
    config.gates.max_yaw_jump_rad = ReadFiniteDouble(gates, "max_yaw_jump_rad");
    config.gates.max_detection_age_s = ReadFiniteDouble(gates, "max_detection_age_s");
    config.gates.minimum_valid_tags = ReadPositiveSize(gates, "minimum_valid_tags");
    config.gates.min_tag_pixel_size_px = ReadFiniteDouble(gates, "min_tag_pixel_size_px");
    config.gates.transform_timestamp_tolerance_s =
        ReadFiniteDouble(gates, "transform_timestamp_tolerance_s");
    config.gates.max_future_skew_s = ReadFiniteDouble(gates, "max_future_skew_s");
    config.gates.fusion_window_s = ReadFiniteDouble(gates, "fusion_window_s");
    config.gates.jump_reference_timeout_s = ReadFiniteDouble(gates, "jump_reference_timeout_s");
    config.gates.max_intertag_position_spread_m =
        ReadFiniteDouble(gates, "max_intertag_position_spread_m");
    config.gates.max_intertag_yaw_spread_rad =
        ReadFiniteDouble(gates, "max_intertag_yaw_spread_rad");

    Require(config.gates.max_tag_distance_m > 0.0, "max_tag_distance_m must be greater than zero");
    Require(config.gates.max_view_angle_rad > 0.0 && config.gates.max_view_angle_rad < kPi / 2.0,
            "max_view_angle_deg must be between zero and 90");
    Require(config.gates.min_decision_margin >= 0.0, "min_decision_margin must be nonnegative");
    Require(config.gates.max_hamming >= 0, "max_hamming must be nonnegative");
    Require(config.gates.max_pose_jump_m > 0.0, "max_pose_jump_m must be greater than zero");
    Require(config.gates.max_yaw_jump_rad > 0.0, "max_yaw_jump_rad must be greater than zero");
    Require(config.gates.max_detection_age_s > 0.0,
            "max_detection_age_s must be greater than zero");
    Require(config.gates.min_tag_pixel_size_px > 0.0,
            "min_tag_pixel_size_px must be greater than zero");
    Require(config.gates.transform_timestamp_tolerance_s >= 0.0,
            "transform_timestamp_tolerance_s must be nonnegative");
    Require(config.gates.max_future_skew_s >= 0.0, "max_future_skew_s must be nonnegative");
    Require(config.gates.fusion_window_s > 0.0, "fusion_window_s must be greater than zero");
    Require(config.gates.jump_reference_timeout_s >= config.gates.fusion_window_s,
            "jump_reference_timeout_s must be at least fusion_window_s");
    Require(config.gates.max_intertag_position_spread_m > 0.0,
            "max_intertag_position_spread_m must be greater than zero");
    Require(config.gates.max_intertag_yaw_spread_rad > 0.0,
            "max_intertag_yaw_spread_rad must be greater than zero");

    const YAML::Node covariance = RequiredNode(root, "covariance_model");
    Require(covariance.IsMap(), "field 'covariance_model' must be a mapping");
    config.covariance_model.position_stddev_floor_m =
        ReadFiniteDouble(covariance, "position_stddev_floor_m");
    config.covariance_model.position_stddev_per_m =
        ReadFiniteDouble(covariance, "position_stddev_per_m");
    config.covariance_model.yaw_stddev_floor_rad =
        ReadFiniteDouble(covariance, "yaw_stddev_floor_rad");
    config.covariance_model.yaw_stddev_per_m = ReadFiniteDouble(covariance, "yaw_stddev_per_m");
    config.covariance_model.roll_pitch_stddev_rad =
        ReadFiniteDouble(covariance, "roll_pitch_stddev_rad");
    config.covariance_model.z_stddev_scale = ReadFiniteDouble(covariance, "z_stddev_scale");
    config.covariance_model.distance_quality_gain =
        ReadFiniteDouble(covariance, "distance_quality_gain");
    config.covariance_model.view_angle_quality_gain =
        ReadFiniteDouble(covariance, "view_angle_quality_gain");
    config.covariance_model.reference_decision_margin =
        ReadFiniteDouble(covariance, "reference_decision_margin");
    config.covariance_model.reference_tag_pixel_size_px =
        ReadFiniteDouble(covariance, "reference_tag_pixel_size_px");
    config.covariance_model.unobserved_roll_pitch_variance_rad2 =
        ReadFiniteDouble(covariance, "unobserved_roll_pitch_variance_rad2");

    Require(config.covariance_model.position_stddev_floor_m > 0.0,
            "position_stddev_floor_m must be greater than zero");
    Require(config.covariance_model.position_stddev_per_m >= 0.0,
            "position_stddev_per_m must be nonnegative");
    Require(config.covariance_model.yaw_stddev_floor_rad > 0.0,
            "yaw_stddev_floor_rad must be greater than zero");
    Require(config.covariance_model.yaw_stddev_per_m >= 0.0,
            "yaw_stddev_per_m must be nonnegative");
    Require(config.covariance_model.roll_pitch_stddev_rad > 0.0,
            "roll_pitch_stddev_rad must be greater than zero");
    Require(config.covariance_model.z_stddev_scale > 0.0,
            "z_stddev_scale must be greater than zero");
    Require(config.covariance_model.distance_quality_gain >= 0.0,
            "distance_quality_gain must be nonnegative");
    Require(config.covariance_model.view_angle_quality_gain >= 0.0,
            "view_angle_quality_gain must be nonnegative");
    Require(config.covariance_model.reference_decision_margin > 0.0,
            "reference_decision_margin must be greater than zero");
    Require(config.covariance_model.reference_tag_pixel_size_px > 0.0,
            "reference_tag_pixel_size_px must be greater than zero");
    Require(config.covariance_model.unobserved_roll_pitch_variance_rad2 > 0.0,
            "unobserved_roll_pitch_variance_rad2 must be greater than zero");

    const YAML::Node sources = RequiredNode(root, "sources");
    Require(sources.IsSequence() && sources.size() > 0U,
            "field 'sources' must be a nonempty sequence");
    std::set<std::string> source_ids;
    std::set<std::string> topics;
    std::set<std::string> prefixes;
    for (const YAML::Node& node : sources) {
      Require(node.IsMap(), "each source record must be a mapping");
      SourceConfig source;
      source.source_id = ReadString(node, "source_id");
      source.detections_topic = ReadString(node, "detections_topic");
      source.expected_camera_frame = ReadString(node, "expected_camera_frame");
      source.observation_frame_prefix = ReadString(node, "observation_frame_prefix");
      source.base_frame = ReadString(node, "base_frame");
      source.extrinsic_status = ReadString(node, "extrinsic_status");
      source.extrinsic_authority = ReadString(node, "extrinsic_authority");
      Require(IsRecognizedStatus(source.extrinsic_status), "unrecognized source extrinsic_status");
      Require(source.base_frame == config.base_frame,
              "source base_frame must match top-level base_frame");
      Require(source_ids.emplace(source.source_id).second, "source_id must be unique");
      Require(topics.emplace(source.detections_topic).second, "detections_topic must be unique");
      Require(prefixes.emplace(source.observation_frame_prefix).second,
              "observation_frame_prefix must be unique across sources");
      config.sources.push_back(source);
    }
    return config;
  } catch (const YAML::Exception& error) {
    ConfigurationError("could not load localizer configuration '" + path + "': " + error.what());
  }
}

const SourceConfig* LocalizerConfig::FindSource(const std::string& source_id) const {
  const auto iterator =
      std::find_if(sources.begin(), sources.end(), [&source_id](const SourceConfig& source) {
        return source.source_id == source_id;
      });
  return iterator == sources.end() ? nullptr : &(*iterator);
}

void ValidateRuntimeInputs(const TagMap& tag_map, const LocalizerConfig& config,
                           const bool allow_synthetic_test_data) {
  Require(config.map_frame == tag_map.map_frame, "localizer map_frame must match tag-map frame_id");
  const bool sources_surveyed =
      std::all_of(config.sources.begin(), config.sources.end(),
                  [](const SourceConfig& source) { return source.extrinsic_status == "surveyed"; });
  if (tag_map.survey_status == "surveyed" && config.configuration_status == "surveyed" &&
      sources_surveyed) {
    return;
  }

  const bool sources_synthetic =
      std::all_of(config.sources.begin(), config.sources.end(), [](const SourceConfig& source) {
        return source.extrinsic_status == "synthetic_test_only";
      });
  if (allow_synthetic_test_data && tag_map.survey_status == "synthetic_test_only" &&
      config.configuration_status == "synthetic_test_only" && sources_synthetic) {
    return;
  }

  ConfigurationError(
      "runtime requires surveyed tag poses and camera extrinsics. Synthetic fixtures require "
      "allow_synthetic_test_data:=true and all inputs marked synthetic_test_only.");
}

std::string ObservationFrameFor(const SourceConfig& source, const int tag_id) {
  return source.observation_frame_prefix + std::to_string(tag_id);
}

bool IsFiniteTransform(const tf2::Transform& transform) {
  const tf2::Vector3 origin = transform.getOrigin();
  const tf2::Quaternion rotation = transform.getRotation();
  return IsFinite(origin.x()) && IsFinite(origin.y()) && IsFinite(origin.z()) &&
         IsFinite(rotation.x()) && IsFinite(rotation.y()) && IsFinite(rotation.z()) &&
         IsFinite(rotation.w()) && rotation.length2() > kMinimumVariance;
}

double WrapAngle(double angle_rad) {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double YawOf(const tf2::Transform& transform) {
  const tf2::Quaternion rotation = NormalizedTransform(transform).getRotation();
  return tf2::getYaw(rotation);
}

std::optional<PoseCandidate>
MakePoseCandidate(const TagDefinition& known_tag, const std::string& source_id,
                  const tf2::Transform& camera_to_tag, const tf2::Transform& base_to_camera,
                  const DetectionQuality& quality, const QualityGates& gates,
                  const CovarianceModel& covariance_model, std::string* reject_reason) {
  const auto reject = [reject_reason](const std::string& reason) {
    if (reject_reason != nullptr) {
      *reject_reason = reason;
    }
    return std::optional<PoseCandidate>{};
  };

  if (!IsFiniteTransform(known_tag.map_to_tag) || !IsFiniteTransform(camera_to_tag) ||
      !IsFiniteTransform(base_to_camera)) {
    return reject("nonfinite_transform");
  }
  if (!IsFinite(quality.decision_margin) || !IsFinite(quality.tag_pixel_size_px)) {
    return reject("nonfinite_detection_quality");
  }
  if (quality.hamming > gates.max_hamming) {
    return reject("max_hamming");
  }
  if (quality.decision_margin < gates.min_decision_margin) {
    return reject("min_decision_margin");
  }
  if (quality.tag_pixel_size_px < gates.min_tag_pixel_size_px) {
    return reject("min_tag_pixel_size_px");
  }

  const tf2::Transform normalized_camera_to_tag = NormalizedTransform(camera_to_tag);
  const double distance_m = normalized_camera_to_tag.getOrigin().length();
  if (!IsFinite(distance_m) || distance_m > gates.max_tag_distance_m) {
    return reject("max_tag_distance_m");
  }
  const tf2::Vector3 tag_normal = normalized_camera_to_tag.getBasis() * tf2::Vector3(0.0, 0.0, 1.0);
  if (tag_normal.length2() <= kMinimumVariance) {
    return reject("invalid_tag_normal");
  }
  const double normal_z = std::clamp(std::abs(tag_normal.normalized().z()), 0.0, 1.0);
  const double view_angle_rad = std::acos(normal_z);
  if (!IsFinite(view_angle_rad) || view_angle_rad > gates.max_view_angle_rad) {
    return reject("max_view_angle_deg");
  }

  const double cosine = std::max(std::cos(view_angle_rad), 1.0e-6);
  const double margin_factor =
      std::max(1.0, covariance_model.reference_decision_margin / quality.decision_margin);
  const double pixel_factor =
      std::max(1.0, covariance_model.reference_tag_pixel_size_px / quality.tag_pixel_size_px);
  const double distance_factor = 1.0 + covariance_model.distance_quality_gain * distance_m;
  const double angle_factor =
      1.0 + covariance_model.view_angle_quality_gain * ((1.0 / cosine) - 1.0);
  const double quality_factor = distance_factor * angle_factor * margin_factor * pixel_factor;
  const double position_stddev = (covariance_model.position_stddev_floor_m +
                                  covariance_model.position_stddev_per_m * distance_m) *
                                 quality_factor;
  const double yaw_stddev =
      (covariance_model.yaw_stddev_floor_rad + covariance_model.yaw_stddev_per_m * distance_m) *
      quality_factor;
  const double roll_pitch_stddev = covariance_model.roll_pitch_stddev_rad * quality_factor;
  if (!IsFinite(position_stddev) || !IsFinite(yaw_stddev) || !IsFinite(roll_pitch_stddev) ||
      position_stddev <= 0.0 || yaw_stddev <= 0.0 || roll_pitch_stddev <= 0.0) {
    return reject("invalid_covariance_model_result");
  }

  PoseCandidate candidate;
  candidate.tag = known_tag.key;
  candidate.source_id = source_id;
  candidate.map_to_base =
      NormalizedTransform(known_tag.map_to_tag * normalized_camera_to_tag.inverse() *
                          NormalizedTransform(base_to_camera).inverse());
  candidate.distance_m = distance_m;
  candidate.view_angle_rad = view_angle_rad;
  candidate.decision_margin = quality.decision_margin;
  candidate.tag_pixel_size_px = quality.tag_pixel_size_px;
  SetDiagonalCovariance(&candidate.covariance, position_stddev * position_stddev,
                        position_stddev * position_stddev,
                        std::pow(covariance_model.z_stddev_scale * position_stddev, 2.0),
                        roll_pitch_stddev * roll_pitch_stddev,
                        roll_pitch_stddev * roll_pitch_stddev, yaw_stddev * yaw_stddev);
  return candidate;
}

std::optional<PoseEstimate> FuseCandidates(const std::vector<PoseCandidate>& candidates,
                                           const QualityGates& gates,
                                           const CovarianceModel& covariance_model,
                                           std::string* reject_reason) {
  const auto reject = [reject_reason](const std::string& reason) {
    if (reject_reason != nullptr) {
      *reject_reason = reason;
    }
    return std::optional<PoseEstimate>{};
  };

  std::map<TagKey, PoseCandidate> unique_candidates;
  for (const PoseCandidate& candidate : candidates) {
    if (!IsFiniteTransform(candidate.map_to_base)) {
      return reject("nonfinite_candidate_transform");
    }
    if (!IsFinite(candidate.covariance[0]) || !IsFinite(candidate.covariance[7]) ||
        !IsFinite(candidate.covariance[14]) || !IsFinite(candidate.covariance[35]) ||
        candidate.covariance[0] <= 0.0 || candidate.covariance[7] <= 0.0 ||
        candidate.covariance[14] <= 0.0 || candidate.covariance[35] <= 0.0) {
      return reject("invalid_candidate_covariance");
    }
    const auto iterator = unique_candidates.find(candidate.tag);
    if (iterator == unique_candidates.end() || IsBetterDuplicate(candidate, iterator->second)) {
      unique_candidates[candidate.tag] = candidate;
    }
  }
  if (unique_candidates.size() < gates.minimum_valid_tags) {
    return reject("minimum_valid_tags");
  }

  double x_sum = 0.0;
  double y_sum = 0.0;
  double z_sum = 0.0;
  double x_weight_sum = 0.0;
  double y_weight_sum = 0.0;
  double z_weight_sum = 0.0;
  double yaw_sin_sum = 0.0;
  double yaw_cos_sum = 0.0;
  double yaw_weight_sum = 0.0;
  double latest_stamp_s = -std::numeric_limits<double>::infinity();
  std::int64_t latest_stamp_nanoseconds = 0;
  std::vector<std::pair<double, double>> x_values;
  std::vector<std::pair<double, double>> y_values;
  std::vector<std::pair<double, double>> z_values;
  std::vector<std::pair<double, double>> yaw_values;

  for (const auto& key_and_candidate : unique_candidates) {
    const PoseCandidate& candidate = key_and_candidate.second;
    const tf2::Vector3 origin = candidate.map_to_base.getOrigin();
    const double x_weight = InverseVariance(candidate.covariance[0]);
    const double y_weight = InverseVariance(candidate.covariance[7]);
    const double z_weight = InverseVariance(candidate.covariance[14]);
    const double yaw_weight = InverseVariance(candidate.covariance[35]);
    x_sum += origin.x() * x_weight;
    y_sum += origin.y() * y_weight;
    z_sum += origin.z() * z_weight;
    x_weight_sum += x_weight;
    y_weight_sum += y_weight;
    z_weight_sum += z_weight;
    const double yaw = YawOf(candidate.map_to_base);
    yaw_sin_sum += std::sin(yaw) * yaw_weight;
    yaw_cos_sum += std::cos(yaw) * yaw_weight;
    yaw_weight_sum += yaw_weight;
    x_values.emplace_back(origin.x(), x_weight);
    y_values.emplace_back(origin.y(), y_weight);
    z_values.emplace_back(origin.z(), z_weight);
    yaw_values.emplace_back(yaw, yaw_weight);
    if (candidate.stamp_s >= latest_stamp_s) {
      latest_stamp_s = candidate.stamp_s;
      latest_stamp_nanoseconds = candidate.stamp_nanoseconds;
    }
  }
  if (x_weight_sum <= 0.0 || y_weight_sum <= 0.0 || z_weight_sum <= 0.0 || yaw_weight_sum <= 0.0) {
    return reject("invalid_fusion_weights");
  }

  const double x = x_sum / x_weight_sum;
  const double y = y_sum / y_weight_sum;
  const double z = z_sum / z_weight_sum;
  const double yaw = std::atan2(yaw_sin_sum, yaw_cos_sum);
  double max_position_residual_m = 0.0;
  double max_yaw_residual_rad = 0.0;
  for (const auto& key_and_candidate : unique_candidates) {
    const PoseCandidate& candidate = key_and_candidate.second;
    const tf2::Vector3 difference = candidate.map_to_base.getOrigin() - tf2::Vector3(x, y, z);
    max_position_residual_m = std::max(max_position_residual_m, difference.length());
    max_yaw_residual_rad =
        std::max(max_yaw_residual_rad, std::abs(WrapAngle(YawOf(candidate.map_to_base) - yaw)));
  }
  if (max_position_residual_m > gates.max_intertag_position_spread_m) {
    return reject("max_intertag_position_spread_m");
  }
  if (max_yaw_residual_rad > gates.max_intertag_yaw_spread_rad) {
    return reject("max_intertag_yaw_spread_rad");
  }

  const double x_variance = 1.0 / x_weight_sum + WeightedVariance(x_values, x, false);
  const double y_variance = 1.0 / y_weight_sum + WeightedVariance(y_values, y, false);
  const double z_variance = 1.0 / z_weight_sum + WeightedVariance(z_values, z, false);
  const double yaw_variance = 1.0 / yaw_weight_sum + WeightedVariance(yaw_values, yaw, true);
  // This localizer produces a planar measurement. It deliberately does not
  // synthesize a roll/pitch estimate from one or more camera observations.
  // Future consumers see an explicit conservative covariance rather than an
  // accidental claim of roll/pitch certainty.
  const double roll_variance = covariance_model.unobserved_roll_pitch_variance_rad2;
  const double pitch_variance = covariance_model.unobserved_roll_pitch_variance_rad2;

  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  rotation.normalize();
  PoseEstimate estimate;
  estimate.map_to_base = tf2::Transform(rotation, tf2::Vector3(x, y, z));
  estimate.stamp_s = latest_stamp_s;
  estimate.stamp_nanoseconds = latest_stamp_nanoseconds;
  estimate.distinct_tag_count = unique_candidates.size();
  estimate.max_position_residual_m = max_position_residual_m;
  estimate.max_yaw_residual_rad = max_yaw_residual_rad;
  SetDiagonalCovariance(&estimate.covariance, x_variance, y_variance, z_variance, roll_variance,
                        pitch_variance, yaw_variance);
  return estimate;
}

bool IsWithinJumpGate(const PoseEstimate& estimate, const std::optional<PoseEstimate>& previous,
                      const QualityGates& gates, std::string* reject_reason) {
  if (!previous.has_value()) {
    return true;
  }
  const double stamp_difference_s = estimate.stamp_s - previous->stamp_s;
  if (stamp_difference_s < 0.0) {
    if (reject_reason != nullptr) {
      *reject_reason = "non_monotonic_correction_stamp";
    }
    return false;
  }
  if (stamp_difference_s > gates.jump_reference_timeout_s) {
    return true;
  }
  const double position_jump_m =
      (estimate.map_to_base.getOrigin() - previous->map_to_base.getOrigin()).length();
  if (position_jump_m > gates.max_pose_jump_m) {
    if (reject_reason != nullptr) {
      *reject_reason = "max_pose_jump_m";
    }
    return false;
  }
  const double yaw_jump_rad =
      std::abs(WrapAngle(YawOf(estimate.map_to_base) - YawOf(previous->map_to_base)));
  if (yaw_jump_rad > gates.max_yaw_jump_rad) {
    if (reject_reason != nullptr) {
      *reject_reason = "max_yaw_jump_rad";
    }
    return false;
  }
  return true;
}

} // namespace lb_localization
