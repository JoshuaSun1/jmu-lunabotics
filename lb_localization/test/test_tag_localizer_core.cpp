// Copyright 2026 JMU NASA Lunabotics Team
// SPDX-License-Identifier: MIT

#include "lb_localization/tag_localizer_core.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <optional>
#include <stdexcept>
#include <string>

namespace lb_localization {
namespace {

constexpr double kPi = 3.14159265358979323846;

tf2::Transform Transform(const double x, const double y, const double z, const double roll = 0.0,
                         const double pitch = 0.0, const double yaw = 0.0) {
  tf2::Quaternion rotation;
  rotation.setRPY(roll, pitch, yaw);
  rotation.normalize();
  return tf2::Transform(rotation, tf2::Vector3(x, y, z));
}

TagDefinition KnownTag(const int id, const tf2::Transform& map_to_tag = Transform(5.0, 0.0, 0.0)) {
  return TagDefinition{
      {"tag36h11", id}, 0.300, "synthetic_field_tag_" + std::to_string(id), map_to_tag};
}

QualityGates TestGates() {
  QualityGates gates;
  gates.max_tag_distance_m = 12.0;
  gates.max_view_angle_rad = 65.0 * kPi / 180.0;
  gates.min_decision_margin = 20.0;
  gates.max_hamming = 0;
  gates.max_pose_jump_m = 2.0;
  gates.max_yaw_jump_rad = 1.0;
  gates.max_detection_age_s = 0.250;
  gates.minimum_valid_tags = 1;
  gates.min_tag_pixel_size_px = 8.0;
  gates.transform_timestamp_tolerance_s = 0.050;
  gates.max_future_skew_s = 0.020;
  gates.fusion_window_s = 0.100;
  gates.jump_reference_timeout_s = 2.0;
  gates.max_intertag_position_spread_m = 2.0;
  gates.max_intertag_yaw_spread_rad = 1.0;
  return gates;
}

CovarianceModel TestCovarianceModel() {
  return CovarianceModel{0.020, 0.010, 0.020, 0.010, 0.100,    2.0,
                         0.050, 0.500, 100.0, 80.0,  1000000.0};
}

DetectionQuality GoodQuality() {
  return DetectionQuality{0, 100.0, 80.0};
}

PoseCandidate Candidate(const TagDefinition& tag, const std::string& source_id,
                        const tf2::Transform& camera_to_tag, const tf2::Transform& base_to_camera,
                        const double stamp_s = 1.0) {
  std::string reason;
  std::optional<PoseCandidate> candidate =
      MakePoseCandidate(tag, source_id, camera_to_tag, base_to_camera, GoodQuality(), TestGates(),
                        TestCovarianceModel(), &reason);
  EXPECT_TRUE(candidate.has_value()) << reason;
  PoseCandidate result = candidate.value();
  result.stamp_s = stamp_s;
  result.stamp_nanoseconds = static_cast<std::int64_t>(stamp_s * 1.0e9);
  return result;
}

PoseCandidate ManualCandidate(const int id, const double x, const double yaw,
                              const double position_variance, const double yaw_variance,
                              const double stamp_s = 1.0) {
  PoseCandidate candidate;
  candidate.tag = {"tag36h11", id};
  candidate.source_id = "synthetic";
  candidate.map_to_base = Transform(x, 0.0, 0.0, 0.0, 0.0, yaw);
  candidate.stamp_s = stamp_s;
  candidate.stamp_nanoseconds = static_cast<std::int64_t>(stamp_s * 1.0e9);
  candidate.decision_margin = 100.0;
  candidate.covariance.fill(0.0);
  candidate.covariance[0] = position_variance;
  candidate.covariance[7] = position_variance;
  candidate.covariance[14] = position_variance;
  candidate.covariance[21] = position_variance;
  candidate.covariance[28] = position_variance;
  candidate.covariance[35] = yaw_variance;
  return candidate;
}

TEST(TagMapTest, SyntheticFixtureHasPlannedFieldInventoryButNoBenchTag) {
  const std::string fixture_directory = TEST_FIXTURE_DIR;
  const TagMap tag_map = TagMap::LoadFromFile(fixture_directory + "/tag_map_synthetic.yaml");

  EXPECT_EQ(tag_map.map_frame, "map");
  EXPECT_EQ(tag_map.survey_status, "synthetic_test_only");
  EXPECT_NE(tag_map.Find({"tag36h11", 1}), nullptr);
  EXPECT_NE(tag_map.Find({CanonicalTagFamily("36h11"), 1}), nullptr);
  EXPECT_NE(tag_map.Find({"tag36h11", 2}), nullptr);
  EXPECT_NE(tag_map.Find({"tag36h11", 3}), nullptr);
  EXPECT_EQ(tag_map.Find({"tag36h11", 0}), nullptr);
  EXPECT_DOUBLE_EQ(tag_map.Find({"tag36h11", 1})->size_m, 0.300);
}

TEST(TagMapTest, SyntheticInputsRequireAnExplicitTestOnlyOptIn) {
  const std::string fixture_directory = TEST_FIXTURE_DIR;
  const TagMap tag_map = TagMap::LoadFromFile(fixture_directory + "/tag_map_synthetic.yaml");
  const LocalizerConfig config =
      LocalizerConfig::LoadFromFile(fixture_directory + "/tag_localizer_synthetic.yaml");

  EXPECT_THROW(ValidateRuntimeInputs(tag_map, config, false), std::runtime_error);
  EXPECT_NO_THROW(ValidateRuntimeInputs(tag_map, config, true));
  EXPECT_EQ(config.FindSource("webcam")->observation_frame_prefix, "webcam_observation_tag_");
  EXPECT_EQ(config.FindSource("zed")->observation_frame_prefix, "zed_observation_tag_");
}

TEST(TagMapTest, RejectsDuplicateRecordsAndTheBenchOnlyId) {
  const std::string fixture_directory = TEST_FIXTURE_DIR;
  EXPECT_THROW(TagMap::LoadFromFile(fixture_directory + "/tag_map_invalid_duplicate.yaml"),
               std::runtime_error);
  EXPECT_THROW(TagMap::LoadFromFile(fixture_directory + "/tag_map_invalid_bench_id.yaml"),
               std::runtime_error);
  EXPECT_THROW(
      TagMap::LoadFromFile(fixture_directory + "/tag_map_invalid_coordinate_convention.yaml"),
      std::runtime_error);
}

TEST(TagLocalizerMathTest, RecoversBasePoseFromKnownTagAndCameraExtrinsic) {
  const TagDefinition tag = KnownTag(1, Transform(5.0, 0.0, 0.0));
  const PoseCandidate candidate =
      Candidate(tag, "webcam", Transform(2.5, 0.0, 0.0), Transform(0.5, 0.0, 0.0));

  EXPECT_NEAR(candidate.map_to_base.getOrigin().x(), 2.0, 1.0e-9);
  EXPECT_NEAR(candidate.map_to_base.getOrigin().y(), 0.0, 1.0e-9);
  EXPECT_NEAR(YawOf(candidate.map_to_base), 0.0, 1.0e-9);
}

TEST(TagLocalizerMathTest, RecoversRotatedAndFullThreeDimensionalGeometry) {
  const tf2::Transform desired_map_to_base = Transform(1.0, 2.0, 0.4, 0.10, -0.20, 0.70);
  const tf2::Transform base_to_camera = Transform(0.2, -0.1, 0.3, 0.02, 0.04, -0.10);
  const TagDefinition tag = KnownTag(1, Transform(4.0, 3.0, 1.1, 0.0, 0.0, 1.0));
  const tf2::Transform camera_to_tag =
      (desired_map_to_base * base_to_camera).inverse() * tag.map_to_tag;
  const PoseCandidate candidate = Candidate(tag, "webcam", camera_to_tag, base_to_camera);

  EXPECT_NEAR(candidate.map_to_base.getOrigin().x(), desired_map_to_base.getOrigin().x(), 1.0e-9);
  EXPECT_NEAR(candidate.map_to_base.getOrigin().y(), desired_map_to_base.getOrigin().y(), 1.0e-9);
  EXPECT_NEAR(candidate.map_to_base.getOrigin().z(), desired_map_to_base.getOrigin().z(), 1.0e-9);
  EXPECT_NEAR(YawOf(candidate.map_to_base), YawOf(desired_map_to_base), 1.0e-9);
}

TEST(TagLocalizerMathTest, DifferentCameraExtrinsicsRecoverTheSameBasePose) {
  const tf2::Transform desired_map_to_base = Transform(1.0, 2.0, 0.0, 0.0, 0.0, kPi / 2.0);
  const TagDefinition tag = KnownTag(1, Transform(1.0, 6.0, 0.0, 0.0, 0.0, kPi / 2.0));
  const tf2::Transform webcam_extrinsic = Transform(1.0, 0.0, 0.2);
  const tf2::Transform zed_extrinsic = Transform(0.3, -0.2, 0.4, 0.0, 0.0, 0.1);
  const PoseCandidate webcam =
      Candidate(tag, "webcam", (desired_map_to_base * webcam_extrinsic).inverse() * tag.map_to_tag,
                webcam_extrinsic);
  const PoseCandidate zed = Candidate(
      tag, "zed", (desired_map_to_base * zed_extrinsic).inverse() * tag.map_to_tag, zed_extrinsic);

  EXPECT_NEAR(webcam.map_to_base.getOrigin().x(), zed.map_to_base.getOrigin().x(), 1.0e-9);
  EXPECT_NEAR(webcam.map_to_base.getOrigin().y(), zed.map_to_base.getOrigin().y(), 1.0e-9);
  EXPECT_NEAR(YawOf(webcam.map_to_base), YawOf(zed.map_to_base), 1.0e-9);
}

TEST(TagLocalizerMathTest, QualityGatesRejectOutOfRangeOrWeakObservations) {
  const TagDefinition tag = KnownTag(1, Transform(5.0, 0.0, 0.0));
  QualityGates gates = TestGates();
  gates.max_tag_distance_m = 5.0;
  std::string reason;
  EXPECT_TRUE(MakePoseCandidate(tag, "webcam", Transform(5.0, 0.0, 0.0), Transform(0.0, 0.0, 0.0),
                                GoodQuality(), gates, TestCovarianceModel(), &reason)
                  .has_value());
  EXPECT_FALSE(MakePoseCandidate(tag, "webcam", Transform(5.001, 0.0, 0.0),
                                 Transform(0.0, 0.0, 0.0), GoodQuality(), gates,
                                 TestCovarianceModel(), &reason)
                   .has_value());
  EXPECT_EQ(reason, "max_tag_distance_m");

  DetectionQuality weak_quality = GoodQuality();
  weak_quality.decision_margin = 19.9;
  EXPECT_FALSE(MakePoseCandidate(tag, "webcam", Transform(1.0, 0.0, 0.0), Transform(0.0, 0.0, 0.0),
                                 weak_quality, TestGates(), TestCovarianceModel(), &reason)
                   .has_value());
  EXPECT_EQ(reason, "min_decision_margin");

  DetectionQuality hamming_quality = GoodQuality();
  hamming_quality.hamming = 1;
  EXPECT_FALSE(MakePoseCandidate(tag, "webcam", Transform(1.0, 0.0, 0.0), Transform(0.0, 0.0, 0.0),
                                 hamming_quality, TestGates(), TestCovarianceModel(), &reason)
                   .has_value());
  EXPECT_EQ(reason, "max_hamming");
}

TEST(TagLocalizerMathTest, CovarianceGrowsForPoorerObservations) {
  const TagDefinition tag = KnownTag(1);
  const PoseCandidate near =
      Candidate(tag, "webcam", Transform(1.0, 0.0, 0.0), Transform(0.0, 0.0, 0.0));
  DetectionQuality poor_quality{0, 20.0, 10.0};
  std::string reason;
  const std::optional<PoseCandidate> far = MakePoseCandidate(
      tag, "webcam", Transform(8.0, 0.0, 0.0, 0.0, 0.5, 0.0), Transform(0.0, 0.0, 0.0),
      poor_quality, TestGates(), TestCovarianceModel(), &reason);

  ASSERT_TRUE(far.has_value()) << reason;
  EXPECT_GT(far->covariance[0], near.covariance[0]);
  EXPECT_GT(far->covariance[35], near.covariance[35]);
}

TEST(TagLocalizerFusionTest, UsesInverseCovarianceWeightsAndIsOrderIndependent) {
  QualityGates gates = TestGates();
  gates.minimum_valid_tags = 2;
  const PoseCandidate precise = ManualCandidate(1, 0.0, 0.0, 1.0, 1.0);
  const PoseCandidate noisy = ManualCandidate(2, 2.0, 0.0, 9.0, 9.0);
  std::string reason;
  const std::optional<PoseEstimate> forward =
      FuseCandidates({precise, noisy}, gates, TestCovarianceModel(), &reason);
  ASSERT_TRUE(forward.has_value()) << reason;
  const std::optional<PoseEstimate> reverse =
      FuseCandidates({noisy, precise}, gates, TestCovarianceModel(), &reason);
  ASSERT_TRUE(reverse.has_value()) << reason;

  EXPECT_NEAR(forward->map_to_base.getOrigin().x(), 0.2, 1.0e-9);
  EXPECT_NEAR(forward->map_to_base.getOrigin().x(), reverse->map_to_base.getOrigin().x(), 1.0e-12);

  const std::optional<PoseEstimate> consistent = FuseCandidates(
      {ManualCandidate(1, 0.0, 0.0, 1.0, 1.0), ManualCandidate(2, 0.0, 0.0, 1.0, 1.0)}, gates,
      TestCovarianceModel(), &reason);
  ASSERT_TRUE(consistent.has_value()) << reason;
  EXPECT_LT(consistent->covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(consistent->covariance[21], 1000000.0);
  EXPECT_DOUBLE_EQ(consistent->covariance[28], 1000000.0);
}

TEST(TagLocalizerFusionTest, HandlesWrappedYawAndDeduplicatesPhysicalTags) {
  QualityGates gates = TestGates();
  gates.minimum_valid_tags = 2;
  gates.max_intertag_yaw_spread_rad = 0.1;
  const std::optional<PoseEstimate> yaw_estimate = FuseCandidates(
      {
          ManualCandidate(1, 0.0, kPi - 0.017, 1.0, 1.0),
          ManualCandidate(2, 0.0, -kPi + 0.017, 1.0, 1.0),
      },
      gates, TestCovarianceModel(), nullptr);
  ASSERT_TRUE(yaw_estimate.has_value());
  EXPECT_NEAR(std::abs(YawOf(yaw_estimate->map_to_base)), kPi, 0.02);

  gates.minimum_valid_tags = 1;
  const std::optional<PoseEstimate> duplicate_estimate = FuseCandidates(
      {
          ManualCandidate(1, 0.0, 0.0, 1.0, 1.0),
          ManualCandidate(1, 100.0, 0.0, 100.0, 100.0),
      },
      gates, TestCovarianceModel(), nullptr);
  ASSERT_TRUE(duplicate_estimate.has_value());
  EXPECT_EQ(duplicate_estimate->distinct_tag_count, 1U);
  EXPECT_NEAR(duplicate_estimate->map_to_base.getOrigin().x(), 0.0, 1.0e-9);
}

TEST(TagLocalizerFusionTest, RejectsIntertagDisagreementAndImplausibleJumps) {
  QualityGates gates = TestGates();
  gates.minimum_valid_tags = 2;
  gates.max_intertag_position_spread_m = 1.0;
  std::string reason;
  EXPECT_FALSE(FuseCandidates(
                   {ManualCandidate(1, 0.0, 0.0, 1.0, 1.0), ManualCandidate(2, 3.0, 0.0, 1.0, 1.0)},
                   gates, TestCovarianceModel(), &reason)
                   .has_value());
  EXPECT_EQ(reason, "max_intertag_position_spread_m");

  PoseEstimate previous;
  previous.map_to_base = Transform(0.0, 0.0, 0.0, 0.0, 0.0, kPi - 0.017);
  previous.stamp_s = 1.0;
  PoseEstimate wrapped_yaw = previous;
  wrapped_yaw.map_to_base = Transform(0.0, 0.0, 0.0, 0.0, 0.0, -kPi + 0.017);
  wrapped_yaw.stamp_s = 1.1;
  gates.max_yaw_jump_rad = 0.1;
  EXPECT_TRUE(IsWithinJumpGate(wrapped_yaw, previous, gates, &reason));

  PoseEstimate large_jump = wrapped_yaw;
  large_jump.map_to_base = Transform(3.0, 0.0, 0.0);
  large_jump.stamp_s = 1.2;
  EXPECT_FALSE(IsWithinJumpGate(large_jump, previous, gates, &reason));
  EXPECT_EQ(reason, "max_pose_jump_m");

  large_jump.stamp_s = 4.0;
  EXPECT_TRUE(IsWithinJumpGate(large_jump, previous, gates, &reason));
}

} // namespace
} // namespace lb_localization
