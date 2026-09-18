#include <gtest/gtest.h>
#include <cmath>
#include "control/longitudinal/path_speed_profile.hpp"

using namespace stier_control;
namespace {
constexpr double kCeiling=15.0/3.6;
std::vector<Point2d> straight() {
  std::vector<Point2d> path;
  for (int i=-4; i<=180; ++i) path.push_back({i*.25, 0.0});
  return path;
}
std::vector<Point2d> bend(double entry) {
  std::vector<Point2d> path;
  for (double x=-1.; x<entry; x+=.1) path.push_back({x,0});
  for (int i=0; i<=31; ++i) {
    double t=i*.05;
    path.push_back({entry+2*std::sin(t), 2*(1-std::cos(t))});
  }
  return path;
}
PathSpeedResult speed(const std::vector<Point2d>& path, double requested=0.,
                      double applied=0., double stop=-1., double cap=kCeiling) {
  return computePathSpeed(path, 2.0, requested, applied, cap, stop, PathSpeedConfig{});
}
}

TEST(PathSpeedProfile, StraightUsesExistingCeiling) {
  EXPECT_DOUBLE_EQ(speed(straight()).speed_mps,kCeiling);
}
TEST(PathSpeedProfile, TightCurveRespectsLateralAcceleration) {
  const auto result=speed(bend(0));
  EXPECT_NEAR(result.curve_limit_mps,std::sqrt(2.0),.03);
  // This short arc also has a visibility cap; either may lower the command.
  EXPECT_LE(result.speed_mps,result.curve_limit_mps);
  EXPECT_GT(result.maximum_curvature_m_inv,.48);
}
TEST(PathSpeedProfile, BrakesBeforeBendAndSlowsFurtherAsItApproaches) {
  const double distant=speed(bend(6)).speed_mps;
  const double near=speed(bend(2)).speed_mps;
  EXPECT_LT(distant,kCeiling);
  EXPECT_LT(near,distant);
  EXPECT_GT(near,1.0/3.6);
}
TEST(PathSpeedProfile, RequestedAndAppliedSteeringBothLimitSpeed) {
  const double angle=25.0*std::acos(-1.)/180.0;
  const double expected=std::sqrt(.75/std::tan(angle));
  EXPECT_NEAR(speed(straight(),angle,0).speed_mps,expected,1e-9);
  EXPECT_NEAR(speed(straight(),0,angle).speed_mps,expected,1e-9);
  EXPECT_NEAR(speed(straight(),-angle,-angle).speed_mps,expected,1e-9);
}
TEST(PathSpeedProfile, UnevenDuplicatePointsDoNotCreateSpuriousBraking) {
  const std::vector<Point2d> path{{-1,0},{0,0},{0,0},{.001,0},{.5,0},{19,0},{45,0}};
  EXPECT_DOUBLE_EQ(speed(path).speed_mps,kCeiling);
}
TEST(PathSpeedProfile, PassedPrefixIsNotAForwardCurve) {
  auto path=straight();
  path.insert(path.begin(),{{-6,0},{-5,2},{-4,0},{-3,-2},{-2,0}});
  EXPECT_DOUBLE_EQ(speed(path).speed_mps,kCeiling);
}
TEST(PathSpeedProfile, MirroredCurveHasSameSpeed) {
  auto path=bend(2);
  const double reference=speed(path).speed_mps;
  for(auto& p:path) p.y=-p.y;
  EXPECT_NEAR(speed(path).speed_mps,reference,1e-9);
}
TEST(PathSpeedProfile, StopApproachSlowsWithoutRoundingIntoNeutral) {
  const double far=speed(straight(),0,0,5).speed_mps;
  const double close=speed(straight(),0,0,2).speed_mps;
  EXPECT_LT(close,far);
  EXPECT_DOUBLE_EQ(speed(straight(),0,0,.1).speed_mps,1.0/3.6);
  EXPECT_DOUBLE_EQ(speed(straight(),0,0,0).speed_mps,1.0/3.6);
}
TEST(PathSpeedProfile, DoesNotRaiseParkingOrExternalMissionCaps) {
  EXPECT_DOUBLE_EQ(speed(straight(),0,0,-1,.5).speed_mps,.5);
  EXPECT_DOUBLE_EQ(speed(straight(),0,0,.1,.1).speed_mps,.1);
}
TEST(PathSpeedProfile, ShortTailDoesNotBecomeNewStopRule) {
  EXPECT_GT(speed({{0,0},{.1,0}}).speed_mps,0.0);
}
TEST(PathSpeedProfile, FifteenKphRequiresEnoughPublishedPreview) {
  const auto short_path=speed({{0,0},{20,0}});
  EXPECT_LT(short_path.speed_mps,kCeiling);
  EXPECT_GT(short_path.speed_mps,1.0/3.6);
  EXPECT_DOUBLE_EQ(speed(straight()).speed_mps,kCeiling);
}
TEST(PathSpeedProfile, ConfigurationContract) {
  PathSpeedConfig c;
  EXPECT_TRUE(isValidPathSpeedConfig(c));
  c.sample_interval_m=0.;
  EXPECT_FALSE(isValidPathSpeedConfig(c));
  c=PathSpeedConfig{}; c.lateral_acceleration_mps2=-1.;
  EXPECT_FALSE(isValidPathSpeedConfig(c));
}
