// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <gtest/gtest.h>
#include <robot_localization/filter_base.hpp>
#include <limits>
#include <vector>
#include <memory>

#include "robot_localization/ekf.hpp"

#include "robot_localization/ros_filter.hpp"

using robot_localization::Ekf;
using robot_localization::RosFilter;
using robot_localization::STATE_SIZE;

class RosEkfPassThrough : public robot_localization::RosFilter
{
public:
  RosEkfPassThrough(
    rclcpp::Node::SharedPtr node,
    robot_localization::FilterBase::UniquePtr & filter)
  : RosFilter(node, filter) {}
  ~RosEkfPassThrough() {}
  robot_localization::Ekf::UniquePtr & getFilter() {return filter_;}
};

TEST(EkfTest, Measurements) {
  auto node_ = rclcpp::Node::make_shared("ekf");
  robot_localization::FilterBase::UniquePtr filter =
    std::make_unique<robot_localization::Ekf>();

  RosEkfPassThrough ekf(node_, filter);
  Eigen::MatrixXd initialCovar(15, 15);

  initialCovar.setIdentity();
  initialCovar *= 0.5;

  ekf.getFilter()->setEstimateErrorCovariance(initialCovar);

  Eigen::VectorXd measurement(STATE_SIZE);
  measurement.setIdentity();

  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurement[i] = i * 0.01 * STATE_SIZE;
  }
  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  measurementCovariance.setIdentity();
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }
  std::vector<bool> updateVector(STATE_SIZE, true);

  // Ensure that measurements are being placed in the queue correctly
  rclcpp::Time time1(1000);
  ekf.robot_localization::RosFilter::enqueueMeasurement(
    "odom0", measurement, measurementCovariance, updateVector,
    std::numeric_limits<double>::max(), time1);

  ekf.robot_localization::RosFilter::integrateMeasurements(rclcpp::Time(1001));

  EXPECT_EQ(ekf.getFilter()->getState(), measurement);
  EXPECT_EQ(ekf.getFilter()->getEstimateErrorCovariance(),
    measurementCovariance);

  ekf.getFilter()->setEstimateErrorCovariance(initialCovar);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }

  rclcpp::Time time2(1002);

  ekf.robot_localization::RosFilter::enqueueMeasurement(
    "odom0", measurement2, measurementCovariance, updateVector,
    std::numeric_limits<double>::max(), time2);

  ekf.robot_localization::RosFilter::integrateMeasurements(rclcpp::Time(1003));

  measurement = measurement2.eval() - ekf.getFilter()->getState();
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    EXPECT_LT(::fabs(measurement[i]), 0.001);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return ret;
}
