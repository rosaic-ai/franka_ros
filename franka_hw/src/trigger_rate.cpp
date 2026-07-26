// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/trigger_rate.h>

#include <cstdint>

namespace franka_hw {

TriggerRate::TriggerRate(double rate) : period_(1.0 / rate), time_stamp_(ros::Time::now()) {}

bool TriggerRate::operator()() {
  return (*this)(ros::Time::now());
}

bool TriggerRate::operator()(const ros::Time& now) {
  if (now < time_stamp_) {
    time_stamp_ = now;
    return false;
  }

  const ros::Duration elapsed = now - time_stamp_;
  if (elapsed < period_) {
    return false;
  }

  const std::int64_t period_ns = period_.toNSec();
  const std::int64_t elapsed_periods = elapsed.toNSec() / period_ns;
  ros::Duration deadline_advance;
  deadline_advance.fromNSec(elapsed_periods * period_ns);
  time_stamp_ += deadline_advance;
  return true;
}

}  // namespace franka_hw
