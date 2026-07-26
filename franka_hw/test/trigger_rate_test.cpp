// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cstdint>

#include <gtest/gtest.h>
#include <ros/time.h>

#include <franka_hw/trigger_rate.h>

namespace {

constexpr std::int64_t kBaseTimeMs = 100000;

ros::Time atOffsetMs(std::int64_t offset_ms) {
  const std::int64_t total_ms = kBaseTimeMs + offset_ms;
  return ros::Time(static_cast<std::uint32_t>(total_ms / 1000),
                   static_cast<std::uint32_t>((total_ms % 1000) * 1000000));
}

class TriggerRateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::Time::init();
    ros::Time::setNow(atOffsetMs(0));
  }
};

TEST_F(TriggerRateTest, TriggersAtExactDeadline) {
  franka_hw::TriggerRate trigger(200.0);

  EXPECT_FALSE(trigger(atOffsetMs(4)));
  EXPECT_TRUE(trigger(atOffsetMs(5)));
}

TEST_F(TriggerRateTest, ProducesExactly200TriggersForOneKilohertzCalls) {
  franka_hw::TriggerRate trigger(200.0);
  int trigger_count = 0;

  for (std::int64_t millisecond = 1; millisecond <= 1000; ++millisecond) {
    if (trigger(atOffsetMs(millisecond))) {
      ++trigger_count;
    }
  }

  EXPECT_EQ(trigger_count, 200);
}

TEST_F(TriggerRateTest, PreservesPhaseAfterLateCall) {
  franka_hw::TriggerRate trigger(200.0);

  EXPECT_TRUE(trigger(atOffsetMs(6)));
  EXPECT_FALSE(trigger(atOffsetMs(9)));
  EXPECT_TRUE(trigger(atOffsetMs(10)));
}

TEST_F(TriggerRateTest, SkipsMissedPeriodsWithoutCatchUpBurst) {
  franka_hw::TriggerRate trigger(200.0);

  EXPECT_TRUE(trigger(atOffsetMs(23)));
  EXPECT_FALSE(trigger(atOffsetMs(24)));
  EXPECT_TRUE(trigger(atOffsetMs(25)));
}

TEST_F(TriggerRateTest, ResynchronizesWhenClockMovesBackward) {
  franka_hw::TriggerRate trigger(200.0);

  EXPECT_TRUE(trigger(atOffsetMs(5)));
  EXPECT_FALSE(trigger(atOffsetMs(-1000)));
  EXPECT_FALSE(trigger(atOffsetMs(-996)));
  EXPECT_TRUE(trigger(atOffsetMs(-995)));
}

TEST_F(TriggerRateTest, PreservesOneHertzBehavior) {
  franka_hw::TriggerRate trigger(1.0);

  EXPECT_FALSE(trigger(atOffsetMs(999)));
  EXPECT_TRUE(trigger(atOffsetMs(1000)));
  EXPECT_FALSE(trigger(atOffsetMs(1500)));
  EXPECT_TRUE(trigger(atOffsetMs(2000)));
}

}  // namespace
