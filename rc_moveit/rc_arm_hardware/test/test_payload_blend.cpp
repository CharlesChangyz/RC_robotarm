#include <gtest/gtest.h>

#include "rc_arm_hardware/payload_blend.hpp"

namespace rc_arm_hardware
{
namespace
{

TEST(PayloadBlend, RampsUpTowardTarget)
{
  double blend = 0.0;

  updatePayloadBlend(blend, 1.0, 0.25, 1.0, 0.2, true);

  EXPECT_DOUBLE_EQ(blend, 0.25);
}

TEST(PayloadBlend, RampsDownTowardTarget)
{
  double blend = 1.0;

  updatePayloadBlend(blend, 0.0, 0.1, 1.0, 0.2, true);

  EXPECT_DOUBLE_EQ(blend, 0.5);
}

TEST(PayloadBlend, DoesNotOvershootTarget)
{
  double blend = 0.9;

  updatePayloadBlend(blend, 1.0, 0.5, 1.0, 0.2, true);

  EXPECT_DOUBLE_EQ(blend, 1.0);
}

TEST(PayloadBlend, DisabledFollowsTargetImmediately)
{
  double blend = 0.2;

  updatePayloadBlend(blend, 1.0, 0.01, 1.0, 0.2, false);

  EXPECT_DOUBLE_EQ(blend, 1.0);
}

TEST(PayloadBlend, MixesScalarsLinearly)
{
  EXPECT_DOUBLE_EQ(lerpPayloadValue(10.0, 20.0, 0.25), 12.5);
}

}  // namespace
}  // namespace rc_arm_hardware
