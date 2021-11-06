#include "../configuration.h"
#include "../jmt.h"
#include "../log.h"

#include <gtest/gtest.h>

namespace {
using namespace pathplanning;
}

TEST(PTGTest, Solve1d_ConstantSpeedTest)
{
  double vel = 30.0;
  double time = 10.0;
  double pos = vel * time;
  double dt = 0.02;
}
