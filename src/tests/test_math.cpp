#include "../math.h"
#include <gtest/gtest.h>

namespace {
using namespace pathplanning;
}

TEST(MathTest, QuinticFunctorTest)
{
  QuinticFunctor func;
  func = QuinticFunctor({ 0, 0, 0, 0, 0, 0 });
  EXPECT_EQ(func(0.0), 0.0);

  func = QuinticFunctor({ 10.0, 0, 0, 0, 0, 0 });
  EXPECT_EQ(func(0.0), 10.0);

  func = QuinticFunctor({ 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 });
  EXPECT_EQ(func(1.0), 3.0);
  EXPECT_EQ(func(2.0), 7.0);

  func = QuinticFunctor({ 0.0, 0.0, 0.0, 0.0, 2.0, 2.0 });
  EXPECT_EQ(func(2.0), 96.0);
}

TEST(MathTest, LogisticTest)
{
  EXPECT_EQ(Logistic(0.0), 0.0);
  EXPECT_EQ(Logistic(std::numeric_limits<double>::max()), 1.0);
}

TEST(MathTest, DifferentiaionTets)
{
  QuinticFunctor func5;
  func5 = QuinticFunctor({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
  QuarticFunctor func4 = func5.Differentiate();
  EXPECT_EQ(func4.coeffs[0], 2.0);
  EXPECT_EQ(func4.coeffs[1], 6.0);
  EXPECT_EQ(func4.coeffs[2], 12.0);
  EXPECT_EQ(func4.coeffs[3], 20.0);
  EXPECT_EQ(func4.coeffs[4], 30.0);

  CubicFunctor func3 = func4.Differentiate();
  EXPECT_EQ(func3.coeffs[0], 6.0);
  EXPECT_EQ(func3.coeffs[1], 24.0);
  EXPECT_EQ(func3.coeffs[2], 60.0);
  EXPECT_EQ(func3.coeffs[3], 120.0);

  QuadraticFunctor func2 = func3.Differentiate();
  EXPECT_EQ(func2.coeffs[0], 24.0);
  EXPECT_EQ(func2.coeffs[1], 120.0);
  EXPECT_EQ(func2.coeffs[2], 360.0);

  LinearFunctor func1 = func2.Differentiate();
  EXPECT_EQ(func1.coeffs[0], 120.0);
  EXPECT_EQ(func1.coeffs[1], 720.0);

  ConstantFunctor func0 = func1.Differentiate();
  EXPECT_EQ(func0.coeffs[0], 720.0);
}
