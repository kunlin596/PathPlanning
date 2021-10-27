#include "log.h"
#include "math.h"

int
main(int argc, char** argv)
{
  using namespace pathplanning;

  QuinticFunctor func;
  func = QuinticFunctor({ 0, 0, 0, 0, 0, 0 });
  assert(func(0.0) == 0.0);
  func = QuinticFunctor({ 10.0, 0, 0, 0, 0, 0 });
  assert(func(0.0) == 10.0);
}
