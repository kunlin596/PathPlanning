#include "system.h"

int
main(int argc, char** argv)
{
  pathplanning::System system;
  system.ResetMap("../data/highway_map.csv");
  system.Spin();
  return 0;
}
