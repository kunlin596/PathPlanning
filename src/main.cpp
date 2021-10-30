#include "system.h"
#include <boost/program_options.hpp>
#include <iostream>

int
main(int argc, char** argv)
{
  namespace po = boost::program_options;
  using namespace pathplanning;

  std::string confFileName;
  try {

    po::options_description desc("Allowed options");

    // clang-format off
    desc.add_options()
      ("help,h", po::value<std::string>(), "Help message")
      ("conf,c", po::value<std::string>(&confFileName)->required(),
        "System configuration file name.");
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "Exception of unknown type!\n";
  }

  System system;
  system.Initialize(confFileName);
  system.ResetMap("../data/highway_map.csv");
  system.Spin();
  return 0;
}
