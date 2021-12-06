#include "log.h"
#include "system.h"
#include <boost/program_options.hpp>
#include <iostream>

#include <pybind11/embed.h>

int
main(int argc, char** argv)
{
  namespace po = boost::program_options;
  using namespace pathplanning;

  namespace py = pybind11;
  py::scoped_interpreter guard{};

  std::string confFileName;
  std::string mapFileName;
  try {

    po::options_description desc("Allowed options");

    // clang-format off
    desc.add_options()
      ("help,h", po::value<std::string>(), "Help message")
      ("conf,c", po::value<std::string>(&confFileName)->required(),
        "System configuration file name.")
      ("map,m", po::value<std::string>(&mapFileName)->required(),
        "Map file name.")
      ("loglevel,l", po::value<std::string>()->default_value("debug"),
        "System configuration file name.");
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    std::string loglevel = vm["loglevel"].as<std::string>();
    if (loglevel == "debug") {
      spdlog::set_level(spdlog::level::debug);
    } else if (loglevel == "trace") {
      spdlog::set_level(spdlog::level::trace);
    } else if (loglevel == "info") {
      spdlog::set_level(spdlog::level::info);
    } else if (loglevel == "error") {
      spdlog::set_level(spdlog::level::err);
    }

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
  system.ResetMap(mapFileName);
  system.Spin();
  return 0;
}
