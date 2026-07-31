#include "environment/environment.h"
#include "solver/visibilityBasedSolver.h"

#include <iostream>
#include <string>

namespace {

void printUsage(const char *program) {
  std::cerr << "Usage: " << program
            << " [solve|visibility|benchmark|benchmark-series]"
               " [config-file]\n";
}

} // namespace

int main(int argc, char **argv) {
  if (argc > 3) {
    printUsage(argv[0]);
    return 1;
  }

  const std::string runMode = argc >= 2 ? argv[1] : "solve";
  const std::string configPath =
      argc >= 3 ? argv[2] : "config/settings.config";

  if (runMode != "solve" && runMode != "visibility" &&
      runMode != "benchmark" && runMode != "benchmark-series") {
    std::cerr << "Unknown run mode: " << runMode << '\n';
    printUsage(argv[0]);
    return 1;
  }

  // Parse settings
  vbs::ConfigParser parser;
  if (!parser.parse(configPath)) {
    std::cout << "################## Parsing results: ####"
                 "################# \n";
    std::cout << "Error parsing config file" << std::endl;
    return 1;
  } else {
    std::cout << "################## Parsing results: ####"
                 "################# \n";
    std::cout << "Config file parsed successfully \n" << std::endl;
  }
  auto config = parser.getConfig();

  // Initialize environment
  vbs::environment env = vbs::environment(config);
  // Initialize solver and run the selected operation.
  vbs::visibilityBasedSolver solver = vbs::visibilityBasedSolver(env);
  if (runMode == "solve") {
    solver.solve();
  } else if (runMode == "visibility") {
    solver.standAloneVisibility();
  } else if (runMode == "benchmark") {
    solver.benchmark();
  } else if (runMode == "benchmark-series") {
    solver.benchmarkSeries();
  }

  return 0;
}
