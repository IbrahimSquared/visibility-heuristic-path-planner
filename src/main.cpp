#include "environment/environment.h"
#include "solver/visibilityBasedSolver.h"

#include <iostream>

using namespace vbs;

int main() {
  // Parse settings
  ConfigParser parser;
  if (!parser.parse("config/settings.config")) {
    std::cout << "########################### Parsing results: ####"
                 "########################## \n";
    std::cout << "Error parsing config file" << std::endl;
    return 1;
  } else {
    std::cout << "########################### Parsing results: ####"
                 "########################## \n";
    std::cout << "Config file parsed successfully \n" << std::endl;
  }
  auto config = parser.getConfig();

  // Initialize environment
  environment env = environment(config);
  // Initialize solver & solve
  visibilityBasedSolver solver = visibilityBasedSolver(env);
  solver.solve();
}