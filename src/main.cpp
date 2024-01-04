#include "environment/environment.h"
#include "solver/visibilityBasedSolver.h"

#include <iostream>

int main() {
  // Parse settings
  vbs::ConfigParser parser;
  if (!parser.parse("config/settings.config")) {
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
  // Initialize solver & solve
  vbs::visibilityBasedSolver solver = vbs::visibilityBasedSolver(env);
  solver.solve();
}
