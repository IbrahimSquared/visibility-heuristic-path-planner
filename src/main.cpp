#include "environment/environment.h"
#include "solver/visibilityBasedSolver.h"

#include <iostream>

using namespace vbs;

int main() {
  // Parse settings
  ConfigParser parser;
  parser.parse("config/settings.config");
  auto config = parser.getConfig();

  // Initialize environment
  environment env = environment(config);
  // Initialize solver & solve
  visibilityBasedSolver solver = visibilityBasedSolver(env);
  solver.solve();
}