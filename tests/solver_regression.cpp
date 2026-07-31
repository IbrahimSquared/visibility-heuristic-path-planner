#include "environment/environment.h"
#include "solver/visibilityBasedSolver.h"

#include <cmath>
#include <iostream>
#include <queue>
#include <vector>

namespace {

bool segmentIsFree(const vbs::Field<double> &occupancy, vbs::point start,
                   vbs::point end) {
  int x = start.first;
  int y = start.second;
  const int dx = std::abs(end.first - x);
  const int dy = std::abs(end.second - y);
  const int sx = x < end.first ? 1 : -1;
  const int sy = y < end.second ? 1 : -1;
  int error = dx - dy;

  while (true) {
    if (occupancy.get(static_cast<vbs::size_t>(x),
                      static_cast<vbs::size_t>(y)) == 0.0) {
      return false;
    }
    if (x == end.first && y == end.second) {
      return true;
    }

    const int twiceError = 2 * error;
    if (twiceError > -dy) {
      error -= dy;
      x += sx;
    }
    if (twiceError < dx) {
      error += dx;
      y += sy;
    }
  }
}

bool pathIsFree(const vbs::Field<double> &occupancy,
                const std::vector<vbs::point> &path) {
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!segmentIsFree(occupancy, path[i - 1], path[i])) {
      return false;
    }
  }
  return true;
}

int fail(const char *message) {
  std::cerr << "Regression failure: " << message << '\n';
  return 1;
}

} // namespace

int main() {
  vbs::Config config;
  config.mode = 1;
  config.ncols = 80;
  config.nrows = 80;
  config.nb_of_obstacles = 0;
  config.randomSeed = false;
  config.seedValue = 7;
  config.start = {5, 5};
  config.end = {74, 74};
  config.max_iter = 100;
  config.visibilityThreshold = 0.5;
  config.timer = false;
  config.saveResults = false;
  config.silent = true;

  vbs::environment environment(config);
  const auto &occupancy = environment.getVisibilityField();

  for (vbs::size_t y = 0; y < 66; ++y) {
    occupancy->set(25, y, 0.0);
  }
  for (vbs::size_t y = 19; y <= 24; ++y) {
    occupancy->set(25, y, 1.0);
  }
  for (vbs::size_t y = 14; y < 80; ++y) {
    occupancy->set(51, y, 0.0);
  }
  for (vbs::size_t y = 55; y <= 61; ++y) {
    occupancy->set(51, y, 1.0);
  }

  std::priority_queue<vbs::Node> tiedNodes;
  tiedNodes.push({5, 3, 1.0});
  tiedNodes.push({2, 9, 1.0});
  tiedNodes.push({2, 4, 1.0});
  if (tiedNodes.top().x != 2 || tiedNodes.top().y != 4) {
    return fail("candidate tie-breaking is not lexicographically stable");
  }

  const std::vector<vbs::point> visibilitySources = {
      {0, 0},   {79, 0},  {0, 79},  {79, 79}, {40, 0},
      {0, 40},  {40, 79}, {79, 40}, {24, 18}, {26, 25},
  };
  for (const auto &source : visibilitySources) {
    environment.getConfig()->start = source;
    vbs::visibilityBasedSolver visibilitySolver(environment);
    if (!visibilitySolver.validateVisibilityKernels()) {
      return fail("visibility kernels do not match at a boundary source");
    }
  }
  environment.getConfig()->start = config.start;

  vbs::visibilityBasedSolver solver(environment);
  solver.solve();
  const auto firstPath = solver.getLastPath();
  if (firstPath.size() < 3) {
    return fail("obstacle fixture did not exercise a multi-segment path");
  }
  if (firstPath.front() != config.start || firstPath.back() != config.end) {
    return fail("reconstructed path endpoints are incorrect");
  }
  if (!pathIsFree(*occupancy, firstPath)) {
    return fail("reconstructed path intersects an occupied cell");
  }

  solver.solve();
  if (solver.getLastPath() != firstPath) {
    return fail("repeated solve produced a different path");
  }

  return 0;
}
