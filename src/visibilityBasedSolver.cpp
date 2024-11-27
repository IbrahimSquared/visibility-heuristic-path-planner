#include "solver/visibilityBasedSolver.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace vbs {

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
visibilityBasedSolver::visibilityBasedSolver(environment &env)
    : occupancyComplement_(env.getVisibilityField()),
      sharedConfig_(env.getConfig()) {
  nx_ = occupancyComplement_->nx();
  ny_ = occupancyComplement_->ny();
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  // Init environment image
  uniqueLoadedImage_.reset(std::make_unique<sf::Image>().release());
  uniqueLoadedImage_->create(nx_, ny_, sf::Color::Black);
  sf::Color color;
  color.a = 1;
  for (size_t j = ny_ - 1; j > 0; --j) {
    for (size_t i = 0; i < nx_; ++i) {
      if (occupancyComplement_->get(i, j) < 1) {
        uniqueLoadedImage_->setPixel(i, ny_ - 1 - j, color.Black);
      } else {
        uniqueLoadedImage_->setPixel(i, ny_ - 1 - j, color.White);
      }
    }
  }

  // Init maps
  reset();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::reset() {
  visibility_global_.resize(nx_, ny_, 0.0);
  visibility_.resize(nx_, ny_, 0.0);
  visibilityRayCasting_.resize(nx_, ny_, 1.0);
  cameFrom_.resize(nx_, ny_, 1e15);

  lightSources_.reset(new point[nx_ * ny_]);
  scale_ = sqrt(ny_ * ny_ + nx_ * nx_);
  heap_.reset();

  // Reserve heap_
  std::vector<Node> container;
  container.reserve(nx_ * ny_);
  std::priority_queue<Node, std::vector<Node>, std::less<Node>> heap(
      std::less<Node>(), std::move(container));
  heap_ = std::make_unique<std::priority_queue<Node>>(heap);

  nb_of_sources_ = 0;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::resetQueue() {
  std::vector<Node> container;
  container.reserve(ny_ * nx_);
  std::priority_queue<Node, std::vector<Node>, std::less<Node>> heap(
      std::less<Node>(), std::move(container));
  heap_ = std::make_unique<std::priority_queue<Node>>(heap);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::solve() {
  auto time_start = std::chrono::high_resolution_clock::now();

  // Init
  auto start = sharedConfig_->start;
  auto end = sharedConfig_->end;

  if (sharedConfig_->mode == 2) {
    start.second = ny_ - 1 - start.second;
    end.second = ny_ - 1 - end.second;
  }

  // check if start and end are valid
  if (!isValid(start.first, start.second)) {
    std::cout << "############################## Solver output "
                 "##############################"
              << std::endl;
    std::cout << "Start point is out of bounds." << std::endl;
    return;
  }
  if (!isValid(end.first, end.second)) {
    std::cout << "############################## Solver output "
                 "##############################"
              << std::endl;
    std::cout << "End point is out of bounds." << std::endl;
    return;
  }
  if (occupancyComplement_->get(start.first, start.second) == 0) {
    std::cout << "############################## Solver output "
                 "##############################"
              << std::endl;
    std::cout << "Start point is not valid (occupied)" << std::endl;
    return;
  }
  if (occupancyComplement_->get(end.first, end.second) == 0) {
    std::cout << "############################## Solver output "
                 "##############################"
              << std::endl;
    std::cout << "End point is not valid (occupied)" << std::endl;
    return;
  }

  ls_ = start;
  end_ = end;

  lightSources_[nb_of_sources_] = start;
  cameFrom_(start.first, start.second) = nb_of_sources_;
  visibility_global_(end.first, end.second) = 0;
  max_iter_ = sharedConfig_->max_iter;
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  while (visibility_global_(end.first, end.second) <= visibilityThreshold_) {
    resetQueue();
    updateVisibility();
    auto node = heap_->top();
    ls_ = {node.x, node.y};
    ++nb_of_sources_;
    lightSources_[nb_of_sources_] = ls_;
    if (nb_of_sources_ > max_iter_) {
      std::cout << "Max iters hit. Solution could not be found. Try lowering "
                   "visibility threshold."
                << std::endl;
      return;
    }
  }
  lightSources_[nb_of_sources_] = end;

  auto time_stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      time_stop - time_start);
  if (!sharedConfig_->silent) {
    if (sharedConfig_->timer) {
      std::cout << "############################## Solver output "
                   "##############################"
                << "\n"
                << "Execution time in us: " << duration.count() << "us"
                << std::endl;
    }
  }
  saveResults();
  std::vector<point> path;
  reconstructPath(Node{static_cast<size_t>(end_.first),
                       static_cast<size_t>(end_.second), 0},
                  path);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::standAloneVisibility() {
  // Init
  auto start = sharedConfig_->start;

  // check if start and end are valid
  if (!isValid(start.first, start.second)) {
    std::cout << "############################## Solver output "
                 "##############################"
              << std::endl;
    std::cout << "Start point is out of bounds." << std::endl;
    return;
  }
  if (occupancyComplement_->get(start.first, start.second) == 0) {
    std::cout << "############################## Solver output "
                 "##############################"
              << std::endl;
    std::cout << "Start point is not valid (occupied)" << std::endl;
    return;
  }

  ls_ = start;
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;
  updateVisibility();
  saveStandAloneVisibility();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::benchmark() {
  // Init
  auto start = sharedConfig_->start;

  // check if start and end are valid
  if (!isValid(start.first, start.second)) {
    std::cout << "############################## Solver output "
                 "##############################"
              << std::endl;
    std::cout << "Start point is out of bounds." << std::endl;
    return;
  }
  if (occupancyComplement_->get(start.first, start.second) == 0) {
    std::cout << "############################## Solver output "
                 "##############################"
              << std::endl;
    std::cout << "Start point is not valid (occupied)" << std::endl;
    return;
  }

  ls_ = start;
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  auto time_start = std::chrono::high_resolution_clock::now();
  computeVisibility();
  // computeVisibilityUsingQueue();
  auto time_stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      time_stop - time_start);

  saveStandAloneVisibility();

  auto time_start2 = std::chrono::high_resolution_clock::now();
  // Raycasting without timing
  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      raycasting(ls_.first, ls_.second, i, j);
    }
  }
  auto time_stop2 = std::chrono::high_resolution_clock::now();
  auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(
      time_stop2 - time_start2);

  saveRayCastingVisibility();

  if (!sharedConfig_->silent) {
    std::cout << "############################## Solver output "
                 "##############################"
              << "\n"
              << "Visibility computation time in us: " << duration.count()
              << "us" << std::endl;
    std::cout << "Raycasting computation time in us: " << duration2.count()
              << "us" << std::endl;
    std::cout << "Ratio. Proposed method is: "
              << (double)duration2.count() / duration.count()
              << " faster than typical raycasting." << std::endl;
  }

  int count = 0;
  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      if (occupancyComplement_->get(i, j) == 0) {
        ++count;
      }
    }
  }
  std::cout << "Density of the occupancy grid: "
            << (double)count / (nx_ * ny_) * 100 << "%" << std::endl;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::raycasting(int x0, int y0, int x1, int y1) {
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (x0 != x1 || y0 != y1) {
    if (occupancyComplement_->get(x0, y0) == 0) {
      visibilityRayCasting_(x0, y0) = 0;
      visibilityRayCasting_(x1, y1) = 0;
      return;
    }
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::benchmarkSeries() {
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  const int num_points = 60;
  const double start_value = 50;
  const double end_value = 5000;

  std::vector<double> ratios;
  std::vector<double> times_visibility;
  std::vector<double> times_raycasting;
  ratios.reserve(num_points);
  times_visibility.reserve(num_points);
  times_raycasting.reserve(num_points);

  std::vector<int> logarithmic_points;
  logarithmic_points.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    double log_space = start_value * exp((log(end_value / start_value) * i) /
                                         (num_points - 1));
    logarithmic_points.push_back(round(log_space));
  }

  for (int iter = 0; iter < num_points; ++iter) {
    size_t i = logarithmic_points[iter];

    visibility_.resize(i, i, 0.0);
    visibilityRayCasting_.resize(i, i, 1.0);
    occupancyComplement_->resize(i, i, 1.0);

    ls_ = {i / 2, i / 2};
    nx_ = i;
    ny_ = i;

    auto time_start = std::chrono::high_resolution_clock::now();
    computeVisibility();
    auto time_stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        time_stop - time_start);

    auto time_start2 = std::chrono::high_resolution_clock::now();
    // Raycasting without timing
    for (size_t k = 0; k < i; ++k) {
      for (size_t j = 0; j < i; ++j) {
        raycasting(ls_.first, ls_.second, k, j);
      }
    }
    auto time_stop2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(
        time_stop2 - time_start2);

    std::cout << "***************************" << std::endl;
    std::cout << "For grid size: " << i << "x" << i << std::endl;
    std::cout << "Visibility computation time in us: " << duration.count()
              << "us" << std::endl;
    std::cout << "Raycasting computation time in us: " << duration2.count()
              << "us" << std::endl;
    std::cout << "Ratio. Proposed method is: "
              << (double)duration2.count() / duration.count()
              << " faster than typical raycasting." << std::endl;

    times_visibility.push_back(duration.count());
    times_raycasting.push_back(duration2.count());
    ratios.push_back((double)duration2.count() / duration.count());
  }

  std::cout << "############################## Solver output "
               "##############################"
            << "\n"
            << "Ratios: " << std::endl;
  for (auto &r : ratios) {
    std::cout << r << std::endl;
  }

  std::ofstream file("output/benchmark_results.txt", std::ios::app);
  for (size_t i = 0; i < ratios.size(); ++i) {
    file << times_visibility[i] << " " << times_raycasting[i] << " "
         << ratios[i] << " " << logarithmic_points[i] << "x"
         << logarithmic_points[i] << std::endl;
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::updateVisibility() {
  size_t currentX, currentY;
  double v = 0.0;
  double h;
  point parent;
  double offset = 0.0;

  visibility_.reset();

  // Q1
  max_x_ = nx_ - ls_.first;
  max_y_ = ny_ - ls_.second;

  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second + j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY - 1);
      } else if (j == 0) {
        v = visibility_(currentX - 1, currentY);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second + offset) /
             (currentX - ls_.first + offset);
        v = visibility_(currentX - 1, currentY) -
            c_ * (visibility_(currentX - 1, currentY) -
                  visibility_(currentX - 1, currentY - 1));
      } else if (j > i) {
        c_ = (double)(currentX - ls_.first + offset) /
             (currentY - ls_.second + offset);
        v = visibility_(currentX, currentY - 1) -
            c_ * (visibility_(currentX, currentY - 1) -
                  visibility_(currentX - 1, currentY - 1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_(currentX, currentY) = v;
      visibility_global_(currentX, currentY) =
          std::max(v, visibility_global_(currentX, currentY));
      if (v >= visibilityThreshold_) {
        if (cameFrom_(currentX, currentY) == 1e15) {
          cameFrom_(currentX, currentY) = nb_of_sources_;
        }
      }
      if (visibility_global_(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_(currentX, currentY)];
        h = (scale_ * visibility_global_(currentX, currentY)) +
            (eval_d(currentX, currentY, end_.first, end_.second) +
             eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q2
  max_x_ = ls_.first;
  max_y_ = ny_ - ls_.second;
  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first - i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second + j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY - 1);
      } else if (j == 0) {
        v = visibility_(currentX + 1, currentY);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second + offset) /
             (ls_.first - currentX + offset);
        v = visibility_(currentX + 1, currentY) -
            c_ * (visibility_(currentX + 1, currentY) -
                  visibility_(currentX + 1, currentY - 1));
      } else if (j > i) {
        c_ = (double)(ls_.first - currentX + offset) /
             (currentY - ls_.second + offset);
        v = visibility_(currentX, currentY - 1) -
            c_ * (visibility_(currentX, currentY - 1) -
                  visibility_(currentX + 1, currentY - 1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_(currentX, currentY) = v;
      visibility_global_(currentX, currentY) =
          std::max(v, visibility_global_(currentX, currentY));
      if (v >= visibilityThreshold_) {
        if (cameFrom_(currentX, currentY) == 1e15) {
          cameFrom_(currentX, currentY) = nb_of_sources_;
        }
      }
      if (visibility_global_(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_(currentX, currentY)];
        h = (scale_ * visibility_global_(currentX, currentY)) +
            (eval_d(currentX, currentY, end_.first, end_.second) +
             eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q3
  max_x_ = ls_.first;
  max_y_ = ls_.second;
  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first - i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second - j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY + 1);
      } else if (j == 0) {
        v = visibility_(currentX + 1, currentY);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY + offset) /
             (ls_.first - currentX + offset);
        v = visibility_(currentX + 1, currentY) -
            c_ * (visibility_(currentX + 1, currentY) -
                  visibility_(currentX + 1, currentY + 1));
      } else if (j > i) {
        c_ = (double)(ls_.first - currentX + offset) /
             (ls_.second - currentY + offset);
        v = visibility_(currentX, currentY + 1) -
            c_ * (visibility_(currentX, currentY + 1) -
                  visibility_(currentX + 1, currentY + 1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_(currentX, currentY) = v;
      visibility_global_(currentX, currentY) =
          std::max(v, visibility_global_(currentX, currentY));
      if (v >= visibilityThreshold_) {
        if (cameFrom_(currentX, currentY) == 1e15) {
          cameFrom_(currentX, currentY) = nb_of_sources_;
        }
      }
      if (visibility_global_(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_(currentX, currentY)];
        h = (scale_ * visibility_global_(currentX, currentY)) +
            (eval_d(currentX, currentY, end_.first, end_.second) +
             eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q4
  max_x_ = nx_ - ls_.first;
  max_y_ = ls_.second;
  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second - j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY + 1);
      } else if (j == 0) {
        v = visibility_(currentX - 1, currentY);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY + offset) /
             (currentX - ls_.first + offset);
        v = visibility_(currentX - 1, currentY) -
            c_ * (visibility_(currentX - 1, currentY) -
                  visibility_(currentX - 1, currentY + 1));
      } else if (j > i) {
        c_ = (double)(currentX - ls_.first + offset) /
             (ls_.second - currentY + offset);
        v = visibility_(currentX, currentY + 1) -
            c_ * (visibility_(currentX, currentY + 1) -
                  visibility_(currentX - 1, currentY + 1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_(currentX, currentY) = v;
      visibility_global_(currentX, currentY) =
          std::max(v, visibility_global_(currentX, currentY));
      if (v >= visibilityThreshold_) {
        if (cameFrom_(currentX, currentY) == 1e15) {
          cameFrom_(currentX, currentY) = nb_of_sources_;
        }
      }
      if (visibility_global_(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_(currentX, currentY)];
        h = (scale_ * visibility_global_(currentX, currentY)) +
            (eval_d(currentX, currentY, end_.first, end_.second) +
             eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::computeVisibility() {
  size_t currentX, currentY;
  double v = 0.0;
  double offset = 0.0;

  // Q1
  max_x_ = nx_ - ls_.first;
  max_y_ = ny_ - ls_.second;

  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second + j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY - 1);
      } else if (j == 0) {
        v = visibility_(currentX - 1, currentY);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second + offset) /
             (currentX - ls_.first + offset);
        v = visibility_(currentX - 1, currentY) -
            c_ * (visibility_(currentX - 1, currentY) -
                  visibility_(currentX - 1, currentY - 1));
      } else if (j > i) {
        c_ = (double)(currentX - ls_.first + offset) /
             (currentY - ls_.second + offset);
        v = visibility_(currentX, currentY - 1) -
            c_ * (visibility_(currentX, currentY - 1) -
                  visibility_(currentX - 1, currentY - 1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_(currentX, currentY) = v;
    }
  }
  // Q2
  max_x_ = ls_.first;
  max_y_ = ny_ - ls_.second;
  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first - i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second + j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY - 1);
      } else if (j == 0) {
        v = visibility_(currentX + 1, currentY);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second + offset) /
             (ls_.first - currentX + offset);
        v = visibility_(currentX + 1, currentY) -
            c_ * (visibility_(currentX + 1, currentY) -
                  visibility_(currentX + 1, currentY - 1));
      } else if (j > i) {
        c_ = (double)(ls_.first - currentX + offset) /
             (currentY - ls_.second + offset);
        v = visibility_(currentX, currentY - 1) -
            c_ * (visibility_(currentX, currentY - 1) -
                  visibility_(currentX + 1, currentY - 1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_(currentX, currentY) = v;
    }
  }
  // Q3
  max_x_ = ls_.first;
  max_y_ = ls_.second;
  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first - i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second - j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY + 1);
      } else if (j == 0) {
        v = visibility_(currentX + 1, currentY);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY + offset) /
             (ls_.first - currentX + offset);
        v = visibility_(currentX + 1, currentY) -
            c_ * (visibility_(currentX + 1, currentY) -
                  visibility_(currentX + 1, currentY + 1));
      } else if (j > i) {
        c_ = (double)(ls_.first - currentX + offset) /
             (ls_.second - currentY + offset);
        v = visibility_(currentX, currentY + 1) -
            c_ * (visibility_(currentX, currentY + 1) -
                  visibility_(currentX + 1, currentY + 1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_(currentX, currentY) = v;
    }
  }
  // Q4
  max_x_ = nx_ - ls_.first;
  max_y_ = ls_.second;
  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second - j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY + 1);
      } else if (j == 0) {
        v = visibility_(currentX - 1, currentY);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY + offset) /
             (currentX - ls_.first + offset);
        v = visibility_(currentX - 1, currentY) -
            c_ * (visibility_(currentX - 1, currentY) -
                  visibility_(currentX - 1, currentY + 1));
      } else if (j > i) {
        c_ = (double)(currentX - ls_.first + offset) /
             (ls_.second - currentY + offset);
        v = visibility_(currentX, currentY + 1) -
            c_ * (visibility_(currentX, currentY + 1) -
                  visibility_(currentX - 1, currentY + 1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_(currentX, currentY) = v;
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::computeVisibilityUsingQueue() {
  size_t currentX, currentY;
  double v = 0.0;
  point current;
  double offset = 0.0;

  const size_t ls_x = ls_.first;
  const size_t ls_y = ls_.second;

  visibility_(ls_x, ls_y) = lightStrength_;
  std::queue<point> q;
  q.push({ls_x + 1, ls_y});
  q.push({ls_x, ls_y + 1});
  q.push({ls_x - 1, ls_y});
  q.push({ls_x, ls_y - 1});
  q.push({ls_x + 1, ls_y + 1});
  q.push({ls_x - 1, ls_y + 1});
  q.push({ls_x - 1, ls_y - 1});
  q.push({ls_x + 1, ls_y - 1});

  Field<bool> visited(nx_, ny_, false);

  while (q.size() > 0) {
    current = q.front();
    const size_t x = current.first;
    const size_t y = current.second;
    q.pop();
    if (x < 0 || x >= nx_ || y < 0 || y >= ny_) {
      continue;
    }
    if (visited.get(x, y)) {
      continue;
    }
    if (occupancyComplement_->get(x, y) == 0) {
      continue;
    }

    const int dx = x - ls_x;
    const int dy = y - ls_y;

    if (dx >= 0 && dy >= 0) {
      if (dx == 0) {
        v = visibility_(x, y - 1);
        if (v > 0.001) {
          q.push({x, y + 1});
        }
      } else if (dy == 0) {
        v = visibility_(x - 1, y);
        if (v > 0.001) {
          q.push({x + 1, y});
        }
      } else if (dx == dy) {
        v = visibility_(x - 1, y - 1);
        if (v > 0.001) {
          q.push({x, y + 1});
          q.push({x + 1, y});
          q.push({x + 1, y + 1});
        }
      } else if (dx > dy) {
        c_ = (double)(dy) / (dx);
        v = visibility_(x - 1, y) -
            c_ * (visibility_(x - 1, y) - visibility_(x - 1, y - 1));
        if (v > 0.001) {
          q.push({x, y + 1});
          q.push({x + 1, y});
        }
      } else if (dy > dx) {
        c_ = (double)(dx) / (dy);
        v = visibility_(x, y - 1) -
            c_ * (visibility_(x, y - 1) - visibility_(x - 1, y - 1));
        if (v > 0.001) {
          q.push({x + 1, y});
          q.push({x, y + 1});
        }
      }
      v = v * occupancyComplement_->get(x, y);
      visibility_(x, y) = v;
      visited(x, y) = true;
    } else if (dx < 0 && dy >= 0) {
      if (dx == 0) {
        v = visibility_(x, y - 1);
        if (v > 0.001) {
          q.push({x, y + 1});
        }
      } else if (dy == 0) {
        v = visibility_(x + 1, y);
        if (v > 0.001) {
          q.push({x - 1, y});
        }
      } else if (-dx == dy) {
        v = visibility_(x + 1, y - 1);
        if (v > 0.001) {
          q.push({x, y + 1});
          q.push({x - 1, y});
          q.push({x - 1, y + 1});
        }
      } else if (-dx > dy) {
        c_ = (double)(dy) / (-dx);
        v = visibility_(x + 1, y) -
            c_ * (visibility_(x + 1, y) - visibility_(x + 1, y - 1));
        if (v > 0.001) {
          q.push({x, y + 1});
          q.push({x - 1, y});
        }
      } else if (dy > -dx) {
        c_ = (double)(-dx) / (dy);
        v = visibility_(x, y - 1) -
            c_ * (visibility_(x, y - 1) - visibility_(x + 1, y - 1));
        if (v > 0.001) {
          q.push({x - 1, y});
          q.push({x, y + 1});
        }
      }
      v = v * occupancyComplement_->get(x, y);
      visibility_(x, y) = v;
      visited(x, y) = true;
    } else if (dx < 0 && dy < 0) {
      if (dx == 0) {
        v = visibility_(x, y + 1);
        if (v > 0.001) {
          q.push({x, y - 1});
        }
      } else if (dy == 0) {
        v = visibility_(x + 1, y);
        if (v > 0.001) {
          q.push({x - 1, y});
        }
      } else if (dx == dy) {
        v = visibility_(x + 1, y + 1);
        if (v > 0.001) {
          q.push({x, y - 1});
          q.push({x - 1, y});
          q.push({x - 1, y - 1});
        }
      } else if (-dx > -dy) {
        c_ = (double)(dy) / (dx);
        v = visibility_(x + 1, y) -
            c_ * (visibility_(x + 1, y) - visibility_(x + 1, y + 1));
        if (v > 0.001) {
          q.push({x, y - 1});
          q.push({x - 1, y});
        }
      } else if (-dy > -dx) {
        c_ = (double)(dx) / (dy);
        v = visibility_(x, y + 1) -
            c_ * (visibility_(x, y + 1) - visibility_(x + 1, y + 1));
        if (v > 0.001) {
          q.push({x - 1, y});
          q.push({x, y - 1});
        }
      }
      v = v * occupancyComplement_->get(x, y);
      visibility_(x, y) = v;
      visited(x, y) = true;
    } else if (dx >= 0 && dy < 0) {
      if (dx == 0) {
        v = visibility_(x, y + 1);
        if (v > 0.001) {
          q.push({x, y - 1});
        }
      } else if (dy == 0) {
        v = visibility_(x - 1, y);
        if (v > 0.001) {
          q.push({x + 1, y});
        }
      } else if (dx == -dy) {
        v = visibility_(x - 1, y + 1);
        if (v > 0.001) {
          q.push({x, y - 1});
          q.push({x + 1, y});
          q.push({x + 1, y - 1});
        }
      } else if (dx > -dy) {
        c_ = (double)(-dy) / (dx);
        v = visibility_(x - 1, y) -
            c_ * (visibility_(x - 1, y) - visibility_(x - 1, y + 1));
        if (v > 0.001) {
          q.push({x, y - 1});
          q.push({x + 1, y});
        }
      } else if (-dy > dx) {
        c_ = (double)(dx) / (-dy);
        v = visibility_(x, y + 1) -
            c_ * (visibility_(x, y + 1) - visibility_(x - 1, y + 1));
        if (v > 0.001) {
          q.push({x + 1, y});
          q.push({x, y - 1});
        }
      }
      v = v * occupancyComplement_->get(x, y);
      visibility_(x, y) = v;
      visited(x, y) = true;
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::saveStandAloneVisibility() const {
  sf::Image image;
  image = *uniqueLoadedImage_;
  sf::Color color;
  color.a = 1;

  for (size_t j = ny_ - 1; j > 0; --j) {
    for (size_t i = 0; i < nx_; ++i) {
      image.setPixel(i, ny_ - 1 - j,
                     sf::Color(255 * visibility_(i, j), 255 * visibility_(i, j),
                               255 * visibility_(i, j)));
      // use this for binary visibility
      // if (visibility_(i, j) < visibilityThreshold_) {
      //   image.setPixel(i, ny_ - 1 - j, color.Black);
      // } else {
      //   image.setPixel(i, ny_ - 1 - j, color.White);
      // }
    }
  }

  // set ls_ as yellow ball
  const int ballRadius = sharedConfig_->ballRadius;
  int x0 = ls_.first;
  int y0 = ny_ - 1 - ls_.second;
  for (int j = -ballRadius; j <= ballRadius; ++j) {
    for (int k = -ballRadius; k <= ballRadius; ++k) {
      if (isValid(x0 + j, y0 + k)) {
        if (j * j + k * k <= ballRadius * ballRadius) {
          image.setPixel(x0 + j, y0 + k, color.Yellow);
        }
      }
    }
  }

  // add black ring around the yellow ball
  for (int j = -ballRadius - 1; j <= ballRadius + 1; ++j) {
    for (int k = -ballRadius - 1; k <= ballRadius + 1; ++k) {
      if (isValid(x0 + j, y0 + k)) {
        if (j * j + k * k > ballRadius * ballRadius &&
            j * j + k * k <= (ballRadius + 1) * (ballRadius + 1)) {
          image.setPixel(x0 + j, y0 + k, color.Black);
        }
      }
    }
  }

  // set occupied cells as red
  for (size_t j = ny_ - 1; j > 0; --j) {
    for (size_t i = 0; i < nx_; ++i) {
      if (occupancyComplement_->get(i, j) == 0) {
        image.setPixel(i, ny_ - 1 - j, color.Red);
      }
    }
  }

  const std::string imageName = "output/standAloneVisibility.png";
  image.saveToFile(imageName);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::saveRayCastingVisibility() const {
  sf::Image image;
  image = *uniqueLoadedImage_;
  sf::Color color;
  color.a = 1;

  for (size_t j = ny_ - 1; j > 0; --j) {
    for (size_t i = 0; i < nx_; ++i) {
      image.setPixel(i, ny_ - 1 - j,
                     sf::Color(255 * visibilityRayCasting_.get(i, j),
                               255 * visibilityRayCasting_.get(i, j),
                               255 * visibilityRayCasting_.get(i, j)));
      // use this for binary visibility
      // if (visibilityRayCasting_.get(i, j) < visibilityThreshold_) {
      //   image.setPixel(i, ny_ - 1 - j, color.Black);
      // } else {
      //   image.setPixel(i, ny_ - 1 - j, color.White);
      // }
    }
  }
  // set ls_ as yellow ball
  const int ballRadius = sharedConfig_->ballRadius;
  int x0 = ls_.first;
  int y0 = ny_ - 1 - ls_.second;
  for (int j = -ballRadius; j <= ballRadius; ++j) {
    for (int k = -ballRadius; k <= ballRadius; ++k) {
      if (isValid(x0 + j, y0 + k)) {
        if (j * j + k * k <= ballRadius * ballRadius) {
          image.setPixel(x0 + j, y0 + k, color.Yellow);
        }
      }
    }
  }

  // add black ring around the yellow ball
  for (int j = -ballRadius - 1; j <= ballRadius + 1; ++j) {
    for (int k = -ballRadius - 1; k <= ballRadius + 1; ++k) {
      if (isValid(x0 + j, y0 + k)) {
        if (j * j + k * k > ballRadius * ballRadius &&
            j * j + k * k <= (ballRadius + 1) * (ballRadius + 1)) {
          image.setPixel(x0 + j, y0 + k, color.Black);
        }
      }
    }
  }

  // set occupied cells as red
  for (size_t j = ny_ - 1; j > 0; --j) {
    for (size_t i = 0; i < nx_; ++i) {
      if (occupancyComplement_->get(i, j) == 0) {
        image.setPixel(i, ny_ - 1 - j, color.Red);
      }
    }
  }

  const std::string imageName = "output/rayCastingVisibility.png";
  image.saveToFile(imageName);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::saveResults() const {
  namespace fs = std::filesystem;
  // Define the path to the output file
  std::string path = "./output/cameFrom.txt";
  // Check if the directory exists, and create it if it doesn't
  fs::path dir = fs::path(path).parent_path();
  if (!fs::exists(dir)) {
    if (!fs::create_directories(dir)) {
      std::cerr << "Failed to create directory " << dir.string() << std::endl;
      return;
    }
  }

  if (sharedConfig_->saveCameFrom) {
    std::fstream of(path, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << path << std::endl;
      return;
    }
    if (of.is_open()) {
      std::ostream &os = of;
      if (sharedConfig_->mode == 2) {
        for (int j = ny_ - 1; j >= 0; --j) {
          for (size_t i = 0; i < nx_; ++i) {
            os << cameFrom_(i, j) << " ";
          }
          os << "\n";
        }
      } else {
        for (size_t j = 0; j < ny_; ++j) {
          for (size_t i = 0; i < nx_; ++i) {
            os << cameFrom_(i, j) << " ";
          }
          os << "\n";
        }
      }
      of.close();
    }
    if (!sharedConfig_->silent) {
      std::cout << "Saved cameFrom_" << std::endl;
    }
  }
  if (sharedConfig_->saveLightSources) {
    path = "./output/lightSources.txt";
    std::fstream of(path, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << path << std::endl;
      return;
    }
    if (of.is_open()) {
      std::ostream &os = of;
      for (size_t i = 0; i < nb_of_sources_; ++i) {
        if (sharedConfig_->mode == 2) {
          os << lightSources_[i].first << " "
             << ny_ - 1 - lightSources_[i].second;
        } else {
          os << lightSources_[i].first << " " << lightSources_[i].second;
        }
        os << "\n";
      }
      of.close();
    }
    if (!sharedConfig_->silent) {
      std::cout << "Saved lightSources" << std::endl;
    }
  }
  if (sharedConfig_->saveGlobalVisibility) {
    path = "./output/VisibilityMap.txt";
    std::fstream of(path, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << path << std::endl;
      return;
    }
    if (of.is_open()) {
      std::ostream &os = of;
      if (sharedConfig_->mode == 2) {
        for (int j = ny_ - 1; j >= 0; --j) {
          for (size_t i = 0; i < nx_; ++i) {
            os << visibility_global_(i, j) << " ";
          }
          os << "\n";
        }
      } else {
        for (size_t j = 0; j < ny_; ++j) {
          for (size_t i = 0; i < nx_; ++i) {
            os << visibility_global_(i, j) << " ";
          }
          os << "\n";
        }
      }
      of.close();
    }
    if (!sharedConfig_->silent) {
      std::cout << "Saved GlobalVisibility" << std::endl;
    }
  }
  if (sharedConfig_->saveLocalVisibility) {
    path = "./output/LocalVisibilityMap.txt";
    std::fstream of(path, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << path << std::endl;
      return;
    }
    if (of.is_open()) {
      std::ostream &os = of;
      if (sharedConfig_->mode == 2) {
        for (int j = ny_ - 1; j >= 0; --j) {
          for (size_t i = 0; i < nx_; ++i) {
            os << visibility_(i, j) << " ";
          }
          os << "\n";
        }
      } else {
        for (size_t j = 0; j < ny_; ++j) {
          for (size_t i = 0; i < nx_; ++i) {
            os << visibility_(i, j) << " ";
          }
          os << "\n";
        }
      }
      of.close();
    }
    if (!sharedConfig_->silent) {
      std::cout << "Saved LocalVisibility" << std::endl;
    }
  }
  if (sharedConfig_->saveVisibilityField) {
    path = "./output/visibilityField.txt";
    std::fstream of(path, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << path << std::endl;
      return;
    }
    if (of.is_open()) {
      std::ostream &os = of;
      if (sharedConfig_->mode == 2) {
        for (int j = ny_ - 1; j >= 0; --j) {
          for (size_t i = 0; i < nx_; ++i) {
            os << occupancyComplement_->get(i, j) << " ";
          }
          os << "\n";
        }
      } else {
        for (size_t j = 0; j < ny_; ++j) {
          for (size_t i = 0; i < nx_; ++i) {
            os << occupancyComplement_->get(i, j) << " ";
          }
          os << "\n";
        }
      }
      of.close();
    }
    if (!sharedConfig_->silent) {
      std::cout << "Saved OccupancyComplement" << std::endl;
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::reconstructPath(const Node &current,
                                            std::vector<point> &resultingPath) {
  int x = current.x, y = current.y;
  double t = cameFrom_(x, y);
  double t_old = std::numeric_limits<double>::max();
  while (t != t_old) {
    resultingPath.push_back({x, y});
    t_old = t;
    x = lightSources_[t].first;
    y = lightSources_[t].second;
    t = cameFrom_(x, y);
  }
  resultingPath.push_back({x, y});
  std::reverse(resultingPath.begin(), resultingPath.end());

  // compute total distance
  double totalDistance = 0;
  for (size_t i = 0; i < resultingPath.size() - 1; ++i) {
    totalDistance +=
        eval_d(resultingPath[i].first, resultingPath[i].second,
               resultingPath[i + 1].first, resultingPath[i + 1].second);
  }
  if (!sharedConfig_->silent) {
    std::cout << "Path length: " << totalDistance << std::endl;
  }

  if (sharedConfig_->saveResults) {
    saveImageWithPath(resultingPath);
  }
  return;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::saveImageWithPath(
    const std::vector<point> &path) const {
  sf::Image image;
  image = *uniqueLoadedImage_;
  sf::Color color;
  color.a = 1;
  int x0, y0, x1, y1;
  int dx, dy, sx, sy, err;

  for (size_t i = 0; i < path.size() - 1; ++i) {
    x0 = path[i].first;
    y0 = ny_ - 1 - path[i].second;
    x1 = path[i + 1].first;
    y1 = ny_ - 1 - path[i + 1].second;

    dx = std::abs(x1 - x0);
    dy = std::abs(y1 - y0);
    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;
    err = dx - dy;

    while (x0 != x1 || y0 != y1) {
      image.setPixel(x0, y0, color.Magenta);
      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }
  }
  const int ballRadius = sharedConfig_->ballRadius;
  for (size_t i = 0; i < path.size(); ++i) {
    x0 = path[i].first;
    y0 = ny_ - 1 - path[i].second;
    for (int j = -ballRadius; j <= ballRadius; ++j) {
      for (int k = -ballRadius; k <= ballRadius; ++k) {
        if (isValid(x0 + j, y0 + k)) {
          if (j * j + k * k <= ballRadius * ballRadius) {
            image.setPixel(x0 + j, y0 + k, color.Cyan);
          }
        }
      }
    }
  }

  x0 = path[0].first;
  y0 = ny_ - 1 - path[0].second;
  for (int j = -ballRadius; j <= ballRadius; ++j) {
    for (int k = -ballRadius; k <= ballRadius; ++k) {
      if (isValid(x0 + j, y0 + k)) {
        if (j * j + k * k <= ballRadius * ballRadius) {
          image.setPixel(x0 + j, y0 + k, color.Green);
        }
      }
    }
  }

  x0 = path[path.size() - 1].first;
  y0 = ny_ - 1 - path[path.size() - 1].second;
  for (int j = -ballRadius; j <= ballRadius; ++j) {
    for (int k = -ballRadius; k <= ballRadius; ++k) {
      if (isValid(x0 + j, y0 + k)) {
        if (j * j + k * k <= ballRadius * ballRadius) {
          image.setPixel(x0 + j, y0 + k, color.Red);
        }
      }
    }
  }
  const std::string imageName = "output/ResultingPath.png";
  image.saveToFile(imageName);
}

} // namespace vbs
