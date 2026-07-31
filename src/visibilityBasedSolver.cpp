#include "solver/visibilityBasedSolver.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>

namespace vbs {
constexpr double CAMEFROM_UNSET = 1e15;

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
  heap_ = std::make_unique<std::priority_queue<Node>>();

  nb_of_sources_ = 0;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::resetQueue() {
  heap_ = std::make_unique<std::priority_queue<Node>>();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void vbs::visibilityBasedSolver::solve() {
  auto time_start = std::chrono::high_resolution_clock::now();
  lastPath_.clear();

  // Init
  auto start = sharedConfig_->start;
  auto end = sharedConfig_->end;

  if (sharedConfig_->mode == 2) {
    start.second = ny_ - 1 - start.second;
    end.second = ny_ - 1 - end.second;
  }

  // Validate start/end
  if (!isValid(start.first, start.second)) {
    std::cout << "############################## Solver output "
                 "##############################\n";
    std::cout << "Start point is out of bounds.\n";
    return;
  }
  if (!isValid(end.first, end.second)) {
    std::cout << "############################## Solver output "
                 "##############################\n";
    std::cout << "End point is out of bounds.\n";
    return;
  }
  if (occupancyComplement_->get(start.first, start.second) == 0) {
    std::cout << "############################## Solver output "
                 "##############################\n";
    std::cout << "Start point is not valid (occupied)\n";
    return;
  }
  if (occupancyComplement_->get(end.first, end.second) == 0) {
    std::cout << "############################## Solver output "
                 "##############################\n";
    std::cout << "End point is not valid (occupied)\n";
    return;
  }

  // Planner state reset (important if solve() can be called multiple times)
  {
    auto *glob = visibility_global_.data();
    auto *cf = cameFrom_.data();
    const size_t N = nx_ * ny_;
    std::fill(glob, glob + N, 0.0);
    std::fill(cf, cf + N, CAMEFROM_UNSET);
  }

  nb_of_sources_ = 0;
  max_iter_ = sharedConfig_->max_iter;
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  ls_ = start;
  end_ = end;

  lightSources_[0] = start;
  cameFrom_(start.first, start.second) = 0;

  // Main loop
  while (true) {
    Node best;

    const bool haveCandidate = updateVisibilityAndPickBest(best);

    // If end is now visible enough, stop WITHOUT consuming another source slot
    if (visibility_global_(end_.first, end_.second) >= visibilityThreshold_) {
      break;
    }

    if (!haveCandidate) {
      std::cout << "No valid next light-source candidate found.\n";
      return;
    }

    // Advance to next source
    ++nb_of_sources_;
    if (nb_of_sources_ > max_iter_) {
      std::cout << "Max iters hit. Solution could not be found. Try lowering "
                   "visibility threshold.\n";
      return;
    }

    ls_ = {static_cast<int>(best.x), static_cast<int>(best.y)};
    lightSources_[nb_of_sources_] = ls_;
  }

  // Optionally append end as a final marker (NOT referenced by cameFrom
  // indices)
  ++nb_of_sources_;
  lightSources_[nb_of_sources_] = end_;

  auto time_stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      time_stop - time_start);

  if (!sharedConfig_->silent && sharedConfig_->timer) {
    std::cout << "############################## Solver output "
                 "##############################\n";
    std::cout << "Execution time in us: " << duration.count() << "us\n";
  }

  saveResults();
  reconstructPath(Node{static_cast<size_t>(end_.first),
                       static_cast<size_t>(end_.second), 0},
                  lastPath_);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::standAloneVisibility() {
  // Init
  auto start = sharedConfig_->start;
  if (sharedConfig_->mode == 2) {
    start.second = ny_ - 1 - start.second;
  }

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
  computeVisibility_parallelSections();
  saveStandAloneVisibility();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::benchmark() {
  // Init
  auto start = sharedConfig_->start;
  if (sharedConfig_->mode == 2) {
    start.second = ny_ - 1 - start.second;
  }

  // check if start and end are valid
  if (!isValid(start.first, start.second)) {
    std::cout << "############################## Solver output "
                 "##############################\n";
    std::cout << "Start point is out of bounds.\n";
    return;
  }
  if (occupancyComplement_->get(start.first, start.second) == 0) {
    std::cout << "############################## Solver output "
                 "##############################\n";
    std::cout << "Start point is not valid (occupied)\n";
    return;
  }

  ls_ = start;
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  // --- Helpers for timing and exactness ---
  auto *vis = visibility_.data();
  const size_t n = visibility_.size();
  const size_t bytes = n * sizeof(double);

  auto clear_visibility = [&]() { std::fill(vis, vis + n, 0.0); };

  auto run_and_capture = [&](const char * /*name*/, auto &&fn,
                             std::vector<double> &out) -> long long {
    // Warm-up (avoids first-run effects: cache/OMP runtime init)
    clear_visibility();
    fn();

    // Timed run
    clear_visibility();
    const auto t0 = std::chrono::high_resolution_clock::now();
    fn();
    const auto t1 = std::chrono::high_resolution_clock::now();

    // Capture output (NOT timed)
    out.resize(n);
    std::memcpy(out.data(), vis, bytes);

    return std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
        .count();
  };

  auto exact_match = [&](const std::vector<double> &a,
                         const std::vector<double> &b) -> bool {
    if (a.size() != b.size())
      return false;
    return std::memcmp(a.data(), b.data(), bytes) == 0;
  };

  // --- Run 3 methods ---
  std::vector<double> out_orig, out_row, out_par;

  const long long t_orig = run_and_capture(
      "original", [&]() { computeVisibility_original(); }, out_orig);

  const long long t_row = run_and_capture(
      "row-opt", [&]() { computeVisibility_rowOptimizedSerial(); }, out_row);

  const long long t_par = run_and_capture(
      "parallel", [&]() { computeVisibility_parallelSections(); }, out_par);

  // --- Exactness checks (NOT timed) ---
  const bool ok_row = exact_match(out_row, out_orig);
  const bool ok_par = exact_match(out_par, out_orig);

  // --- Print results ---
  if (!sharedConfig_->silent) {
    std::cout
        << "================ Visibility kernel benchmark ================\n";
    std::cout << "Grid: " << nx_ << " x " << ny_ << " (N=" << (nx_ * ny_)
              << ")\n\n";

    std::cout << std::left << std::setw(22) << "Method" << std::setw(14)
              << "time (us)" << std::setw(14) << "speedup"
              << "exact?\n";
    std::cout
        << "-------------------------------------------------------------\n";

    auto speedup = [&](long long t) -> double {
      return (t > 0) ? (double)t_orig / (double)t : 0.0;
    };

    std::cout << std::left << std::setw(22) << "Original" << std::setw(14)
              << t_orig << std::setw(14) << 1.0 << "baseline\n";

    std::cout << std::left << std::setw(22) << "Row-optimized (serial)"
              << std::setw(14) << t_row << std::setw(14) << speedup(t_row)
              << (ok_row ? "EXACT ✅" : "MISMATCH ❌") << "\n";

    std::cout << std::left << std::setw(22) << "Parallel (sections)"
              << std::setw(14) << t_par << std::setw(14) << speedup(t_par)
              << (ok_par ? "EXACT ✅" : "MISMATCH ❌") << "\n";
    std::cout
        << "=============================================================\n";
  }

  // --- Restore the fastest output into visibility_ for saving/visualization
  // --- Prefer fastest among row/parallel (original is baseline).
  const std::vector<double> *best = &out_row;
  long long t_best = t_row;

  if (t_par > 0 && t_par < t_best) {
    best = &out_par;
    t_best = t_par;
  }

  std::memcpy(vis, best->data(), bytes);
  saveStandAloneVisibility();

  // --- Keep your raycasting benchmark (optional; can comment out) ---
  auto time_start2 = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      raycasting(ls_.first, ls_.second, (int)i, (int)j);
    }
  }
  auto time_stop2 = std::chrono::high_resolution_clock::now();
  auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(
      time_stop2 - time_start2);

  saveRayCastingVisibility();

  if (!sharedConfig_->silent) {
    std::cout << "Raycasting computation time in us: " << duration2.count()
              << "us\n";
    std::cout << "Ratio (raycasting / best-visibility): "
              << (double)duration2.count() / (double)t_best << "\n";
  }

  // Density print (unchanged)
  int count = 0;
  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      if (occupancyComplement_->get(i, j) == 0)
        ++count;
    }
  }
  std::cout << "Density of the occupancy grid: "
            << (double)count / (nx_ * ny_) * 100 << "%\n";
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool visibilityBasedSolver::validateVisibilityKernels() {
  auto start = sharedConfig_->start;
  if (sharedConfig_->mode == 2) {
    start.second = ny_ - 1 - start.second;
  }

  if (!isValid(start.first, start.second) ||
      occupancyComplement_->get(start.first, start.second) == 0) {
    return false;
  }

  ls_ = start;
  auto *vis = visibility_.data();
  const size_t n = visibility_.size();
  const size_t bytes = n * sizeof(double);

  auto capture = [&](auto &&fn) {
    std::fill(vis, vis + n, 0.0);
    fn();
    std::vector<double> result(n);
    std::memcpy(result.data(), vis, bytes);
    return result;
  };

  const auto original =
      capture([&]() { computeVisibility_original(); });
  const auto rowOptimized =
      capture([&]() { computeVisibility_rowOptimizedSerial(); });
  const auto parallel =
      capture([&]() { computeVisibility_parallelSections(); });

  const bool rowMatches =
      std::memcmp(rowOptimized.data(), original.data(), bytes) == 0;
  const bool parallelMatches =
      std::memcmp(parallel.data(), original.data(), bytes) == 0;

  std::memcpy(vis, parallel.data(), bytes);
  return rowMatches && parallelMatches;
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
    computeVisibility_parallelSections();
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
      } else if (i == j) {
        v = visibility_(currentX - 1, currentY - 1);
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
      } else if (i == j) {
        v = visibility_(currentX + 1, currentY - 1);
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
      } else if (i == j) {
        v = visibility_(currentX + 1, currentY + 1);
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
      } else if (i == j) {
        v = visibility_(currentX - 1, currentY + 1);
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
bool vbs::visibilityBasedSolver::updateVisibilityAndPickBest(Node &bestOut) {
  // 1) Compute local visibility from current ls_ using your fastest exact
  // kernel.
  //    This overwrites the entire visibility_ field (no need to clear it).
  computeVisibility_parallelSections();
  // computeVisibility_original();
  // computeVisibility_rowOptimizedSerial();

  auto *vis = visibility_.data();
  auto *glob = visibility_global_.data();
  auto *cf = cameFrom_.data();

  const size_t nx = nx_;
  const size_t ny = ny_;
  const size_t N = nx * ny;

  const size_t endx = static_cast<size_t>(end_.first);
  const size_t endy = static_cast<size_t>(end_.second);

  std::less<Node> cmp; // same comparator you used in priority_queue
  bool haveBest = false;
  Node best{0, 0, 0.0};

  // Scan all cells:
  // - update global visibility
  // - set cameFrom once when local vis crosses threshold
  // - select best candidate according to same ordering as your heap top
#ifdef _OPENMP
#pragma omp parallel
#endif
  {
    std::less<Node> lcmp;
    bool lhaveBest = false;
    Node lbest{0, 0, 0.0};

#ifdef _OPENMP
#pragma omp for nowait
#endif
    for (long long idx_ll = 0; idx_ll < static_cast<long long>(N); ++idx_ll) {
      const size_t idx = static_cast<size_t>(idx_ll);

      const double v = vis[idx];

      // global = max(global, local)
      double g = glob[idx];
      if (v > g) {
        g = v;
        glob[idx] = v;
      }

      // cameFrom: first source that makes local >= threshold
      if (v >= visibilityThreshold_ && cf[idx] == CAMEFROM_UNSET) {
        cf[idx] = static_cast<double>(nb_of_sources_);
      }

      // Candidate set is based on GLOBAL threshold (same as your original code)
      if (g >= visibilityThreshold_) {
        const size_t x = idx % nx;
        const size_t y = idx / nx;

        const size_t parentIdx = static_cast<size_t>(cf[idx]);
        const point parent = lightSources_[parentIdx];

        const double h =
            (scale_ * g) + (eval_d(x, y, endx, endy) +
                            eval_d(x, y, static_cast<size_t>(parent.first),
                                   static_cast<size_t>(parent.second)));

        Node cand{x, y, h};

        // pick "best" exactly like priority_queue(top) would
        if (!lhaveBest || lcmp(lbest, cand)) {
          lbest = cand;
          lhaveBest = true;
        }
      }
    }

#ifdef _OPENMP
#pragma omp critical
#endif
    {
      if (lhaveBest && (!haveBest || cmp(best, lbest))) {
        best = lbest;
        haveBest = true;
      }
    }
  }

  if (!haveBest)
    return false;
  bestOut = best;
  return true;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::computeVisibility_original() {
  size_t currentX, currentY;
  double v = 0.0;
  double offset = 0.0;

  // Q1 (right, down) — already covers x=nx-1 and y=ny-1
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
      } else if (i == j) {
        v = visibility_(currentX - 1, currentY - 1);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second + offset) /
             (currentX - ls_.first + offset);
        v = visibility_(currentX - 1, currentY) -
            c_ * (visibility_(currentX - 1, currentY) -
                  visibility_(currentX - 1, currentY - 1));
      } else { // j > i
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

  // Q2 (left, down) — FIX: include x=0 by using lx+1
  max_x_ = ls_.first + 1; // was: ls_.first
  max_y_ = ny_ - ls_.second;

  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first - i; // now reaches 0 when i==lx
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second + j;

      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY - 1);
      } else if (j == 0) {
        v = visibility_(currentX + 1, currentY);
      } else if (i == j) {
        v = visibility_(currentX + 1, currentY - 1);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second + offset) /
             (ls_.first - currentX + offset);
        v = visibility_(currentX + 1, currentY) -
            c_ * (visibility_(currentX + 1, currentY) -
                  visibility_(currentX + 1, currentY - 1));
      } else { // j > i
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

  // Q3 (left, up) — FIX: include x=0 and y=0 by using +1 on both
  max_x_ = ls_.first + 1;  // was: ls_.first
  max_y_ = ls_.second + 1; // was: ls_.second

  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first - i; // reaches 0
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second - j; // reaches 0

      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY + 1);
      } else if (j == 0) {
        v = visibility_(currentX + 1, currentY);
      } else if (i == j) {
        v = visibility_(currentX + 1, currentY + 1);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY + offset) /
             (ls_.first - currentX + offset);
        v = visibility_(currentX + 1, currentY) -
            c_ * (visibility_(currentX + 1, currentY) -
                  visibility_(currentX + 1, currentY + 1));
      } else { // j > i
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

  // Q4 (right, up) — FIX: include y=0 by using ly+1
  max_x_ = nx_ - ls_.first;
  max_y_ = ls_.second + 1; // was: ls_.second

  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second - j; // now reaches 0

      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY + 1);
      } else if (j == 0) {
        v = visibility_(currentX - 1, currentY);
      } else if (i == j) {
        v = visibility_(currentX - 1, currentY + 1);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY + offset) /
             (currentX - ls_.first + offset);
        v = visibility_(currentX - 1, currentY) -
            c_ * (visibility_(currentX - 1, currentY) -
                  visibility_(currentX - 1, currentY + 1));
      } else { // j > i
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
void visibilityBasedSolver::computeVisibility_rowOptimizedSerial() {
  // Performance notes:
  // - Field<T> is stored as data[x + y*nx] => x is contiguous in memory.
  // - We traverse with x as the inner loop (per quadrant) for cache
  // locality/prefetch.
  // - Use raw pointers (Field::data()) and precomputed row offsets to avoid
  // repeated
  //   (x,y)->linear index calculations in tight loops.
  // Q1 (x+, y+): x increases, y increases. Inner loop over x => contiguous
  // writes.
  auto *vis = visibility_.data();
  const auto *occ = occupancyComplement_->data();
  const size_t nx = nx_;
  const size_t ny = ny_;
  const size_t lx = ls_.first;
  const size_t ly = ls_.second;

  for (size_t y = ly; y < ny; ++y) {
    const size_t j = y - ly;
    const size_t row = y * nx;
    const size_t row_m1 = (y - 1) * nx;

    for (size_t x = lx; x < nx; ++x) {
      const size_t i = x - lx;
      const size_t idx = row + x;

      double v;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = vis[row_m1 + x]; // (x, y-1)
      } else if (j == 0) {
        v = vis[row + (x - 1)]; // (x-1, y)
      } else if (i == j) {
        v = vis[row_m1 + (x - 1)]; // (x-1, y-1)
      } else if (i > j) {
        const double c = double(j) / double(i);
        const double uP = vis[row + (x - 1)];
        const double uD = vis[row_m1 + (x - 1)];
        v = uP - c * (uP - uD);
      } else { // j > i
        const double c = double(i) / double(j);
        const double uP = vis[row_m1 + x];
        const double uD = vis[row_m1 + (x - 1)];
        v = uP - c * (uP - uD);
      }

      vis[idx] = v * occ[idx];
    }
  }

  // Q2 (x-, y+): x decreases but still contiguous in memory; avoid size_t
  // underflow.
  for (size_t y = ly; y < ny; ++y) {
    const size_t j = y - ly;
    const size_t row = y * nx;
    const size_t row_m1 = (y - 1) * nx;

    // x runs from lx down to 0 (inclusive)
    for (size_t x = lx; /*x>=0*/; --x) {
      const size_t i = lx - x; // >= 0
      const size_t idx = row + x;

      double v;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        // from (x, y-1)
        v = vis[row_m1 + x];
      } else if (j == 0) {
        // from (x+1, y)
        v = vis[row + (x + 1)];
      } else if (i == j) {
        // from (x+1, y-1)
        v = vis[row_m1 + (x + 1)];
      } else if (i > j) {
        // c = (dy)/(dx) where dx is negative => c = j/(-i)
        const double c = double(j) / (-double(i));
        const double uP = vis[row + (x + 1)];
        const double uD = vis[row_m1 + (x + 1)];
        v = uP + c * (uP - uD);
      } else { // j > i
        // c = dx/dy where dx is negative => c = (-i)/j
        const double c = (-double(i)) / double(j);
        const double uP = vis[row_m1 + x];
        const double uD = vis[row_m1 + (x + 1)];
        v = uP + c * (uP - uD);
      }

      vis[idx] = v * occ[idx];

      if (x == 0)
        break; // size_t underflow guard
    }
  }

  // Q3 (x-, y-): same contiguous access, y decreases (row_p1 used for y+1
  // neighbor).
  for (size_t y = ly; /*y>=0*/; --y) {
    const size_t j = ly - y;
    const size_t row = y * nx;
    const size_t row_p1 = (y + 1) * nx;

    // x runs from lx down to 0
    for (size_t x = lx; /*x>=0*/; --x) {
      const size_t i = lx - x;
      const size_t idx = row + x;

      double v;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        // from (x, y+1)
        v = vis[row_p1 + x];
      } else if (j == 0) {
        // from (x+1, y)
        v = vis[row + (x + 1)];
      } else if (i == j) {
        // from (x+1, y+1)
        v = vis[row_p1 + (x + 1)];
      } else if (i > j) {
        // dy negative, dx negative => c = (-j)/(-i) = j/i
        const double c = double(j) / double(i);
        const double uP = vis[row + (x + 1)];
        const double uD = vis[row_p1 + (x + 1)];
        v = uP - c * (uP - uD);
      } else { // j > i
        // c = dx/dy = (-i)/(-j) = i/j
        const double c = double(i) / double(j);
        const double uP = vis[row_p1 + x];
        const double uD = vis[row_p1 + (x + 1)];
        v = uP - c * (uP - uD);
      }

      vis[idx] = v * occ[idx];

      if (x == 0)
        break;
    }

    if (y == 0)
      break;
  }

  // Q4 (x+, y-): x increases, y decreases.
  for (size_t y = ly; /*y>=0*/; --y) {
    const size_t j = ly - y;
    const size_t row = y * nx;
    const size_t row_p1 = (y + 1) * nx;

    for (size_t x = lx; x < nx; ++x) {
      const size_t i = x - lx;
      const size_t idx = row + x;

      double v;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        // from (x, y+1)
        v = vis[row_p1 + x];
      } else if (j == 0) {
        // from (x-1, y)
        v = vis[row + (x - 1)];
      } else if (i == j) {
        // from (x-1, y+1)
        v = vis[row_p1 + (x - 1)];
      } else if (i > j) {
        // dy negative, dx positive => c = (-j)/i
        const double c = (-double(j)) / double(i);
        const double uP = vis[row + (x - 1)];
        const double uD = vis[row_p1 + (x - 1)];
        v = uP + c * (uP - uD);
      } else { // j > i
        // c = dx/dy = i/(-j)
        const double c = double(i) / (-double(j));
        const double uP = vis[row_p1 + x];
        const double uD = vis[row_p1 + (x - 1)];
        v = uP + c * (uP - uD);
      }

      vis[idx] = v * occ[idx];
    }

    if (y == 0)
      break;
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
// What this does:
// - Keeps your fast row-wise (x-inner) traversal for cache locality.
// - Parallelizes only across the 4 disjoint quadrants using OpenMP sections.
// - Avoids races by partitioning the grid into non-overlapping regions:
//     Q1: x >= lx, y >= ly  (but we SKIP the source cell)
//     Q2: x <  lx, y >= ly
//     Q3: x <  lx, y <  ly
//     Q4: x >= lx, y <  ly
//   so no quadrant writes the same cell.
//
// Build:
// - Ensure CMake links OpenMP (you already do via OpenMP::OpenMP_CXX).

// This version fixes the correctness issue in the previous "parallel sections"
// attempt: Q3/Q4 (and Q2/Q3 near the axes) depend on values on the source
// row/column. If those axis values are computed by a *different* OpenMP section
// concurrently, you get races and wrong results.
//
// Fix: compute the full source row (y=ly) and source column (x=lx) serially
// first, then run 4 OpenMP sections on the *interiors* of each quadrant
// (disjoint regions).
//
// Math matches the original implementation (same neighbor choices and same c
// ratios). The only extra is an optional diagonal (i==j) branch; keep it if you
// want the intended behavior, or remove it to mimic the original
// "no-diagonal-case" behavior.
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void vbs::visibilityBasedSolver::computeVisibility_parallelSections() {
  auto *vis = visibility_.data();
  const auto *occ = occupancyComplement_->data();
  const size_t nx = nx_;
  const size_t ny = ny_;
  const size_t lx = ls_.first;
  const size_t ly = ls_.second;

  // --- 0) Source ---
  const size_t src = lx + ly * nx;
  vis[src] = lightStrength_ * occ[src];

  // --- 1) Precompute axes (serial) ---
  // Source row y=ly (right side)
  if (ly < ny) {
    const size_t row = ly * nx;
    for (size_t x = lx + 1; x < nx; ++x) {
      const size_t idx = row + x;
      const double v = vis[idx - 1]; // (x-1, ly)
      vis[idx] = v * occ[idx];
    }
    // Source row y=ly (left side)
    if (lx > 0) {
      for (size_t x = lx; x-- > 0;) {
        const size_t idx = row + x;
        const double v = vis[idx + 1]; // (x+1, ly)
        vis[idx] = v * occ[idx];
      }
    }
  }

  // Source column x=lx (downwards, y increasing)
  if (lx < nx) {
    for (size_t y = ly + 1; y < ny; ++y) {
      const size_t idx = lx + y * nx;
      const double v = vis[idx - nx]; // (lx, y-1)
      vis[idx] = v * occ[idx];
    }
    // Source column x=lx (upwards, y decreasing)
    if (ly > 0) {
      for (size_t y = ly; y-- > 0;) {
        const size_t idx = lx + y * nx;
        const double v = vis[idx + nx]; // (lx, y+1)
        vis[idx] = v * occ[idx];
      }
    }
  }

  // --- 2) Quadrant interiors in parallel (disjoint writes) ---
  // Q1 interior: x in [lx+1..nx-1], y in [ly+1..ny-1]
  // Q2 interior: x in [0..lx-1],    y in [ly+1..ny-1]
  // Q3 interior: x in [0..lx-1],    y in [0..ly-1]
  // Q4 interior: x in [lx+1..nx-1], y in [0..ly-1]
  //
  // Only 4 sections => usually best to cap threads at 4 for lower overhead.
#ifdef _OPENMP
#pragma omp parallel sections num_threads(4)
  {
#pragma omp section
    {
      computeQ1_rowwise(vis, occ, nx, ny, lx, ly);
    }
#pragma omp section
    {
      computeQ2_rowwise(vis, occ, nx, ny, lx, ly);
    }
#pragma omp section
    {
      computeQ3_rowwise(vis, occ, nx, ny, lx, ly);
    }
#pragma omp section
    {
      computeQ4_rowwise(vis, occ, nx, ny, lx, ly);
    }
  }
#else
  computeQ1_rowwise(vis, occ, nx, ny, lx, ly);
  computeQ2_rowwise(vis, occ, nx, ny, lx, ly);
  computeQ3_rowwise(vis, occ, nx, ny, lx, ly);
  computeQ4_rowwise(vis, occ, nx, ny, lx, ly);
#endif
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
// -------------------------
// Q1 interior (x+, y+)
// x = lx + i, y = ly + j, with i>=1, j>=1
// Original (Q1):
//   if i>j: c=j/i; v=U(x-1,y) - c*(U(x-1,y)-U(x-1,y-1))
//   if j>i: c=i/j; v=U(x,y-1) - c*(U(x,y-1)-U(x-1,y-1))
// -------------------------
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void vbs::visibilityBasedSolver::computeQ1_rowwise(double *vis,
                                                   const double *occ, size_t nx,
                                                   size_t ny, size_t lx,
                                                   size_t ly) const {

  if (lx + 1 >= nx || ly + 1 >= ny)
    return;

  for (size_t y = ly + 1; y < ny; ++y) {
    const size_t j = y - ly;
    const size_t row = y * nx;
    const size_t row_m1 = (y - 1) * nx;

    for (size_t x = lx + 1; x < nx; ++x) {
      const size_t i = x - lx;
      const size_t idx = row + x;

      double v;

      // Optional diagonal fix (remove to mimic original behavior exactly)
      if (i == j) {
        v = vis[row_m1 + (x - 1)]; // (x-1, y-1)
      } else if (i > j) {
        const double c = double(j) / double(i);
        const double uP = vis[row + (x - 1)];    // (x-1, y)
        const double uD = vis[row_m1 + (x - 1)]; // (x-1, y-1)
        v = uP - c * (uP - uD);
      } else { // j > i
        const double c = double(i) / double(j);
        const double uP = vis[row_m1 + x];       // (x, y-1)
        const double uD = vis[row_m1 + (x - 1)]; // (x-1, y-1)
        v = uP - c * (uP - uD);
      }

      vis[idx] = v * occ[idx];
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
// -------------------------
// Q2 interior (x-, y+)
// x = lx - i, y = ly + j, with i>=1, j>=1
// Original (Q2):
//   if i>j: c=j/i; v=U(x+1,y) - c*(U(x+1,y)-U(x+1,y-1))
//   if j>i: c=i/j; v=U(x,y-1) - c*(U(x,y-1)-U(x+1,y-1))
// -------------------------
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void vbs::visibilityBasedSolver::computeQ2_rowwise(double *vis,
                                                   const double *occ, size_t nx,
                                                   size_t ny, size_t lx,
                                                   size_t ly) const {

  if (lx == 0 || ly + 1 >= ny)
    return;

  for (size_t y = ly + 1; y < ny; ++y) {
    const size_t j = y - ly;
    const size_t row = y * nx;
    const size_t row_m1 = (y - 1) * nx;

    // x: lx-1 .. 0 (decreasing so (x+1,y) is already computed in this row)
    for (size_t x = lx; x-- > 0;) {
      const size_t i = lx - x;
      const size_t idx = row + x;

      double v;

      // Optional diagonal fix (remove to mimic original behavior exactly)
      if (i == j) {
        v = vis[row_m1 + (x + 1)]; // (x+1, y-1)
      } else if (i > j) {
        const double c = double(j) / double(i);
        const double uP = vis[row + (x + 1)];    // (x+1, y)
        const double uD = vis[row_m1 + (x + 1)]; // (x+1, y-1)
        v = uP - c * (uP - uD);
      } else { // j > i
        const double c = double(i) / double(j);
        const double uP = vis[row_m1 + x];       // (x, y-1)
        const double uD = vis[row_m1 + (x + 1)]; // (x+1, y-1)
        v = uP - c * (uP - uD);
      }

      vis[idx] = v * occ[idx];
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
// -------------------------
// Q3 interior (x-, y-)
// x = lx - i, y = ly - j, with i>=1, j>=1
// Original (Q3):
//   if i>j: c=j/i; v=U(x+1,y) - c*(U(x+1,y)-U(x+1,y+1))
//   if j>i: c=i/j; v=U(x,y+1) - c*(U(x,y+1)-U(x+1,y+1))
// -------------------------
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void vbs::visibilityBasedSolver::computeQ3_rowwise(double *vis,
                                                   const double *occ, size_t nx,
                                                   size_t /*ny*/, size_t lx,
                                                   size_t ly) const {

  if (lx == 0 || ly == 0)
    return;

  // y: ly-1 .. 0 (decreasing so (x, y+1) is already computed)
  for (size_t y = ly; y-- > 0;) {
    const size_t j = ly - y;
    const size_t row = y * nx;
    const size_t row_p1 = (y + 1) * nx;

    // x: lx-1 .. 0 (decreasing so (x+1, y) is already computed)
    for (size_t x = lx; x-- > 0;) {
      const size_t i = lx - x;
      const size_t idx = row + x;

      double v;

      // Optional diagonal fix (remove to mimic original behavior exactly)
      if (i == j) {
        v = vis[row_p1 + (x + 1)]; // (x+1, y+1)
      } else if (i > j) {
        const double c = double(j) / double(i);
        const double uP = vis[row + (x + 1)];    // (x+1, y)
        const double uD = vis[row_p1 + (x + 1)]; // (x+1, y+1)
        v = uP - c * (uP - uD);
      } else { // j > i
        const double c = double(i) / double(j);
        const double uP = vis[row_p1 + x];       // (x, y+1)
        const double uD = vis[row_p1 + (x + 1)]; // (x+1, y+1)
        v = uP - c * (uP - uD);
      }

      vis[idx] = v * occ[idx];
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
// -------------------------
// Q4 interior (x+, y-)
// x = lx + i, y = ly - j, with i>=1, j>=1
// Original (Q4):
//   if i>j: c=j/i; v=U(x-1,y) - c*(U(x-1,y)-U(x-1,y+1))
//   if j>i: c=i/j; v=U(x,y+1) - c*(U(x,y+1)-U(x-1,y+1))
// -------------------------
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void vbs::visibilityBasedSolver::computeQ4_rowwise(double *vis,
                                                   const double *occ, size_t nx,
                                                   size_t /*ny*/, size_t lx,
                                                   size_t ly) const {

  if (lx + 1 >= nx || ly == 0)
    return;

  // y: ly-1 .. 0 (decreasing so (x, y+1) is already computed)
  for (size_t y = ly; y-- > 0;) {
    const size_t j = ly - y;
    const size_t row = y * nx;
    const size_t row_p1 = (y + 1) * nx;

    for (size_t x = lx + 1; x < nx; ++x) {
      const size_t i = x - lx;
      const size_t idx = row + x;

      double v;

      // Optional diagonal fix (remove to mimic original behavior exactly)
      if (i == j) {
        v = vis[row_p1 + (x - 1)]; // (x-1, y+1)
      } else if (i > j) {
        const double c = double(j) / double(i);
        const double uP = vis[row + (x - 1)];    // (x-1, y)
        const double uD = vis[row_p1 + (x - 1)]; // (x-1, y+1)
        v = uP - c * (uP - uD);
      } else { // j > i
        const double c = double(i) / double(j);
        const double uP = vis[row_p1 + x];       // (x, y+1)
        const double uD = vis[row_p1 + (x - 1)]; // (x-1, y+1)
        v = uP - c * (uP - uD);
      }

      vis[idx] = v * occ[idx];
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void visibilityBasedSolver::computeVisibilityUsingQueue() {
  double v = 0.0;
  point current;

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
    if (!isValid(x, y)) {
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
