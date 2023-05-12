#ifndef VISIBILITYBASEDSOLVER_H
#define VISIBILITYBASEDSOLVER_H

#include "environment/environment.h"

#include <cmath>
#include <algorithm>
#include <vector>
#include <queue>

namespace vbs {

// Node structure that is maintained by the heap: a double distance value for every x, y size_t values
struct Node {
  size_t x, y;
  double h;

  bool operator<(const Node& other) const {
    return h > other.h;
  }
};

class visibilityBasedSolver {
 public:
  /*!
   * Constructor.
   * @brief Initialize the solver with an environment.
   * @param [in] env Environment reference.
   */
  visibilityBasedSolver(environment& env);
  // Deconstructor
  ~visibilityBasedSolver() = default;

  // Get global iterator (number of iterations that had to be completed)
  inline int getGlobalIter() const { return globalIter; };

  // Solve
  void solve();

 private:

  void reset();
  Field<double, 1> occupancyComplement_;
  Field<double, 0> visibility_global_;
  Field<double, 0> visibility_;
  Field<size_t, 0> cameFrom_;
  Field<bool, 0> isUpdated_;
  std::unique_ptr<point[]> lightSources_;

  std::shared_ptr<Config> sharedConfig_;

  // Dimensions.
  size_t nrows_; size_t ncols_;

  /*!
   * @brief Currently a Euclidean measure of the distance but can be generalized.
   * @param [in] source_x P1_x.
   * @param [in] source_y P1_y.
   * @param [in] target_x P2_x.
   * @param [in] target_y P2_y.
   */
  inline double eval_d(int source_x, int source_y, int target_x, int target_y) { 
    return sqrt((double)(source_x-target_x) * (source_x-target_x) + (source_y-target_y) * (source_y-target_y));
  };

  // Reset queue
  void resetQueue();

  // visibility update
  void updateVisibility();

  // Save results
  void saveResults();

  // Heap to maintain the heuristic
  std::unique_ptr<std::priority_queue<Node>> heap_;

  // Number of lightsources/pivots.
  size_t nb_of_sources_ = 0;
  // Lightstrength, can be decreased. Can add later an alpha that has light decay, enforcing adding a new pivot periodically.
  const double lightStrength_ = 1.0;
  // Visibility threshold
  double visibilityThreshold_;
  // Ratio used in PDE update.
  double c_ = 1.0;
  // Global iterator.
  int globalIter = 0;

  // For computing the heuristic
  double f_min_;
  double d_tot_min;
  double d_tot_max;
  double vmin_;
  double vmax_;

  // max iters
  size_t max_iter_ = 0;

  // lightsource
  point ls_;
  point parent_;
  point end_;

  // Point holds the minimum heuristic
  point pt_min_;

  // Dimensions
  size_t max_col_, max_row_;

  // For scaling the visibility in the heuristic
  double scale_ = 0;
};

} // namespace vbs
#endif