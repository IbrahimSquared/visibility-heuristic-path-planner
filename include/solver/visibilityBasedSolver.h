#ifndef VISIBILITYBASEDSOLVER_H
#define VISIBILITYBASEDSOLVER_H

#include "environment/environment.h"

#include <cmath>
#include <queue>
#include <vector>

#include <SFML/Graphics.hpp>

namespace vbs {

// Node structure that is maintained by the heap: a double distance value for
// every x, y size_t values
struct Node {
  size_t x, y;
  double h;

  bool operator<(const Node &other) const { return h > other.h; }
};

class visibilityBasedSolver {
public:
  /*!
   * Constructor.
   * @brief Initialize the solver with an environment.
   * @param [in] env Environment reference.
   */
  explicit visibilityBasedSolver(environment &env);
  // Deconstructor
  ~visibilityBasedSolver() = default;

  // Get global iterator (number of iterations that had to be completed)
  inline int getGlobalIter() const { return globalIter; };

  // Solve
  void solve();

  // Compute standAloneVisibility
  void standAloneVisibility();

  // save standAloneVisibility results
  void saveStandAloneVisibility();

  // save standAloneVisibility results
  void saveRayCastingVisibility();

  /*!
   * @brief Benchmark by comparing the visibility computation
   * against raycasting for one environment that has settings
   * in the config file.
   */
  void benchmark();

  /*!
   * @brief Benchmark by comparing the visibility computation
   * against raycasting for a series of environments of increasing
   * size.
   */
  void benchmarkSeries();

  /*!
   * @brief Compute visibility using a typical raycasting algorithm. Enumerate
   * in a map the number of times each cell is traveresed.
   * @param [in] x0 Start x.
   * @param [in] y0 Start y.
   * @param [in] x1 End x.
   * @param [in] y1 End y.
   */
  void raycasting(int x0, int y0, int x1, int y1);

private:
  void reset();
  std::shared_ptr<Field<double>> occupancyComplement_;
  Field<double> visibility_global_;
  Field<double> visibility_;
  Field<double> visibilityRayCasting_;
  Field<size_t> cameFrom_;
  Field<bool> isUpdated_;

  std::unique_ptr<point[]> lightSources_;

  std::shared_ptr<Config> sharedConfig_;

  std::unique_ptr<sf::Image> uniqueLoadedImage_;

  void reconstructPath(const Node &current, std::vector<point> &resultingPath);

  // Dimensions.
  size_t ny_;
  size_t nx_;

  /*!
   * @brief Currently a Euclidean measure of the distance but can be
   * generalized.
   * @param [in] source_x P1_x.
   * @param [in] source_y P1_y.
   * @param [in] target_x P2_x.
   * @param [in] target_y P2_y.
   */
  inline double eval_d(int source_x, int source_y, int target_x, int target_y) {
    return sqrt((double)(source_x - target_x) * (source_x - target_x) +
                (source_y - target_y) * (source_y - target_y));
  };

  // Reset queue
  void resetQueue();

  // visibility update
  void updateVisibility();

  // Stand-alone visibility computation (Algorithm 1 in the paper). More
  // suitable for sparse environments.
  void computeVisibility();

  /*!
   * @brief Stand-alone visibility computation using a queue. It has termination
   * conditions. generalized. More suitable for denser environments.
   */
  void computeVisibilityUsingQueue();

  // Save results
  void saveResults();
  void saveImageWithPath(const std::vector<point> &path);

  // Heap to maintain the heuristic
  std::unique_ptr<std::priority_queue<Node>> heap_;

  // Number of lightsources/pivots.
  size_t nb_of_sources_ = 0;
  // Lightstrength, can be decreased. Can add later an alpha that has light
  // decay, enforcing adding a new pivot periodically.
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
  size_t max_x_, max_y_;

  // For scaling the visibility in the heuristic
  double scale_ = 0;
};

} // namespace vbs
#endif
