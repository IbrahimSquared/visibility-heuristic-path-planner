#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "environment/Field.h"
#include "parser/ConfigParser.h"

#include <memory>
#include <vector>
#include <filesystem>

#include <SFML/Graphics.hpp>

namespace vbs {

// Environment simulator
class environment {
 public:
  /*!
   * Constructor.
   * @brief Initialize an environment.
   */
  environment(Config& config);

  /*!
   * @brief Generate a random new environment on request. Overwrites previously generated environment.
   * @param [in] nrows Number of rows.
   * @param [in] ncols Number of cols.
   * @param [in] nb_of_obstacles Number of obstacles.
   * @param [in] min_width Min obstacle width.
   * @param [in] max_width Max obstacle width.
   * @param [in] min_height Min obstacle height.
   * @param [in] max_height Max obstacle height.
   * @param [in] seedValue 0 by default.
   */
  void generateNewEnvironment(size_t nrows, size_t ncols, int nb_of_obstacles, 
    int min_width, int max_width, int min_height, int max_height, int seedValue = 0);

  /*!
   * @brief Generate a random new environment from parsed config settings. Overwrites previously generated environment.
   */
  void generateNewEnvironmentFromSettings();

  /*!
   * @brief Loads image data using SFML. Overwrites previously generated environment.
   * @param [in] filename Filename.
   */
  void loadImage(const std::string& filename);

  void loadMaps(const std::string& filename);
  std::vector<float> stringToFloatVector(const std::string& str, char delimiter);

  // Get visibility field shared pointer.
  inline const auto& getVisibilityField() const { return sharedVisibilityField_; };
  // Get speed field shared pointer.
  inline const auto& getSpeedField() const { return sharedSpeedField_; };
  // Get parsed configuration.
  inline const auto& getConfig() const { return sharedConfig_; };

  // Deconstructor
  ~environment() = default;

 private:
  size_t ny_;
  size_t nx_;
  size_t size_;
  double speedValue_ = 2.0;
  int seedValue_ = 1;

  // Shared pointer to a Field object of type double that has map occupancy values.
  std::shared_ptr<Field<double>> sharedVisibilityField_;
  std::shared_ptr<Field<double>> sharedSpeedField_;

  // Shared pointer to configuration
  std::shared_ptr<Config> sharedConfig_;
  // Unique pointer to image holder
  std::unique_ptr<sf::Image> uniqueLoadedImage_;

  void saveEnvironment();
  void resetEnvironment();
};

} // namespace vbs
#endif