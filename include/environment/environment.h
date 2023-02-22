#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "environment/Field.h"
#include "parser/ConfigParser.h"

#include <memory>
#include <vector>

#include <SFML/Graphics.hpp>

namespace vbs {

using FloatField = Field<float>;

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
   */
  void generateNewEnvironment(size_t nrows, size_t ncols, int nb_of_obstacles, 
    int min_width, int max_width, int min_height, int max_height, int seedValue = 0);

  /*!
   * @brief Generate a random new environment from parsed config settings. Overwrites previously generated environment.
   */
  void generateNewEnvironment();

  /*!
   * @brief Loads image data using SFML. Overwrites previously generated environment.
   * @param [in] filename Filename.
   */
  void loadImage(const std::string& filename);

  /*!
   * @brief Helper function to load maps.
   */
  std::vector<float> stringToFloatVector(const std::string& str, char delimiter);

  // Get visibility field shared pointer.
  inline std::shared_ptr<FloatField> getVisibilityField() const { return visibilityField_; };
  // Get speed field shared pointer.
  inline std::shared_ptr<FloatField> getSpeedField() const { return speedField_; };

  inline const std::shared_ptr<Config> getConfig() const { return config_; };

  // Deconstructor
  ~environment() = default;

 private:
  size_t nrows_;
  size_t ncols_;
  size_t size_;
  const float speedValue_ = 2.0;
  int seedValue_ = 1;

  // Shared pointer to a Field object of type float that has map visibility values (complement of occupancy grid).
  std::shared_ptr<FloatField> visibilityField_;
  // Shared pointer to a Field object of type float that has map speed values.
  std::shared_ptr<FloatField> speedField_;

  // Shared pointer to configuration
  std::shared_ptr<Config> config_;

  // Unique pointer to image holder
  std::unique_ptr<sf::Image> loadedImage_;
};

} // namespace vbs
#endif