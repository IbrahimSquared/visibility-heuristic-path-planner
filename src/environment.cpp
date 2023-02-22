#include "environment/environment.h"

#include <algorithm>
#include <ctime>

#include <iostream>
#include <fstream>
#include <sstream>

namespace vbs {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
environment::environment(Config& config) : config_(std::make_shared<Config>(config)) {
  if (config_->randomEnvironment) {
    nrows_ = config_->nrows;
    ncols_ = config_->ncols;
    if (!config_->randomSeed) {
      seedValue_ = config_->seedValue;
    }
    generateNewEnvironment();
  } else if (config_->importImage) {
    loadImage(config_->imagePath);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void environment::generateNewEnvironment() {
  // Resets environment
  visibilityField_.reset(new FloatField(nrows_, ncols_, 1.0));
    speedField_.reset(new FloatField(nrows_, ncols_, 1.0));

  int seedValue = 0;
  if (!config_->randomSeed) {
    seedValue = seedValue_;
  } else {
    seedValue = std::time(0);
  }
  std::srand(seedValue);
  
  for(size_t i = 0; i < config_->nb_of_obstacles; ++i) {
    int row_1 = 1 + (std::rand() % (nrows_ - 0 + 1));
    int row_2 = row_1 + config_->minWidth + (std::rand() % (config_->maxWidth - config_->minWidth + 1));
    row_1 = std::min(row_1, (int)nrows_ - 1); row_2 = std::min(row_2,(int) nrows_ - 1); 

    int col_1 = 1 + (std::rand() % (ncols_ - 0 + 1));
    int col_2 = col_1 + config_->minHeight + (std::rand() % (config_->maxHeight - config_->minHeight + 1));
    col_1 = std::min(col_1, (int)ncols_ - 1); col_2 = std::min(col_2, (int)ncols_ - 1); 

    for (int j = row_1; j < row_2; ++j) {
      for (int k = col_1; k < col_2; ++k) {
        visibilityField_->Set(j, k, 0.0);
        speedField_->Set(j, k, speedValue_);
      }
    }
  }

  if (!config_->silent) {
    std::cout << "########################### Environment output ############################ \n" 
      << "Generated new environment based on parsed settings at a seed value of: " << seedValue << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void environment::generateNewEnvironment(size_t nrows, size_t ncols, int nb_of_obstacles, 
  int min_width, int max_width, int min_height, int max_height, int seedValue) {
  nrows_ = nrows; ncols_ = ncols;
  // Resets environment
  visibilityField_.reset(new FloatField(nrows_, ncols_, 1.0));
    speedField_.reset(new FloatField(nrows_, ncols_, 1.0));
  
  std::srand(seedValue);

  for(int i = 0; i < nb_of_obstacles; ++i) {

    int row_1 = 1 + (std::rand() % (nrows_ - 0 + 1));
    int row_2 = row_1 + min_width + (std::rand() % (max_width - min_width + 1));
    row_1 = std::min(row_1, (int)nrows_ - 1); row_2 = std::min(row_2,(int) nrows_ - 1); 

    int col_1 = 1 + (std::rand() % (ncols_ - 0 + 1));
    int col_2 = col_1 + min_height + (std::rand() % (max_height - min_height + 1));
    col_1 = std::min(col_1, (int)ncols_ - 1); col_2 = std::min(col_2, (int)ncols_ - 1); 

    for (int j = row_1; j < row_2; ++j) {
      for (int k = col_1; k < col_2; ++k) {
        visibilityField_->Set(j, k, 0.0);
        speedField_->Set(j, k, speedValue_);
      }
    }
  }
  if (!config_->silent) {
    std::cout << "########################### Environment output ############################ \n"
      << "Generated new environment on request based on custom settings" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void environment::loadImage(const std::string& filename) {
  loadedImage_.reset(std::make_unique<sf::Image>().release());
  // Load the image from a file
  if (!loadedImage_->loadFromFile(filename)) {
    std::cout << "Error: Failed to load image" << std::endl;
  } else {
    auto size = loadedImage_->getSize();
    nrows_ = size.x; ncols_ = size.y;
    // Resets environment
    visibilityField_.reset(new FloatField(nrows_, ncols_, 1.0));
    speedField_.reset(new FloatField(nrows_, ncols_, 1.0));
    sf::Color color;
    int gray;
    // Access the pixel data of the image
    for (size_t y = 0; y < ncols_; ++y) {
      for (size_t x = 0; x < nrows_; ++x) {
        color = loadedImage_->getPixel(x, y);
        gray = color.r;
        if (gray == 255) {
          visibilityField_->Set(x, y, 1.0);
          speedField_->Set(x, y, 1.0);
        } else {
          visibilityField_->Set(x, y, 0.0);
          speedField_->Set(x, y, 2.0);
        }
      }
    }
    std::cout << "Loaded image of dimensions " << nrows_ << "x" << ncols_ << " successfully" << std::endl;
  }
}

} // namespace vbs