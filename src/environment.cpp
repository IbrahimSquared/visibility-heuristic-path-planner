#include "environment/environment.h"

#include <algorithm>
#include <ctime>

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

namespace vbs {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
environment::environment(Config& config) : sharedConfig_(std::make_shared<Config>(config)) {
  if (sharedConfig_->mode == 1) {
    nx_ = sharedConfig_->ncols;
    ny_ = sharedConfig_->nrows;
    
    if (!sharedConfig_->randomSeed) {
      seedValue_ = sharedConfig_->seedValue;
    }
    generateNewEnvironmentFromSettings();
    if (sharedConfig_->saveResults) {
      saveEnvironment();
    }
  } else if (sharedConfig_->mode == 2) {
    // loadMaps(sharedConfig_->imagePath);
    loadImage(sharedConfig_->imagePath);
    if (sharedConfig_->saveResults) {
      saveEnvironment();
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void environment::generateNewEnvironmentFromSettings() {
  resetEnvironment();
  int seedValue = 0;
  if (!sharedConfig_->randomSeed) {
    seedValue = seedValue_;
  } else {
    // Get the current time point using the high resolution clock
    auto time_point = std::chrono::high_resolution_clock::now();
    // Convert the time point to nanoseconds since the epoch
    auto ns_since_epoch = std::chrono::time_point_cast<std::chrono::nanoseconds>(time_point).time_since_epoch().count();
    seedValue = ns_since_epoch;
  }
  std::srand(seedValue);
  
  for(int i = 0; i < sharedConfig_->nb_of_obstacles; ++i) {

    int col_1 = 1 + (std::rand() % (nx_ - 0 + 1));
    int col_2 = col_1 + sharedConfig_->minWidth + (std::rand() % (sharedConfig_->maxWidth - sharedConfig_->minWidth + 1));
    col_1 = std::min(col_1, (int)nx_ - 1); col_2 = std::min(col_2, (int)nx_ - 1); 

    int row_1 = 1 + (std::rand() % (ny_ - 0 + 1));
    int row_2 = row_1 + sharedConfig_->minHeight + (std::rand() % (sharedConfig_->maxHeight - sharedConfig_->minHeight + 1));
    row_1 = std::min(row_1, (int)ny_ - 1); row_2 = std::min(row_2,(int) ny_ - 1); 

    for (int j = col_1; j < col_2; ++j) {
      for (int k = row_1; k < row_2; ++k) {
        sharedVisibilityField_->set(j, k, 0);
        sharedSpeedField_->set(j, k, speedValue_);
      }
    }
  }

  if (!sharedConfig_->silent) {
    std::cout << "########################### Environment output ############################ \n" 
      << "Generated new environment based on parsed settings at a seed value of: " << seedValue << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void environment::generateNewEnvironment(size_t ncols, size_t nrows, int nb_of_obstacles, 
  int min_width, int max_width, int min_height, int max_height, int seedValue) {
  nx_ = ncols;
  ny_ = nrows;
  resetEnvironment();
  
  std::srand(seedValue);

  for(int i = 0; i < nb_of_obstacles; ++i) {

    int col_1 = 1 + (std::rand() % (nx_ - 0 + 1));
    int col_2 = col_1 + min_width + (std::rand() % (max_width - min_width + 1));
    col_1 = std::min(col_1, (int)nx_ - 1); col_2 = std::min(col_2, (int)nx_ - 1); 

    int row_1 = 1 + (std::rand() % (ny_ - 0 + 1));
    int row_2 = row_1 + min_height + (std::rand() % (max_height - min_height + 1));
    row_1 = std::min(row_1, (int)ny_ - 1); row_2 = std::min(row_2,(int) ny_ - 1); 

    for (int j = col_1; j < col_2; ++j) {
      for (int k = row_1; k < row_2; ++k) {
        sharedVisibilityField_->set(j, k, 0);
        sharedSpeedField_->set(j, k, speedValue_);
      }
    }
  }
  if (!sharedConfig_->silent) {
    std::cout << "########################### Environment output ############################ \n"
      << "Generated new environment on request based on custom settings" << std::endl;
  }
  if (sharedConfig_->saveResults) {
    saveEnvironment();
  }
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void environment::loadMaps(const std::string& filename) {
  std::ifstream input(filename);
  std::vector<std::vector<float>> visibilityField;
  for (std::string line; std::getline(input, line); ) {
    std::vector<float> floatVector = stringToFloatVector(line, ' ');
    visibilityField.push_back(floatVector);
  }
  nx_ = visibilityField.size();
  ny_ = visibilityField[0].size();
  std::cout << "Loaded vector of dimensions " << nx_ << "x" << ny_ << " successfully" << std::endl;

  // Resets environment
  resetEnvironment();
  
  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      sharedVisibilityField_->set(i, j, visibilityField[i][j]);
      if (visibilityField[i][j] == 1.0) {
        sharedSpeedField_->set(i, j, 1.0);
      } else {
        sharedSpeedField_->set(i, j, speedValue_);
      }
    }
  }
  std::cout << "Loaded image of dimensions " << nx_ << "x" << ny_ << " successfully" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<float> environment::stringToFloatVector(const std::string& str, char delimiter) {
  std::vector<float> floatVector;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    floatVector.push_back(std::stof(item));
  }
  return floatVector;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void environment::loadImage(const std::string& filename) {
  uniqueLoadedImage_.reset(std::make_unique<sf::Image>().release());
  // Load the image from a file
  if (!uniqueLoadedImage_->loadFromFile(filename)) {
    std::cout << "Error: Failed to load image" << std::endl;
  } else {
    auto size = uniqueLoadedImage_->getSize();
    nx_ = size.x; ny_ = size.y;

    resetEnvironment();

    sf::Color color;
    int gray;
    // Access the pixel data of the image
    for (size_t x = 0; x < nx_; ++x) {
      for (size_t y = 0; y < ny_; ++y) {
        color = uniqueLoadedImage_->getPixel(x, y);
        gray = color.r;
        if (gray == 255) {
          sharedVisibilityField_->set(x, y, 1.0);
          sharedSpeedField_->set(x, y, 1.0);
        } else {
          sharedVisibilityField_->set(x, y, 0);
          sharedSpeedField_->set(x, y, speedValue_);
        }
      }
    }
    std::cout << "Loaded image of dimensions " << nx_ << "x" << ny_ << " successfully" << std::endl;
  }
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void environment::resetEnvironment() {
  sharedVisibilityField_ = std::make_shared<Field<double>>(nx_, ny_, 1.0);
  sharedSpeedField_ = std::make_shared<Field<double>>(nx_, ny_, 1.0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void environment::saveEnvironment() {
  // Define the path to the output file
  std::string outputFilePath = "./output/visibilityField.txt";
  // Check if the directory exists, and create it if it doesn't
  namespace fs = std::filesystem;
  fs::path directory = fs::path(outputFilePath).parent_path();
  if (!fs::exists(directory)) {
    if (!fs::create_directories(directory)) {
      std::cerr << "Failed to create directory " << directory.string() << std::endl;
      return;
    }
  }

  // save visibility field
  if (sharedConfig_->saveVisibilityField) {
    std::fstream  of(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream& os = of;
    for (int j = ny_-1; j >= 0; --j) {
      for (size_t i = 0; i < nx_; ++i) {
        os << sharedVisibilityField_->get(i, j) << " "; 
      }
      os << "\n";
    }
    of.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved visibility field" << std::endl;
    }
  }
}

} // namespace vbs