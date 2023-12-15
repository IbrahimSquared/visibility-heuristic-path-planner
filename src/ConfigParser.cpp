#include "parser/ConfigParser.h"

#include <fstream>
#include <sstream>
#include <iostream>

namespace vbs {

bool ConfigParser::parse(const std::string& filename) {
  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Failed to open " << filename << '\n';
    return false;
  }
  
  std::string line;
  while (std::getline(file, line)) {
    // Ignore comments and blank lines
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // Split the line into key and value
    std::istringstream iss(line);
    std::string key;
    if (!std::getline(iss, key, '=')) {
      continue;
    }
    std::string value;
    if (!std::getline(iss, value)) {
      continue;
    }

    // Trim leading and trailing whitespace from key and value
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_last_not_of(" \t") + 1);

    // Parse the value based on the key's data type
    if (key == "mode") {
      try {
        config_.mode = std::stoi(value);
        if (config_.mode != 1 && config_.mode != 2) {
          std::cerr << "Invalid value for " << key << ": " << value
                    << ", using default value 1\n";
          config_.mode = 1;
        }
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer 1 or 2 \n";
        return false;
      }
    } else if (key == "ncols") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.ncols = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "nrows") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.nrows = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "nb_of_obstacles") {
      try {
        config_.nb_of_obstacles = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "minWidth") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.minWidth = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "maxWidth") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.maxWidth = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "minHeight") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.minHeight = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "maxHeight") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.maxHeight = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "randomSeed") {
      if (value == "0" || value == "false") {
        config_.randomSeed = false;
      } else if (value == "1" || value == "true") {
        config_.randomSeed = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "seedValue") {
      try {
        config_.seedValue = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "imagePath") {
      config_.imagePath = value;
    } else if (key == "saveLocalVisibility") {
      if (value == "0" || value == "false") {
        config_.saveLocalVisibility = false;
      } else if (value == "1" || value == "true") {
        config_.saveLocalVisibility = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "start") {
      config_.start = parsePairString(value);
    } else if (key == "end") {
      config_.end = parsePairString(value);
    } else if (key == "max_iter") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.max_iter = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "visibilityThreshold") {
      try {
        config_.visibilityThreshold = std::stod(value);
        if (config_.visibilityThreshold > 1.0 || config_.visibilityThreshold < 0.0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a double between 0 and 1\n";
          return false;
        }
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive double between 0 and 1\n";
        return false;
      }
    } else if (key == "lightStrength") {
      try {
        config_.lightStrength = std::stod(value);
        if (config_.lightStrength > 1.0 || config_.lightStrength < 0.0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a double between 0 and 1\n";
          return false;
        }
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive double between 0 and 1\n";
        return false;
      }
    } else if (key == "timer") {
      if (value == "0" || value == "false") {
        config_.timer = false;
      } else if (value == "1" || value == "true") {
        config_.timer = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveResults") {
      if (value == "0" || value == "false") {
        config_.saveResults = false;
      } else if (value == "1" || value == "true") {
        config_.saveResults = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveCameFrom") {
      if (value == "0" || value == "false") {
        config_.saveCameFrom = false;
      } else if (value == "1" || value == "true") {
        config_.saveCameFrom = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveLightSources") {
      if (value == "0" || value == "false") {
        config_.saveLightSources = false;
      } else if (value == "1" || value == "true") {
        config_.saveLightSources = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveGlobalVisibility") {
      if (value == "0" || value == "false") {
        config_.saveGlobalVisibility = false;
      } else if (value == "1" || value == "true") {
        config_.saveGlobalVisibility = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveVisibilityField") {
      if (value == "0" || value == "false") {
        config_.saveVisibilityField = false;
      } else if (value == "1" || value == "true") {
        config_.saveVisibilityField = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "silent") {
      if (value == "0" || value == "false") {
        config_.silent = false;
      } else if (value == "1" || value == "true") {
        config_.silent = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "ballRadius") {
      try {
        config_.ballRadius = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else {
      std::cerr << "Invalid/irrelavent key: " << key << '\n';
    }
  }
  if (!config_.silent) {
    if (config_.mode == 1) {
      std::cout << "Random environment mode" << std::endl;
      std::cout << "########################### Environment settings ########################## \n"
        << "nrows: " << config_.nrows << "\n"
        << "ncols: " << config_.ncols << "\n"
        << "Nb of obstacles: " << config_.nb_of_obstacles << "\n"
        << "Min width: " << config_.minWidth << "\n"
        << "Max width: " << config_.maxWidth << "\n"
        << "Min height: " << config_.minHeight << "\n"
        << "Max height: " << config_.maxHeight << std::endl;
      if (config_.randomSeed) {
        std::cout << "Random seed: " << config_.randomSeed << std::endl;
      } else {
        std::cout << "Fixed seed value: " << config_.seedValue << std::endl;
      }
    } else if (config_.mode == 2) {
      std::cout << "Import image mode" << "\n"
        << "Image path: " << config_.imagePath << std::endl;
    }
    std::cout << "############################ Solver settings ############################## \n"
        << "Start point: " << config_.start.first << ", " << config_.start.second << "\n"
        << "End point: " << config_.end.first << ", " << config_.end.second << "\n"
        << "Maximum iterations: " << config_.max_iter << "\n"
        << "Solver visibility threshold: " << config_.visibilityThreshold << "\n"
        << "Light strength: " << config_.lightStrength << std::endl;
    std::cout << "############################ Output settings ############################## \n"
      << "timer: " << config_.timer << "\n"
      << "saveLightSourceEnum: " << config_.saveCameFrom << "\n"
      << "saveLightSources: " << config_.saveLightSources << "\n"
      << "saveVisibilityField: " << config_.saveGlobalVisibility << "\n"
      << "saveLocalVisibility: " << config_.saveLocalVisibility << "\n"
      << "saveVisibilityMapEnv: " << config_.saveVisibilityField << std::endl;
  }
  // Return true if we successfully parsed the config file
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
point ConfigParser::parsePairString(const std::string& str) {
  point result;
  int n = sscanf(str.c_str(), "{%d,%d}", &result.first, &result.second);
  if (n != 2) {
    std::cerr << "Error: Invalid pair string: " << str << std::endl;
    // Set default values
    result.first = 0;
    result.second = 0;
  }
  return result;
}

} // namespace vbs