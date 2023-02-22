#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <string>
#include <vector>

namespace vbs {

using size_t = std::size_t;
using point = std::pair<int, int>;

struct Config {
  bool randomEnvironment = true;
  bool importImage = false;
  size_t nrows = 100;
  size_t ncols = 100;
  size_t nb_of_obstacles = 10;
  size_t minWidth = 10;
  size_t maxWidth = 20;
  size_t minHeight = 10;
  size_t maxHeight = 20;
  bool randomSeed = true;
  int seedValue = 0;
  std::string imagePath = "C:\\...";
  point start;
  point end;
  size_t max_iter = 100;
  float threshold = 0.5;
  float lightStrength = 1;
  bool timer = true;
  bool saveLocalVisibility = true;
  bool saveLightSourceEnum = true;
  bool saveLightSources = true;
  bool saveGlobalVisibility = true;
  bool saveVisibilityMapEnv = true;
  bool silent = false;
};

class ConfigParser {
public:
  ConfigParser(){};
  bool parse(const std::string& filename);
  inline const Config& getConfig() const { return config_; };
  // Deconstructor
  ~ConfigParser() = default;

private:
    Config config_;
    point parsePairString(const std::string& str);
};

} // namespace vbs
#endif // CONFIG_PARSER_H