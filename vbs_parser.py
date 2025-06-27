import configparser
from dataclasses import dataclass, field
from typing import Tuple

@dataclass
class VBSConfig:
    mode: int = 1
    ncols: int = 100
    nrows: int = 100
    nb_of_obstacles: int = 10
    min_width: int = 10
    max_width: int = 20
    min_height: int = 10
    max_height: int = 20
    random_seed: bool = True
    seed_value: int = 0
    image_path: str = "C:\\..."
    start: Tuple[int, int] = (0,0)
    end: Tuple[int, int] = (0,0)
    max_iter: int = 100
    visibility_threshold: float = 0.5
    light_strength: float = 1.0
    timer: bool = True
    save_results: bool = True
    save_local_visibility: bool = True
    save_came_from: bool = True
    save_light_sources: bool = True
    save_global_visibility: bool = True
    save_visibility_field: bool = True
    silent: bool = False
    ball_radius: int = 5

    # Field for attributes that need custom parsing or have different names
    # in the config file vs. class attributes.
    # For example, if config file has 'start_point' and class has 'start'
    _config_map: dict = field(default_factory=lambda: {
        "minWidth": "min_width",
        "maxWidth": "max_width",
        "minHeight": "min_height",
        "maxHeight": "max_height",
        "randomSeed": "random_seed",
        "seedValue": "seed_value",
        "imagePath": "image_path",
        "max_iter": "max_iter", # Corrected, was max_iter_
        "visibilityThreshold": "visibility_threshold",
        "lightStrength": "light_strength",
        "saveLocalVisibility": "save_local_visibility",
        "saveCameFrom": "save_came_from",
        "saveLightSources": "save_light_sources",
        "saveGlobalVisibility": "save_global_visibility",
        "saveVisibilityField": "save_visibility_field",
        "ballRadius": "ball_radius"
    })


def parse_pair_string(s: str) -> Tuple[int, int]:
    """Parses a string like '{x,y}' into a tuple (x, y)."""
    try:
        s = s.strip('{}')
        parts = s.split(',')
        if len(parts) == 2:
            return int(parts[0].strip()), int(parts[1].strip())
        else:
            # print(f"Warning: Could not parse pair string correctly: {s}. Using (0,0).")
            return 0, 0
    except ValueError:
        # print(f"Warning: ValueError during parsing pair string: {s}. Using (0,0).")
        return 0,0

def parse_config(filename: str) -> VBSConfig:
    """Parses the configuration file and returns a VBSConfig object."""
    parser = configparser.ConfigParser()
    # Add a dummy section header if the config file doesn't have one
    try:
        with open(filename, 'r') as f:
            content = f.read()
        if not content.strip().startswith('['): # Basic check for section
            content = "[DEFAULT]\n" + content

        # Use StringIO to read the modified content
        from io import StringIO
        file_obj = StringIO(content)
        parser.read_file(file_obj)

    except FileNotFoundError:
        print(f"Error: Configuration file '{filename}' not found.")
        return VBSConfig() # Return default config
    except Exception as e:
        print(f"Error reading config file '{filename}': {e}")
        return VBSConfig()


    config_data = {}
    # Assuming all settings are in a 'DEFAULT' section if not specified,
    # or the first section if specified.
    section = parser.sections()[0] if parser.sections() else 'DEFAULT'

    if section not in parser:
        if 'DEFAULT' in parser:
            section = 'DEFAULT'
        else:
            print(f"Error: No section found in config file that can be parsed.")
            return VBSConfig()


    cfg = VBSConfig()

    for key, value in parser.items(section):
        # Map from config file key to VBSConfig attribute name if necessary
        attr_name = cfg._config_map.get(key, key)

        if hasattr(cfg, attr_name):
            current_type = type(getattr(cfg, attr_name))
            try:
                if current_type == bool:
                    if value.lower() in ['true', '1']:
                        config_data[attr_name] = True
                    elif value.lower() in ['false', '0']:
                        config_data[attr_name] = False
                    else:
                        print(f"Warning: Invalid boolean value for {attr_name}: {value}. Using default.")
                elif attr_name in ["start", "end"]:
                    config_data[attr_name] = parse_pair_string(value)
                elif current_type == tuple and isinstance(getattr(cfg, attr_name)[0], int): # For tuples of ints like start/end
                     config_data[attr_name] = parse_pair_string(value)
                elif current_type == int:
                    val = int(value)
                    # Add C++ parser's checks for non-negative integers
                    if key in ["ncols", "nrows", "minWidth", "maxWidth", "minHeight", "maxHeight", "max_iter"]:
                        if val < 0:
                            print(f"Warning: Invalid negative value for {attr_name}: {val}. Using default or previously set value.")
                            continue # Skip this update
                    config_data[attr_name] = val
                elif current_type == float:
                    f_val = float(value)
                    if key in ["visibilityThreshold", "lightStrength"]:
                         if not (0.0 <= f_val <= 1.0):
                            print(f"Warning: Value for {attr_name} ({f_val}) out of range [0,1]. Using default.")
                            continue
                    config_data[attr_name] = f_val
                else: # primarily for strings like image_path
                    config_data[attr_name] = value
            except ValueError:
                print(f"Warning: Could not convert value '{value}' for {attr_name} to {current_type}. Using default.")
        # else:
            # print(f"Warning: Unknown configuration key '{key}' (maps to '{attr_name}'). Ignoring.")

    # Update VBSConfig instance with parsed data
    for key, val in config_data.items():
        setattr(cfg, key, val)

    # C++ specific checks / post-processing
    if not (1 <= cfg.mode <= 2):
        print(f"Warning: Invalid mode {cfg.mode}. Defaulting to 1.")
        cfg.mode = 1

    if not cfg.silent:
        print("################## Parsing results: ###################")
        if cfg.mode == 1:
            print("Random environment mode")
            print("################### Environment settings ##################")
            print(f"nrows: {cfg.nrows}")
            print(f"ncols: {cfg.ncols}")
            print(f"Nb of obstacles: {cfg.nb_of_obstacles}")
            print(f"Min width: {cfg.min_width}")
            print(f"Max width: {cfg.max_width}")
            print(f"Min height: {cfg.min_height}")
            print(f"Max height: {cfg.max_height}")
            if cfg.random_seed:
                print(f"Random seed: {cfg.random_seed}")
            else:
                print(f"Fixed seed value: {cfg.seed_value}")
        elif cfg.mode == 2:
            print("Import image mode")
            print(f"Image path: {cfg.image_path}")

        print("#################### Solver settings ######################")
        print(f"Start point: {cfg.start[0]}, {cfg.start[1]}")
        print(f"End point: {cfg.end[0]}, {cfg.end[1]}")
        print(f"Maximum iterations: {cfg.max_iter}")
        print(f"Solver visibility threshold: {cfg.visibility_threshold}")
        print(f"Light strength: {cfg.light_strength}")

        print("#################### Output settings ######################")
        print(f"timer: {cfg.timer}")
        print(f"saveCameFrom: {cfg.save_came_from}")
        print(f"saveLightSources: {cfg.save_light_sources}")
        print(f"saveGlobalVisibility: {cfg.save_global_visibility}")
        print(f"saveLocalVisibility: {cfg.save_local_visibility}")
        print(f"saveVisibilityField: {cfg.save_visibility_field}")
        print(f"silent: {cfg.silent}")
        print(f"ballRadius: {cfg.ball_radius}\n")


    return cfg

if __name__ == '__main__':
    # Create a dummy config file for testing
    dummy_config_content = """\
# Example settings
mode = 1
ncols = 150
nrows = 150
nb_of_obstacles = 20
minWidth = 5
maxWidth = 15
minHeight = 5
maxHeight = 15
randomSeed = true
# seedValue = 12345
imagePath = "./images/maze_1.png"
start = {10,10}
end = {140,140}
max_iter = 200
visibilityThreshold = 0.6
lightStrength = 0.9
timer = true
saveResults = true
saveLocalVisibility = false
saveCameFrom = true
saveLightSources = true
saveGlobalVisibility = false
saveVisibilityField = true
silent = false
ballRadius = 3
"""
    dummy_config_path = "dummy_settings.config"
    with open(dummy_config_path, "w") as f:
        f.write(dummy_config_content)

    print("Testing with dummy_settings.config:")
    config = parse_config(dummy_config_path)
    # print(config) # Full dataclass print

    print("\nTesting with original config/settings.config (if it exists):")
    # Attempt to parse the original config file if it exists
    original_config_path = "config/settings.config"
    try:
        config_orig = parse_config(original_config_path)
        # print(config_orig) # Full dataclass print
    except Exception as e:
        print(f"Could not test with {original_config_path}: {e}")

    # Test a non-existent file
    print("\nTesting with non_existent_settings.config:")
    config_non_existent = parse_config("non_existent_settings.config")
    # print(config_non_existent) # Should show defaults and error message

    # Test a config file with a parse error for a value
    error_config_content = """\
mode = not_an_int
ncols = 100
"""
    error_config_path = "error_settings.config"
    with open(error_config_path, "w") as f:
        f.write(error_config_content)
    print("\nTesting with error_settings.config (invalid value):")
    config_error = parse_config(error_config_path)
    # print(config_error)

    # Test a config file with a bad pair string
    bad_pair_config_content = """\
start = {10,nan}
end = {1,2,3}
"""
    bad_pair_config_path = "bad_pair_settings.config"
    with open(bad_pair_config_path, "w") as f:
        f.write(bad_pair_config_content)
    print("\nTesting with bad_pair_settings.config (bad pair string):")
    config_bad_pair = parse_config(bad_pair_config_path)
    # print(config_bad_pair)

    # Test negative value for restricted int
    negative_value_config = """\
ncols = -50
"""
    negative_value_config_path = "negative_value.config"
    with open(negative_value_config_path, "w") as f:
        f.write(negative_value_config)
    print("\nTesting with negative_value.config:")
    config_neg_val = parse_config(negative_value_config_path)

    # Test out of range float
    out_of_range_float_config = """\
visibilityThreshold = 1.5
"""
    out_of_range_float_path = "out_of_range_float.config"
    with open(out_of_range_float_path, "w") as f:
        f.write(out_of_range_float_config)
    print("\nTesting with out_of_range_float.config:")
    config_oor_float = parse_config(out_of_range_float_path)

    print("\nDone with parser tests.")

    # Clean up dummy files
    import os
    os.remove(dummy_config_path)
    os.remove(error_config_path)
    os.remove(bad_pair_config_path)
    os.remove(negative_value_config_path)
    os.remove(out_of_range_float_path)

"""
C++ Config struct:
struct Config {
  int mode = 1;
  size_t ncols = 100;
  size_t nrows = 100;
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
  double visibilityThreshold = 0.5;
  float lightStrength = 1; // Note: C++ float, Python float is double precision
  bool timer = true;
  bool saveResults = true;
  bool saveLocalVisibility = true;
  bool saveCameFrom = true;
  bool saveLightSources = true;
  bool saveGlobalVisibility = true;
  bool saveVisibilityField = true;
  bool silent = false;
  int ballRadius = 5;
};
"""
