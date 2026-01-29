import numpy as np
from PIL import Image
import random
import os
import time # For seed generation

from vbs_parser import VBSConfig
from vbs_field import Field

class Environment:
    def __init__(self, config: VBSConfig):
        self.config_ = config
        self.nx_ = 0
        self.ny_ = 0
        self.size_ = 0
        self.speed_value_ = 2.0  # Default from C++ environment.cpp
        self.seed_value_ = 1     # Default from C++

        self.shared_visibility_field_ = Field()
        self.shared_speed_field_ = Field()
        # self.unique_loaded_image_  # Not directly storing PIL image, data goes to fields

        if self.config_.mode == 1: # Generate random environment
            self.nx_ = self.config_.ncols
            self.ny_ = self.config_.nrows
            self.size_ = self.nx_ * self.ny_

            if not self.config_.random_seed:
                self.seed_value_ = self.config_.seed_value

            self.generate_new_environment_from_settings()
            if self.config_.save_results and self.config_.save_visibility_field: # C++ only saves if saveVisibilityField is true
                self.save_environment()

        elif self.config_.mode == 2: # Load image
            self.load_image(self.config_.image_path) # nx_, ny_ set in load_image
            if self.config_.save_results and self.config_.save_visibility_field:
                self.save_environment()
        else:
            print(f"Warning: Environment initialized with invalid mode: {self.config_.mode}")


    def _reset_environment_fields(self):
        """Helper to initialize or reset field objects."""
        # Visibility field: 1.0 means free, 0.0 means obstacle
        self.shared_visibility_field_.resize(self.nx_, self.ny_, 1.0)
        # Speed field: 1.0 in free space, speed_value_ in obstacle space (though typically not used there)
        self.shared_speed_field_.resize(self.nx_, self.ny_, 1.0)


    def generate_new_environment_from_settings(self):
        self._reset_environment_fields() # Initializes fields with default values

        current_seed = 0
        if not self.config_.random_seed:
            current_seed = self.seed_value_
        else:
            # Ensure seed is within the valid range for np.random.seed and random.seed
            current_seed = int(time.time() * 1_000_000) % (2**32) # Using microseconds and modulo

        random.seed(current_seed)
        np.random.seed(current_seed)

        for _ in range(self.config_.nb_of_obstacles):
            # Generate obstacle dimensions and position
            # Note: C++ uses 1-based indexing for rand() % (nx_ - 0 + 1), then clamps.
            # Python's random.randint is inclusive [a,b].
            # We need to be careful with 0-based vs 1-based and ranges.
            # C++ `col_1 = 1 + (std::rand() % (nx_ - 0 + 1));`
            # `nx_` in C++ is number of columns. If nx_ = 100, rand gives 0-99. So col_1 is 1 to 100.
            # Let's stick to 0-indexed coordinates for Python/NumPy.

            # Max possible start col/row such that min_width/height can still fit.
            # Ensure col_1 < nx_ and row_1 < ny_
            if self.nx_ <=1 or self.ny_ <=1: continue # Avoid issues with tiny grids

            col_1 = random.randint(0, self.nx_ - 2 if self.nx_ > 1 else 0)
            row_1 = random.randint(0, self.ny_ - 2 if self.ny_ > 1 else 0)

            # Obstacle width/height
            obs_w = self.config_.min_width + random.randint(0, self.config_.max_width - self.config_.min_width)
            obs_h = self.config_.min_height + random.randint(0, self.config_.max_height - self.config_.min_height)

            col_2 = min(col_1 + obs_w, self.nx_)
            row_2 = min(row_1 + obs_h, self.ny_)

            # Fill obstacle area
            # C++: sharedVisibilityField_->set(j, k, 0); sharedSpeedField_->set(j, k, speedValue_);
            # In Python Field (NumPy), this is field_data[k, j]
            for r in range(row_1, row_2): # k in C++
                for c in range(col_1, col_2): # j in C++
                    self.shared_visibility_field_.set_value(c, r, 0.0)
                    self.shared_speed_field_.set_value(c, r, self.speed_value_)

        if not self.config_.silent:
            print("########################### Environment output ############################")
            print(f"Generated new environment based on parsed settings at a seed value of: {current_seed}")

    def generate_new_environment(self, ncols: int, nrows: int, nb_of_obstacles: int,
                                 min_width: int, max_width: int, min_height: int,
                                 max_height: int, seed_value: int = 0):
        self.nx_ = ncols
        self.ny_ = nrows
        self.size_ = self.nx_ * self.ny_
        self._reset_environment_fields()

        random.seed(seed_value)
        np.random.seed(seed_value)

        # Similar logic to generate_new_environment_from_settings
        for _ in range(nb_of_obstacles):
            if self.nx_ <=1 or self.ny_ <=1: continue

            col_1 = random.randint(0, self.nx_ - 2 if self.nx_ > 1 else 0)
            row_1 = random.randint(0, self.ny_ - 2 if self.ny_ > 1 else 0)

            obs_w = min_width + random.randint(0, max_width - min_width)
            obs_h = min_height + random.randint(0, max_height - min_height)

            col_2 = min(col_1 + obs_w, self.nx_)
            row_2 = min(row_1 + obs_h, self.ny_)

            for r in range(row_1, row_2):
                for c in range(col_1, col_2):
                    self.shared_visibility_field_.set_value(c, r, 0.0)
                    self.shared_speed_field_.set_value(c, r, self.speed_value_)

        if not self.config_.silent:
            print("########################### Environment output ############################")
            print("Generated new environment on request based on custom settings")

        if self.config_.save_results and self.config_.save_visibility_field:
            self.save_environment()


    def load_image(self, filename: str):
        try:
            img = Image.open(filename).convert('L') # Convert to grayscale
            self.nx_, self.ny_ = img.size # PIL size is (width, height) -> (nx, ny)
            self.size_ = self.nx_ * self.ny_
            self._reset_environment_fields()

            pixels = np.array(img) # Creates a ny x nx array

            for r in range(self.ny_): # y in C++
                for c in range(self.nx_): # x in C++
                    gray_value = pixels[r, c]
                    if gray_value == 255: # White pixel = free space
                        self.shared_visibility_field_.set_value(c, r, 1.0)
                        self.shared_speed_field_.set_value(c, r, 1.0)
                    else: # Black/other pixel = obstacle
                        self.shared_visibility_field_.set_value(c, r, 0.0)
                        self.shared_speed_field_.set_value(c, r, self.speed_value_)

            if not self.config_.silent:
                print(f"Loaded image '{filename}' of dimensions {self.nx_}x{self.ny_} successfully")

        except FileNotFoundError:
            print(f"Error: Failed to load image '{filename}'. File not found.")
            # Potentially set nx, ny to 0 or raise error, C++ version prints error and continues
            self.nx_, self.ny_, self.size_ = 0,0,0
            self._reset_environment_fields() # create empty fields
        except Exception as e:
            print(f"Error: Failed to load image '{filename}'. Exception: {e}")
            self.nx_, self.ny_, self.size_ = 0,0,0
            self._reset_environment_fields()


    def save_environment(self, output_dir: str = "./output_py"):
        if not (self.config_.save_results and self.config_.save_visibility_field):
            return

        output_filepath = os.path.join(output_dir, "visibilityField.txt")
        os.makedirs(output_dir, exist_ok=True)

        try:
            with open(output_filepath, "w") as f:
                # C++ saves rows in reverse order (ny-1 down to 0)
                for r_idx in range(self.ny_ - 1, -1, -1): # r_idx is actual row index in numpy array
                    row_values = [str(self.shared_visibility_field_.get_value(c_idx, r_idx)) for c_idx in range(self.nx_)]
                    f.write(" ".join(row_values) + " \n") # C++ adds trailing space before newline

            if not self.config_.silent:
                print(f"Saved visibility field to {output_filepath}")
        except Exception as e:
            print(f"Error: Failed to save environment to {output_filepath}. Exception: {e}")

    @property
    def visibility_field(self) -> Field:
        return self.shared_visibility_field_

    @property
    def speed_field(self) -> Field:
        return self.shared_speed_field_

    @property
    def config(self) -> VBSConfig:
        return self.config_

    @property
    def nx(self) -> int:
        return self.nx_

    @property
    def ny(self) -> int:
        return self.ny_


if __name__ == '__main__':
    print("Testing Environment class...")

    # Create a dummy config for mode 1 (random generation)
    cfg_mode1 = VBSConfig(
        mode=1, ncols=20, nrows=15, nb_of_obstacles=5,
        min_width=2, max_width=4, min_height=2, max_height=4,
        random_seed=False, seed_value=123, silent=False,
        save_results=True, save_visibility_field=True
    )
    env1 = Environment(cfg_mode1)
    print(f"Env1 (random): nx={env1.nx}, ny={env1.ny}")
    if env1.nx > 0 and env1.ny > 0 :
        print("Visibility field sample (0,0):", env1.visibility_field.get_value(0,0))
        # Check if output file was created
        assert os.path.exists("./output_py/visibilityField.txt")
        print("Mode 1 test: visibilityField.txt created.")
        # Basic check on content (e.g. not empty, some 0s and 1s)
        with open("./output_py/visibilityField.txt", "r") as f:
            content = f.read()
            assert "0.0" in content and "1.0" in content
            print("Mode 1 test: visibilityField.txt seems to contain data.")


    # Create a dummy config for mode 2 (image loading)
    # First, create a dummy image
    os.makedirs("./images_py_test", exist_ok=True)
    dummy_image_path = "./images_py_test/test_maze.png"
    try:
        array = np.zeros((10, 20), dtype=np.uint8)  # ny=10, nx=20
        array[1:3, 1:5] = 255  # A white rectangle (free space)
        array[5:8, 10:15] = 255 # Another white rectangle
        img = Image.fromarray(array, 'L')
        img.save(dummy_image_path)
        print(f"Created dummy image {dummy_image_path}")
    except Exception as e:
        print(f"Could not create dummy image: {e}")


    cfg_mode2 = VBSConfig(
        mode=2, image_path=dummy_image_path, silent=False,
        save_results=True, save_visibility_field=True
    )
    env2 = Environment(cfg_mode2)
    print(f"Env2 (image): nx={env2.nx}, ny={env2.ny}")
    if env2.nx > 0 and env2.ny > 0:
        assert env2.nx == 20
        assert env2.ny == 10
        # Check a known obstacle point (should be 0.0) and a known free point (should be 1.0)
        assert env2.visibility_field.get_value(0,0) == 0.0
        assert env2.visibility_field.get_value(1,1) == 1.0
        assert env2.speed_field.get_value(0,0) == env2.speed_value_
        assert env2.speed_field.get_value(1,1) == 1.0
        print("Mode 2 test: Loaded image and checked some pixel values.")
        assert os.path.exists("./output_py/visibilityField.txt") # Should be overwritten
        print("Mode 2 test: visibilityField.txt overwritten/created.")


    # Test generate_new_environment directly
    cfg_custom_gen = VBSConfig(silent=False, save_results=False) # Don't save this one by default
    env3 = Environment(cfg_custom_gen) # Creates a default env first
    env3.generate_new_environment(
        ncols=8, nrows=6, nb_of_obstacles=2,
        min_width=1, max_width=2, min_height=1, max_height=2,
        seed_value=42
    )
    print(f"Env3 (custom gen): nx={env3.nx}, ny={env3.ny}")
    assert env3.nx == 8 and env3.ny == 6
    print("Custom generation test passed.")

    # Test loading a non-existent image
    cfg_bad_image = VBSConfig(mode=2, image_path="non_existent_image.png", silent=False)
    env_bad = Environment(cfg_bad_image)
    assert env_bad.nx == 0 and env_bad.ny == 0 # Should handle error gracefully

    # Clean up
    if os.path.exists(dummy_image_path):
        os.remove(dummy_image_path)
    if os.path.exists("./images_py_test"):
        os.rmdir("./images_py_test")
    if os.path.exists("./output_py/visibilityField.txt"):
        os.remove("./output_py/visibilityField.txt")
    if os.path.exists("./output_py"):
        # Check if empty before removing, to avoid error if other files exist
        if not os.listdir("./output_py"):
            os.rmdir("./output_py")
        else:
            print("Note: ./output_py not empty, not removing directory.")


    print("Environment class tests seem to have passed.")
