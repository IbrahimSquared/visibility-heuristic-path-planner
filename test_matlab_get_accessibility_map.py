import numpy as np
import os
import scipy.io # For loading .mat files

# Assuming the main modules are in the same directory or accessible in PYTHONPATH
from matlab_visibility import get_accessibility_map_planner_py

def load_matlab_gamp_data(data_dir: str, mat_filename: str = "gamp_test_data.mat"):
    """
    Loads test data for getAccessibilityMapPlanner from a .mat file or individual .txt files.
    Returns a dictionary of the loaded data, or None if loading fails.

    Expected .mat file structure (keys are illustrative):
    - 'alpha_in', 'lightStrength_in', 'lightPos_in' (1x2 array, 1-based)
    - 'obstacle_in' (MxN matrix, 0=obs, 1=free)
    - 'accessibilityMap_in' (MxN matrix)
    - 'lightSource_enum_in' (MxN matrix, NaNs for unassigned)
    - 'iter_in', 'threshold_in'
    - 'accessibilityMap_out_expected' (MxN matrix)
    - 'lightSource_enum_out_expected' (MxN matrix)

    Expected .txt file structure (in data_dir):
    - alpha.txt, lightStrength.txt, iter.txt, threshold.txt (single numbers)
    - lightPos.txt (2 numbers, row_1based col_1based)
    - obstacle_map_input.txt (MxN matrix)
    - accessibility_map_input.txt (MxN matrix)
    - light_source_enum_input.txt (MxN matrix, use np.nan for NaNs)
    - accessibility_map_output_expected.txt (MxN matrix)
    - light_source_enum_output_expected.txt (MxN matrix)
    """
    data = {}
    mat_filepath = os.path.join(data_dir, mat_filename)

    if os.path.exists(mat_filepath):
        try:
            mat_data = scipy.io.loadmat(mat_filepath)
            print(f"Loaded data from {mat_filepath}")
            # Extract data, assuming specific key names used when saving from MATLAB
            data['alpha'] = float(mat_data['alpha_in'][0,0])
            data['light_strength'] = float(mat_data['lightStrength_in'][0,0])
            # MATLAB lightPos is 1-based [row, col]. Convert to 0-based (y,x) for Python.
            lp_mat = mat_data['lightPos_in'].flatten().astype(int)
            data['light_pos_py'] = (lp_mat[0] - 1, lp_mat[1] - 1)

            data['obstacle_map'] = mat_data['obstacle_in'].astype(float) # Ensure float for 0.0/1.0
            data['accessibility_map_input'] = mat_data['accessibilityMap_in'].astype(float)
            # Handle NaNs in enum map carefully
            data['light_source_enum_input'] = mat_data['lightSource_enum_in'].astype(float)

            data['iteration'] = int(mat_data['iter_in'][0,0])
            data['threshold'] = float(mat_data['threshold_in'][0,0])

            data['expected_accessibility_map_output'] = mat_data['accessibilityMap_out_expected'].astype(float)
            data['expected_light_source_enum_output'] = mat_data['lightSource_enum_out_expected'].astype(float)
            return data
        except Exception as e:
            print(f"Error loading .mat file {mat_filepath}: {e}")
            return None
    else:
        # Fallback to loading individual text files (more manual for user to create)
        print(f".mat file not found at {mat_filepath}. Attempting to load from individual .txt files in {data_dir} (not fully implemented yet).")
        # This part would need individual load_grid_from_txt etc.
        # For now, prioritize .mat file for simplicity from MATLAB side.
        print("Please provide data as a .mat file for this test script.")
        return None


def run_gamp_comparison_test():
    print("Running MATLAB getAccessibilityMapPlanner vs Python comparison test...")

    data_dir = "./matlab_gamp_test_data/"
    os.makedirs(data_dir, exist_ok=True)

    loaded_data = load_matlab_gamp_data(data_dir)

    if loaded_data is None:
        print("Test data could not be loaded. Please ensure 'gamp_test_data.mat' exists in "
              f"'{data_dir}' with all required variables.")
        print("Required .mat variables: alpha_in, lightStrength_in, lightPos_in, "
              "obstacle_in, accessibilityMap_in, lightSource_enum_in, iter_in, threshold_in, "
              "accessibilityMap_out_expected, lightSource_enum_out_expected.")
        return False

    # Prepare inputs for Python function
    # Python function expects obstacle_map: 1 for free, 0 for obstacle.
    # MATLAB script's 'obstacle' variable is usually 1 for obstacle, 0 for free.
    # The getAccessibilityMapPlanner call uses `1-obstacle`.
    # So, if 'obstacle_in' from .mat is (1=obs, 0=free), then use it directly for `1-obstacle_map_matlab`.
    # If 'obstacle_in' from .mat is already (0=obs, 1=free) as per python convention, use it directly.
    # The prompt specified "obstacle (MxN matrix, 0 for obstacle, 1 for free space)" for .mat file.
    # This matches Python's convention for obstacle_map.

    py_accessibility_map = np.copy(loaded_data['accessibility_map_input'])
    py_light_source_enum = np.copy(loaded_data['light_source_enum_input'])

    print(f"Input shapes: obstacle_map: {loaded_data['obstacle_map'].shape}, "
          f"accessibility_map_input: {py_accessibility_map.shape}, "
          f"light_source_enum_input: {py_light_source_enum.shape}")
    print(f"Input params: alpha={loaded_data['alpha']}, strength={loaded_data['light_strength']}, "
          f"light_pos_py={loaded_data['light_pos_py']}, iter={loaded_data['iteration']}, thresh={loaded_data['threshold']}")

    # Call the Python implementation
    returned_access_map, returned_enum_map = get_accessibility_map_planner_py(
        alpha=loaded_data['alpha'],
        light_strength=loaded_data['light_strength'],
        light_pos=loaded_data['light_pos_py'], # Already 0-based (y,x)
        obstacle_map=loaded_data['obstacle_map'], # Expects 1=free, 0=obs
        accessibility_map_py=py_accessibility_map, # Modified in-place
        light_source_enum_py=py_light_source_enum, # Modified in-place
        iteration=loaded_data['iteration'],
        threshold=loaded_data['threshold']
    )

    # The function modifies inputs in-place and also returns them.
    # So, py_accessibility_map IS returned_access_map.

    print("Comparing Python accessibility_map output with MATLAB expected output...")
    try:
        np.testing.assert_allclose(
            py_accessibility_map,
            loaded_data['expected_accessibility_map_output'],
            rtol=1e-5, atol=1e-7,
            err_msg="Mismatch between Python and MATLAB accessibilityMap outputs."
        )
        print("SUCCESS: Python accessibility_map output matches MATLAB output.")
    except AssertionError as e:
        print("FAILURE: accessibility_map outputs do NOT match.")
        print(e)
        # diff = np.abs(py_accessibility_map - loaded_data['expected_accessibility_map_output'])
        # print("Max difference in accessibility_map:", np.max(diff) if diff.size > 0 else "N/A (empty diff)")
        return False # Stop test if first map fails

    print("Comparing Python light_source_enum output with MATLAB expected output...")
    try:
        # For enum map, NaNs should match if used, otherwise direct comparison.
        # MATLAB NaNs become np.nan when loaded via scipy.io.loadmat.
        # Python function uses float('inf') or np.nan as unassigned.
        # The loaded data['expected_light_source_enum_output'] will have np.nan.
        # The py_light_source_enum might have float('inf'). Need to reconcile.

        # Let's assume for comparison, we convert Python's float('inf') to np.nan
        py_enum_for_compare = np.where(py_light_source_enum == float('inf'), np.nan, py_light_source_enum)

        np.testing.assert_allclose(
            py_enum_for_compare, # Python output (inf replaced with nan)
            loaded_data['expected_light_source_enum_output'], # MATLAB output (should have nan)
            rtol=0, atol=0, equal_nan=True, # Exact match for enums, NaNs should be equal
            err_msg="Mismatch between Python and MATLAB lightSource_enum outputs."
        )
        print("SUCCESS: Python light_source_enum output matches MATLAB output (NaNs treated as equal).")
    except AssertionError as e:
        print("FAILURE: light_source_enum outputs do NOT match.")
        print(e)
        # print("Python enum output (inf replaced with nan):\n", py_enum_for_compare)
        # print("MATLAB expected enum output:\n", loaded_data['expected_light_source_enum_output'])
        return False

    print("\nAll getAccessibilityMapPlanner comparisons PASSED.")
    return True

if __name__ == '__main__':
    if not run_gamp_comparison_test():
        print("\nMATLAB getAccessibilityMapPlanner Comparison Test FAILED.")
    else:
        print("\nMATLAB getAccessibilityMapPlanner Comparison Test PASSED.")
    print(f"Reminder: This test expects a '.mat' file named 'gamp_test_data.mat' in a directory "
          f"'./matlab_gamp_test_data/' with specific variable names for inputs and expected outputs from MATLAB.")
