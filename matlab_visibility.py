import numpy as np

def get_accessibility_map_planner_py(
    alpha: float,
    light_strength: float,
    light_pos: tuple, # (y, x) using 0-based indexing
    obstacle_map: np.ndarray, # 0 for obstacle, 1 for free space (complement of MATLAB's obstacle)
    accessibility_map_py: np.ndarray, # Output array, will be modified
    light_source_enum_py: np.ndarray, # Output array, will be modified
    iteration: int, # Current light source index/iteration number
    threshold: float
):
    """
    Python translation of MATLAB's getAccessibilityMapPlanner.m.

    Args:
        alpha: Decay factor (e.g., 1.0 for no decay beyond obstacles).
        light_strength: Initial light value at the source.
        light_pos: Tuple (row, col) of the light source (0-based).
        obstacle_map: NumPy array where 1 is free, 0 is obstacle.
                      (Note: MATLAB script uses 'obstacle' as complement of occupancy,
                       so 1 means free, 0 means obstacle. This matches our Python convention).
        accessibility_map_py: NumPy array to store computed visibility values.
                               Initialized appropriately (e.g., with zeros or ones) before call.
                               This function modifies it.
        light_source_enum_py: NumPy array to store the index of the light source
                              that made a cell visible above threshold. Modifies it.
                              NaN or a sentinel value (like float('inf')) indicates not yet assigned.
        iteration: The ID/index of the current light source.
        threshold: Visibility threshold to update light_source_enum_py.

    Returns:
        (accessibility_map_py, light_source_enum_py) - the modified arrays.
    """
    ny, nx = accessibility_map_py.shape # MATLAB uses (rows, cols) -> (ny, nx) from size()

    # MATLAB uses 1-based indexing, lightPos(1) is row, lightPos(2) is col.
    # Python uses 0-based, so light_pos[0] is row (y), light_pos[1] is col (x).
    lp_y, lp_x = light_pos[0], light_pos[1]

    # Ensure accessibility_map_py is float for calculations
    if not np.issubdtype(accessibility_map_py.dtype, np.floating):
        accessibility_map_py = accessibility_map_py.astype(float)

    # Temporary local visibility map for this specific light source, reset each call.
    # The MATLAB code passes 'accessibilityMap' and modifies it.
    # If 'accessibilityMap' is meant to be the *global* cumulative map, then the logic
    # of directly modifying it needs to be very careful, or a local copy should be used
    # for the propagation from the current light_pos, and then merged (e.g., max)
    # with a global map.
    # The MATLAB scripts c_sample_planner... and d_sample_planner... do:
    #   [current_vis_map, enum_map] = getAccessibilityMapPlanner(..., global_vis_map, enum_map, ...)
    #   map_builder = max(map_builder, current_vis_map);
    # This implies getAccessibilityMapPlanner calculates visibility from current light source only,
    # and it might be modifying a *copy* or a scratchpad 'accessibilityMap' passed to it.
    # Let's assume accessibility_map_py passed here is a scratchpad for the current light source.
    # So, we should initialize it or ensure it's clean.
    # The MATLAB example scripts pass `accessibilityMap` (ones(nx,ny)) to it.
    # This means it's a scratchpad that gets overwritten for the current light source.

    # For safety and clarity, let's create a local visibility map for this call.
    # The caller can then decide how to merge it (e.g., np.maximum).
    # However, the MATLAB code *directly modifies* the passed 'accessibilityMap'.
    # To match that, we will also directly modify accessibility_map_py.
    # The MATLAB example uses `accessibilityMap = ones(nx,ny)` then passes it.
    # This means it's being reset for each light source effectively.
    # This seems different from the C++/Python solver's local `visibility_` field that is reset.

    # Let's follow the MATLAB `getAccessibilityMapPlanner` which seems to calculate visibility
    # for the current `lightPos` and stores it in the passed `accessibilityMap`.
    # It does not accumulate, the accumulation (max) is done in the calling script.
    # So, we should clear/reset `accessibility_map_py` at the start of this function's effect
    # for the current light source, or ensure the caller provides a fresh one.
    # The C++ `updateVisibility` uses a local `visibility_` field that's reset.
    # MATLAB's example scripts pass a fresh `ones(nx,ny)` map named `accessibilityMap`
    # to `getAccessibilityMapPlanner`. So, this function *calculates* the local visibility.

    # Let's rename the input `accessibility_map_py` to `local_vis_map_out` for clarity.
    # And assume it's pre-initialized (e.g. to zeros) by the caller.
    # The MATLAB code initializes the point (lightPos) to lightStrength, and then propagates.
    # If we modify `accessibility_map_py` in place, it should be a zeroed map initially.
    # For this translation, we will assume accessibility_map_py is correctly initialized (e.g. to zeros)
    # by the caller, and we will fill it.

    # Quadrant 1: (x increases, y increases)
    # MATLAB: currentX = lightPos(1) + i; currentY = lightPos(2) + j;
    # Python: curr_r = lp_y + i_y; curr_c = lp_x + i_x; (i_y is like MATLAB's i, i_x like MATLAB's j)
    # MATLAB loops i=0:max_col, j=0:max_row. max_col for Q1 is nx-lightPos(1). nx is num_rows.
    # So MATLAB's i is row offset, j is col offset.
    # MATLAB map(row, col) -> Python map[row, col]

    # Loop bounds in MATLAB:
    # Q1: i = 0:(nx-lp_y), j = 0:(ny-lp_x) (nx=num_rows, ny=num_cols)
    # currentX = lp_y + i (row)
    # currentY = lp_x + j (col)

    # Quadrant 1
    for i_row_offset in range(ny - lp_y): # MATLAB's i loop (row offset)
        curr_r = lp_y + i_row_offset
        for i_col_offset in range(nx - lp_x): # MATLAB's j loop (col offset)
            curr_c = lp_x + i_col_offset
            v = 0.0
            if i_row_offset == 0 and i_col_offset == 0:
                v = light_strength
            elif i_row_offset == 0: # Moving along same row, increasing col
                v = alpha * accessibility_map_py[curr_r, curr_c - 1]
            elif i_col_offset == 0: # Moving along same col, increasing row
                v = alpha * accessibility_map_py[curr_r - 1, curr_c]
            elif i_row_offset == i_col_offset: # Diagonal
                v = alpha * accessibility_map_py[curr_r - 1, curr_c - 1]
            elif i_row_offset > i_col_offset: # Predominantly row-wise step
                # MATLAB: c = (currentY-lightPos(2))/(currentX-lightPos(1)); (col_offset / row_offset)
                c_slope = i_col_offset / i_row_offset if i_row_offset != 0 else 0
                # MATLAB: f = accessibilityMap(currentX-1,currentY) - c * (accessibilityMap(currentX-1,currentY)-accessibilityMap(currentX-1,currentY-1));
                # f = vis(r-1, c) - c_slope * ( vis(r-1,c) - vis(r-1,c-1) )
                val_prev_r_curr_c = accessibility_map_py[curr_r - 1, curr_c]
                val_prev_r_prev_c = accessibility_map_py[curr_r - 1, curr_c - 1]
                f = val_prev_r_curr_c - c_slope * (val_prev_r_curr_c - val_prev_r_prev_c)
                v = alpha * f
            elif i_col_offset > i_row_offset: # Predominantly col-wise step
                # MATLAB: c = (currentX-lightPos(1))/(currentY-lightPos(2)); (row_offset / col_offset)
                c_slope = i_row_offset / i_col_offset if i_col_offset != 0 else 0
                # MATLAB: f = accessibilityMap(currentX,currentY-1) - c * (accessibilityMap(currentX,currentY-1) - accessibilityMap(currentX-1,currentY-1));
                # f = vis(r,c-1) - c_slope * ( vis(r,c-1) - vis(r-1,c-1) )
                val_curr_r_prev_c = accessibility_map_py[curr_r, curr_c - 1]
                val_prev_r_prev_c = accessibility_map_py[curr_r - 1, curr_c - 1]
                f = val_curr_r_prev_c - c_slope * (val_curr_r_prev_c - val_prev_r_prev_c)
                v = alpha * f

            v_final = v * obstacle_map[curr_r, curr_c]
            accessibility_map_py[curr_r, curr_c] = v_final
            if v_final >= threshold:
                # MATLAB uses isnan for lightSource_enum. If Python uses float('inf'), check for that.
                if light_source_enum_py[curr_r, curr_c] == float('inf') or np.isnan(light_source_enum_py[curr_r, curr_c]):
                    light_source_enum_py[curr_r, curr_c] = iteration

    # Quadrant 2
    # MATLAB: i = 0:(lp_y-1), j = 0:(nx-lp_x) (nx is num_cols here in MATLAB due to size(map,2))
    # Python: i_row_offset for rows (negative), i_col_offset for columns (positive)
    # curr_r = lp_y - i_row_offset
    # curr_c = lp_x + i_col_offset
    for i_row_offset in range(lp_y + 1): # 0 to lp_y
        curr_r = lp_y - i_row_offset
        for i_col_offset in range(nx - lp_x): # 0 to num_cols - lp_x -1
            curr_c = lp_x + i_col_offset
            if not (0 <= curr_r < ny and 0 <= curr_c < nx): continue # Boundary check
            v = 0.0
            if i_row_offset == 0 and i_col_offset == 0: # This is light_pos itself
                v = light_strength # Should have been set by Q1, or handle if Q1 is skipped
            elif i_row_offset == 0: # Moving along same row (lp_y), increasing col from lp_x
                v = alpha * accessibility_map_py[curr_r, curr_c - 1]
            elif i_col_offset == 0: # Moving along same col (lp_x), decreasing row from lp_y
                v = alpha * accessibility_map_py[curr_r + 1, curr_c]
            elif i_row_offset == i_col_offset and i_row_offset > 0 : # Diagonal (up-right)
                v = alpha * accessibility_map_py[curr_r + 1, curr_c - 1]
            elif i_row_offset > i_col_offset: # Predominantly row-wise step (moving "up" more)
                # MATLAB: c = (currentY-lightPos(2))/(currentX-lightPos(1)); (col_offset / (-row_offset))
                c_slope = i_col_offset / (-i_row_offset) if i_row_offset != 0 else 0
                # MATLAB: f = accessibilityMap(currentX+1,currentY) + c * (accessibilityMap(currentX+1,currentY)-accessibilityMap(currentX+1,currentY-1));
                # f = vis(r+1,c) + c_slope * (vis(r+1,c) - vis(r+1,c-1))
                val_next_r_curr_c = accessibility_map_py[curr_r + 1, curr_c]
                val_next_r_prev_c = accessibility_map_py[curr_r + 1, curr_c - 1]
                f = val_next_r_curr_c + c_slope * (val_next_r_curr_c - val_next_r_prev_c)
                v = alpha * f
            elif i_col_offset > i_row_offset: # Predominantly col-wise step (moving "right" more)
                # MATLAB: c = (currentX-lightPos(1))/(currentY-lightPos(2)); ((-row_offset) / col_offset)
                c_slope = (-i_row_offset) / i_col_offset if i_col_offset != 0 else 0
                # MATLAB: f = accessibilityMap(currentX,currentY-1) + c * (accessibilityMap(currentX,currentY-1) - accessibilityMap(currentX+1,currentY-1));
                # f = vis(r,c-1) + c_slope * (vis(r,c-1) - vis(r+1,c-1))
                val_curr_r_prev_c = accessibility_map_py[curr_r, curr_c - 1]
                val_next_r_prev_c = accessibility_map_py[curr_r + 1, curr_c - 1]
                f = val_curr_r_prev_c + c_slope * (val_curr_r_prev_c - val_next_r_prev_c)
                v = alpha * f

            v_final = v * obstacle_map[curr_r, curr_c]
            accessibility_map_py[curr_r, curr_c] = v_final
            if v_final >= threshold:
                if light_source_enum_py[curr_r, curr_c] == float('inf') or np.isnan(light_source_enum_py[curr_r, curr_c]):
                    light_source_enum_py[curr_r, curr_c] = iteration

    # Quadrant 3
    # curr_r = lp_y - i_row_offset
    # curr_c = lp_x - i_col_offset
    for i_row_offset in range(lp_y + 1):
        curr_r = lp_y - i_row_offset
        for i_col_offset in range(lp_x + 1):
            curr_c = lp_x - i_col_offset
            if not (0 <= curr_r < ny and 0 <= curr_c < nx): continue
            v = 0.0
            if i_row_offset == 0 and i_col_offset == 0: v = light_strength
            elif i_row_offset == 0: # Moving along row lp_y, decreasing col from lp_x
                v = alpha * accessibility_map_py[curr_r, curr_c + 1]
            elif i_col_offset == 0: # Moving along col lp_x, decreasing row from lp_y
                v = alpha * accessibility_map_py[curr_r + 1, curr_c]
            elif i_row_offset == i_col_offset and i_row_offset > 0: # Diagonal (up-left)
                v = alpha * accessibility_map_py[curr_r + 1, curr_c + 1]
            elif i_row_offset > i_col_offset: # Predominantly row-wise ("up" more)
                # MATLAB: c = (currentY-lightPos(2))/(currentX-lightPos(1)); ((-col_offset) / (-row_offset)) = col/row
                c_slope = i_col_offset / i_row_offset if i_row_offset != 0 else 0
                # MATLAB: f = accessibilityMap(currentX+1,currentY) - c * (accessibilityMap(currentX+1,currentY)-accessibilityMap(currentX+1,currentY+1));
                # f = vis(r+1,c) - c_slope * (vis(r+1,c) - vis(r+1,c+1))
                val_next_r_curr_c = accessibility_map_py[curr_r + 1, curr_c]
                val_next_r_next_c = accessibility_map_py[curr_r + 1, curr_c + 1]
                f = val_next_r_curr_c - c_slope * (val_next_r_curr_c - val_next_r_next_c)
                v = alpha * f
            elif i_col_offset > i_row_offset: # Predominantly col-wise ("left" more)
                # MATLAB: c = (currentX-lightPos(1))/(currentY-lightPos(2)); ((-row_offset) / (-col_offset)) = row/col
                c_slope = i_row_offset / i_col_offset if i_col_offset != 0 else 0
                # MATLAB: f = accessibilityMap(currentX,currentY+1) - c * (accessibilityMap(currentX,currentY+1) - accessibilityMap(currentX+1,currentY+1));
                # f = vis(r,c+1) - c_slope * (vis(r,c+1) - vis(r+1,c+1))
                val_curr_r_next_c = accessibility_map_py[curr_r, curr_c + 1]
                val_next_r_next_c = accessibility_map_py[curr_r + 1, curr_c + 1]
                f = val_curr_r_next_c - c_slope * (val_curr_r_next_c - val_next_r_next_c)
                v = alpha * f

            v_final = v * obstacle_map[curr_r, curr_c]
            accessibility_map_py[curr_r, curr_c] = v_final
            if v_final >= threshold:
                if light_source_enum_py[curr_r, curr_c] == float('inf') or np.isnan(light_source_enum_py[curr_r, curr_c]):
                    light_source_enum_py[curr_r, curr_c] = iteration

    # Quadrant 4
    # curr_r = lp_y + i_row_offset
    # curr_c = lp_x - i_col_offset
    for i_row_offset in range(ny - lp_y):
        curr_r = lp_y + i_row_offset
        for i_col_offset in range(lp_x + 1):
            curr_c = lp_x - i_col_offset
            if not (0 <= curr_r < ny and 0 <= curr_c < nx): continue
            v = 0.0
            if i_row_offset == 0 and i_col_offset == 0: v = light_strength
            elif i_row_offset == 0: # Moving along row lp_y, decreasing col
                v = alpha * accessibility_map_py[curr_r, curr_c + 1]
            elif i_col_offset == 0: # Moving along col lp_x, increasing row
                v = alpha * accessibility_map_py[curr_r - 1, curr_c]
            elif i_row_offset == i_col_offset and i_row_offset > 0: # Diagonal (down-left)
                v = alpha * accessibility_map_py[curr_r - 1, curr_c + 1]
            elif i_row_offset > i_col_offset: # Predominantly row-wise ("down" more)
                # MATLAB: c = (currentY-lightPos(2))/(currentX-lightPos(1)); ((-col_offset) / row_offset)
                c_slope = (-i_col_offset) / i_row_offset if i_row_offset != 0 else 0
                # MATLAB: f = accessibilityMap(currentX-1,currentY) + c * (accessibilityMap(currentX-1,currentY)-accessibilityMap(currentX-1,currentY+1));
                # f = vis(r-1,c) + c_slope * (vis(r-1,c) - vis(r-1,c+1))
                val_prev_r_curr_c = accessibility_map_py[curr_r - 1, curr_c]
                val_prev_r_next_c = accessibility_map_py[curr_r - 1, curr_c + 1]
                f = val_prev_r_curr_c + c_slope * (val_prev_r_curr_c - val_prev_r_next_c)
                v = alpha * f
            elif i_col_offset > i_row_offset: # Predominantly col-wise ("left" more)
                # MATLAB: c = (currentX-lightPos(1))/(currentY-lightPos(2)); (row_offset / (-col_offset))
                c_slope = i_row_offset / (-i_col_offset) if i_col_offset != 0 else 0
                # MATLAB: f = accessibilityMap(currentX,currentY+1) + c * (accessibilityMap(currentX,currentY+1) - accessibilityMap(currentX-1,currentY+1));
                # f = vis(r,c+1) + c_slope * (vis(r,c+1) - vis(r-1,c+1))
                val_curr_r_next_c = accessibility_map_py[curr_r, curr_c + 1]
                val_prev_r_next_c = accessibility_map_py[curr_r - 1, curr_c + 1]
                f = val_curr_r_next_c + c_slope * (val_curr_r_next_c - val_prev_r_next_c)
                v = alpha * f

            v_final = v * obstacle_map[curr_r, curr_c]
            accessibility_map_py[curr_r, curr_c] = v_final
            if v_final >= threshold:
                if light_source_enum_py[curr_r, curr_c] == float('inf') or np.isnan(light_source_enum_py[curr_r, curr_c]):
                    light_source_enum_py[curr_r, curr_c] = iteration

    return accessibility_map_py, light_source_enum_py


if __name__ == '__main__':
    print("Testing MATLAB visibility function get_accessibility_map_planner_py...")

    # Test case 1: Simple
    rows, cols = 5, 5
    obstacle = np.ones((rows, cols), dtype=float) # All free
    # obstacle[2,2] = 0 # Add an obstacle

    # Initialize maps
    # accessibility_map should be initialized by the caller before each call to this function
    # if it's meant to represent only the current light source's contribution.
    # The MATLAB sample scripts pass `accessibilityMap = ones(nx,ny)` then it's modified.
    # This implies the function calculates local visibility.

    # Test 1: Light source at (0,0)
    print("\nTest 1: Source at (0,0)")
    local_vis_map1 = np.zeros((rows, cols), dtype=float)
    enum_map1 = np.full((rows, cols), float('inf'), dtype=float)

    get_accessibility_map_planner_py(
        alpha=1.0, light_strength=1.0, light_pos=(0,0),
        obstacle_map=obstacle, accessibility_map_py=local_vis_map1,
        light_source_enum_py=enum_map1, iteration=1, threshold=0.5
    )
    print("Local Visibility Map (source 0,0):\n", local_vis_map1)
    print("Enum Map (source 0,0):\n", enum_map1)

    # Test 2: Light source at center, with an obstacle
    print("\nTest 2: Source at (2,2) with obstacle at (2,3)")
    obstacle2 = np.ones((rows, cols), dtype=float)
    obstacle2[2,3] = 0 # Obstacle to the right of light source
    local_vis_map2 = np.zeros((rows, cols), dtype=float)
    enum_map2 = np.full((rows, cols), float('inf'), dtype=float)

    get_accessibility_map_planner_py(
        alpha=1.0, light_strength=1.0, light_pos=(2,2),
        obstacle_map=obstacle2, accessibility_map_py=local_vis_map2,
        light_source_enum_py=enum_map2, iteration=1, threshold=0.5
    )
    print("Obstacle Map:\n", obstacle2)
    print("Local Visibility Map (source 2,2):\n", local_vis_map2)
    print("Enum Map (source 2,2):\n", enum_map2)

    # Test 3: Alpha < 1 (decay)
    print("\nTest 3: Source at (0,0) with alpha=0.8 (decay)")
    local_vis_map3 = np.zeros((rows, cols), dtype=float)
    enum_map3 = np.full((rows, cols), float('inf'), dtype=float)
    get_accessibility_map_planner_py(
        alpha=0.8, light_strength=1.0, light_pos=(0,0),
        obstacle_map=obstacle, accessibility_map_py=local_vis_map3,
        light_source_enum_py=enum_map3, iteration=1, threshold=0.3 # Lower threshold due to decay
    )
    print("Local Visibility Map (alpha=0.8):\n", local_vis_map3)
    print("Enum Map (alpha=0.8):\n", enum_map3)

    print("\nMATLAB visibility tests completed.")
