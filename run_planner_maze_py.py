import numpy as np
import matplotlib.pyplot as plt
import time
import random
import os # <-- Added import
from PIL import Image # For loading maze image

# Assuming these files are in the same directory or Python path
from vbs_parser import VBSConfig # Using VBSConfig for consistency, though not parsing file here
from vbs_environment import Environment # For image loading via Environment class
from matlab_visibility import get_accessibility_map_planner_py # Using the MATLAB-ported visibility

def run_planner_maze():
    print("Running Python Planner for Maze Environments (based on MATLAB d_sample_planner...m)")

    # Configuration (similar to d_sample_planner...)
    image_path = "images/maze_1.png" # Default maze from MATLAB example
    # Check if default maze exists, otherwise try to create a dummy one or use a known one from repo
    try:
        img_check = Image.open(image_path)
        nrows_raw, ncols_raw = img_check.height, img_check.width # PIL Image.size is (width,height)
    except FileNotFoundError:
        print(f"Warning: Maze image '{image_path}' not found.")
        # Fallback to a known image from the repository if possible, or skip
        # For now, let's assume it exists or this script won't run properly.
        # If a general solution is needed, create a dummy maze.
        # For this conversion, we assume 'images/maze_1.png' is available as in the original repo.
        print("Please ensure 'images/maze_1.png' is available.")
        return

    # Define start and end points for maze_1.png (from MATLAB script)
    # MATLAB uses [row, col] 1-based. Python will use 0-based.
    # MATLAB sp_o = [315 169]; ep_o = [7 153];
    # These seem to be for a specific maze image. We need to ensure they are valid for the loaded image.
    # The image `maze_1.png` in `MATLAB_code/mazes/` seems to be 321x321 based on its use.
    # If `images/maze_1.png` is different, these will need adjustment.
    # Let's use the dimensions from the loaded image.

    # For `MATLAB_code/mazes/maze_1.png` (321x321)
    # sp_matlab = [315, 169] -> sp_py = (314, 168)
    # ep_matlab = [7, 153]   -> ep_py = (6, 152)
    # These need to be verified against the actual `images/maze_1.png` at the root.
    # The `images/` dir at root has maze_1.png which is 403x403.
    # Let's assume we're using `MATLAB_code/mazes/maze_1.png` (321x321) for now
    # And adjust if the main `images/maze_1.png` (403x403) is the target.
    # For robustness, it's better to use relative positions if possible or smaller mazes for tests.

    # Using Environment class to load the image and get obstacle map
    # Create a minimal VBSConfig for Environment
    cfg_for_env = VBSConfig(mode=2, image_path=image_path, silent=True)
    env = Environment(cfg_for_env)

    if env.nx == 0 or env.ny == 0:
        print(f"Failed to load environment from image: {image_path}")
        return

    obstacle_map = env.visibility_field.data_ # 1.0 for free, 0.0 for obstacle
    nrows, ncols = env.ny, env.nx # Actual dimensions from loaded image

    # Define start/end based on the MATLAB script's values for its maze_1.png (321x321)
    # Convert from 1-based (MATLAB) to 0-based (Python)
    sp_o = (314, 168)
    ep_o = (6, 152)

    # Validate start/end points are within bounds for the loaded image
    if not (0 <= sp_o[0] < nrows and 0 <= sp_o[1] < ncols and \
            0 <= ep_o[0] < nrows and 0 <= ep_o[1] < ncols):
        print(f"Error: Start/End points out of bounds for loaded image ({nrows}x{ncols}).")
        print(f"SP: {sp_o}, EP: {ep_o}. Adjust based on '{image_path}'.")
        # Attempt to use default valid start/end for testing if above are bad
        sp_o = (min(1, nrows-2), min(1, ncols-2)) # e.g., (1,1)
        ep_o = (max(1, nrows-2), max(1,ncols-2)) # e.g., ( H-2, W-2)
        print(f"Using fallback SP: {sp_o}, EP: {ep_o}")


    # Ensure start/end points are not obstacles
    if obstacle_map[sp_o[0], sp_o[1]] == 0.0:
        print(f"Warning: Start point {sp_o} is an obstacle. Attempting to make it free.")
        obstacle_map[sp_o[0], sp_o[1]] = 1.0
    if obstacle_map[ep_o[0], ep_o[1]] == 0.0:
        print(f"Warning: End point {ep_o} is an obstacle. Attempting to make it free.")
        obstacle_map[ep_o[0], ep_o[1]] = 1.0

    cfg = VBSConfig( # Main config for this planner run
        start=sp_o, end=ep_o,
        visibility_threshold=0.2, # From MATLAB d_sample_planner...
        light_strength=1.0,
        max_iter = 200, # Potentially needs more for complex mazes
        silent=False
    )

    light_source_enum = np.full((nrows, ncols), float('inf'), dtype=float)
    if 0 <= sp_o[0] < nrows and 0 <= sp_o[1] < ncols: # Re-check after potential fallback
        light_source_enum[sp_o[0], sp_o[1]] = 0
    else: # Should not happen if fallback logic is okay
        print("Critical error: Start point still invalid after fallback.")
        return

    current_accessibility_map = np.zeros((nrows, ncols), dtype=float)
    current_accessibility_map, light_source_enum = get_accessibility_map_planner_py(
        alpha=1.0, light_strength=cfg.light_strength, light_pos=sp_o,
        obstacle_map=obstacle_map, accessibility_map_py=current_accessibility_map,
        light_source_enum_py=light_source_enum, iteration=0,
        threshold=cfg.visibility_threshold
    )

    plt.figure(figsize=(10,10 * nrows/ncols if ncols > 0 else 1)) # Adjust aspect ratio
    plt.imshow(1 - obstacle_map, cmap='gray', origin='lower', interpolation='none')
    plt.plot(sp_o[1], sp_o[0], 'go', markersize=10, label='Start')
    plt.plot(ep_o[1], ep_o[0], 'bo', markersize=10, label='End')
    plt.title(f"Visibility Planner (Maze: {os.path.basename(image_path)}) - {nrows}x{ncols}")

    map_builder = np.copy(current_accessibility_map)

    wp = sp_o
    wp_old = wp
    wps_holder = [wp]

    max_iterations = cfg.max_iter
    path_found = False

    if map_builder[ep_o[0], ep_o[1]] > cfg.visibility_threshold:
        print(f"Target visible from start point.")
        wps_holder.append(ep_o)
        path_found = True

    iter_count = 0
    for iteration_idx in range(1, max_iterations + 1):
        if path_found: break
        iter_count = iteration_idx

        candidate_indices = np.argwhere(map_builder > cfg.visibility_threshold)
        if candidate_indices.shape[0] == 0:
            print("No candidate cells above threshold. Path not found.")
            break

        xq = candidate_indices[:, 0]; yq = candidate_indices[:, 1]
        v_visibility = map_builder[xq, yq]

        d_to_target = np.sqrt((xq - ep_o[0])**2 + (yq - ep_o[1])**2)
        # MATLAB d_sample_planner uses d_sp = sqrt( (xq-wp(1)).^2 + (yq-wp(2)).^2); (dist to CURRENT wp)
        # This seems different from c_sample which uses wp_old. Let's use current wp as per d_sample.
        d_from_current_wp = np.sqrt((xq - wp[0])**2 + (yq - wp[1])**2)
        d_tot = d_to_target + d_from_current_wp # Heuristic from d_sample_planner

        min_v = np.min(v_visibility); max_v = np.max(v_visibility)
        if (max_v - min_v) == 0:
            v_scaled = np.full_like(v_visibility, np.min(d_tot) if d_tot.size > 0 else 0.0)
        else:
            min_d_tot = np.min(d_tot) if d_tot.size > 0 else 0.0
            max_d_tot = np.max(d_tot) if d_tot.size > 0 else 0.0
            v_scaled = (max_d_tot - min_d_tot) * (v_visibility - min_v) / (max_v - min_v) + min_d_tot

        fun = v_scaled + d_tot
        if fun.size == 0: print("Heuristic function empty. Path not found."); break

        min_fun_idx = np.argmin(fun)
        wp = (xq[min_fun_idx], yq[min_fun_idx])

        wps_holder.append(wp)
        plt.plot(wp[1], wp[0], 'co', markersize=7)
        # wp_old = wp # Not used if heuristic uses current wp

        current_accessibility_map.fill(0)
        current_accessibility_map, light_source_enum = get_accessibility_map_planner_py(
            alpha=1.0, light_strength=cfg.light_strength, light_pos=wp,
            obstacle_map=obstacle_map, accessibility_map_py=current_accessibility_map,
            light_source_enum_py=light_source_enum, iteration=iteration_idx,
            threshold=cfg.visibility_threshold
        )
        map_builder = np.maximum(map_builder, current_accessibility_map)

        # plt.pause(0.01) # For dynamic plotting if desired

        if map_builder[ep_o[0], ep_o[1]] > cfg.visibility_threshold:
            print(f"Target visual confirmed in {iteration_idx} iterations.")
            path_found = True
            wps_holder.append(ep_o)
            if light_source_enum[ep_o[0], ep_o[1]] == float('inf') or np.isnan(light_source_enum[ep_o[0], ep_o[1]]):
                 light_source_enum[ep_o[0], ep_o[1]] = iteration_idx
            break

    if path_found:
        print("Reconstructing and plotting final path...")
        final_path_points = []; pt = ep_o; final_path_points.append(pt)
        max_segments = len(wps_holder) + 5; count_segments = 0
        while not (pt[0] == sp_o[0] and pt[1] == sp_o[1]):
            count_segments +=1
            if count_segments > max_segments: print("Path reconstruction loop limit."); break
            parent_wp_idx_float = light_source_enum[pt[0], pt[1]]
            if parent_wp_idx_float==float('inf') or np.isnan(parent_wp_idx_float): print(f"Parent unknown for {pt}."); break
            parent_wp_idx = int(parent_wp_idx_float)
            if not (0 <= parent_wp_idx < len(wps_holder)): print(f"Invalid parent index {parent_wp_idx}."); break
            pt_1 = wps_holder[parent_wp_idx]
            plt.plot([pt[1], pt_1[1]], [pt[0], pt_1[0]], 'm-', linewidth=2)
            plt.plot(pt_1[1], pt_1[0], 'mo', markersize=5)
            final_path_points.append(pt_1); pt = pt_1
        final_path_points.reverse()
        print(f"Final path waypoints: {final_path_points}")
    else:
        print(f"Path not found after {iter_count} iterations.")

    plt.legend()
    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    run_planner_maze()
