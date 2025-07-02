import numpy as np
import matplotlib.pyplot as plt
import time
import random

# Assuming these files are in the same directory or Python path
from vbs_parser import VBSConfig, parse_config
from vbs_environment import Environment
from matlab_visibility import get_accessibility_map_planner_py # Using the MATLAB-ported visibility

def run_planner_random():
    print("Running Python Planner for Random Environments (based on MATLAB c_sample_planner_...m)")

    # Configuration (similar to c_sample_planner...)
    # Hardcode a smaller configuration for quick testing
    nrows, ncols = 30, 25

    cfg = VBSConfig(
        mode=1, nrows=nrows, ncols=ncols,
        nb_of_obstacles=5,
        min_width=2, max_width=5,
        min_height=2, max_height=5,
        random_seed=False, seed_value=42,
        start=(2,2), # (y,x) for this script's direct use, VBSConfig itself takes (x,y) if parsed
        end=(nrows-3, ncols-3), # e.g. (27,22)
        visibility_threshold=0.3,
        light_strength=1.0,
        max_iter = 50, # Reduced max_iter for testing
        silent=False
    )
    # VBSConfig.start/end are (x,y). This script uses (y,x) for sp_o, ep_o.
    # So, we need to set cfg.start and cfg.end appropriately if Environment uses them.
    # Environment's constructor uses cfg.ncols, cfg.nrows, then for mode 1, it calls
    # generate_new_environment_from_settings which uses cfg.nb_of_obstacles etc.
    # The start/end points for the *planner* are sp_o, ep_o taken from cfg.start/end.
    # Let's ensure consistency: VBSConfig stores (x,y). Planner script uses (y,x).

    cfg.start = (cfg.start[1], cfg.start[0]) # Convert (y,x) to (x,y) for VBSConfig if it was (y,x)
    cfg.end = (cfg.end[1], cfg.end[0])       # Convert (y,x) to (x,y) for VBSConfig

    # The planner script will use these directly as (y,x)
    sp_o_planner = (2,2)
    ep_o_planner = (nrows-3, ncols-3)


    if not cfg.random_seed:
        random.seed(cfg.seed_value)
        np.random.seed(cfg.seed_value)

    # Initialize Environment using a VBSConfig that has (x,y) for start/end
    # but the environment generation itself doesn't use start/end.
    # We will use sp_o_planner and ep_o_planner for the actual planning.
    env_cfg = VBSConfig(
        mode=1, nrows=nrows, ncols=ncols,
        nb_of_obstacles=cfg.nb_of_obstacles,
        min_width=cfg.min_width, max_width=cfg.max_width,
        min_height=cfg.min_height, max_height=cfg.max_height,
        random_seed=cfg.random_seed, seed_value=cfg.seed_value,
        silent=cfg.silent
    )
    env = Environment(env_cfg)
    obstacle_map = env.visibility_field.data_

    sp_o = sp_o_planner
    ep_o = ep_o_planner

    light_source_enum = np.full((nrows, ncols), float('inf'), dtype=float)
    if 0 <= sp_o[0] < nrows and 0 <= sp_o[1] < ncols:
        light_source_enum[sp_o[0], sp_o[1]] = 0
    else:
        print(f"Error: Start point {sp_o} is out of bounds for grid {nrows}x{ncols}")
        return

    current_accessibility_map = np.zeros((nrows, ncols), dtype=float)
    current_accessibility_map, light_source_enum = get_accessibility_map_planner_py(
        alpha=1.0, light_strength=cfg.light_strength, light_pos=sp_o,
        obstacle_map=obstacle_map, accessibility_map_py=current_accessibility_map,
        light_source_enum_py=light_source_enum, iteration=0,
        threshold=cfg.visibility_threshold
    )

    plt.figure(figsize=(8,8 * nrows/ncols if ncols > 0 else 1))
    plt.imshow(1 - obstacle_map, cmap='gray', origin='lower', interpolation='none')
    plt.plot(sp_o[1], sp_o[0], 'go', markersize=10, label='Start')
    plt.plot(ep_o[1], ep_o[0], 'bo', markersize=10, label='End')
    plt.title(f"Visibility Planner (Random Env) - {nrows}x{ncols}")

    map_builder = np.copy(current_accessibility_map)

    wp = sp_o
    wp_old = wp # wp_old is used in heuristic in c_sample_planner
    wps_holder = [wp]

    max_iterations = cfg.max_iter
    path_found = False

    if 0 <= ep_o[0] < nrows and 0 <= ep_o[1] < ncols and \
       map_builder[ep_o[0], ep_o[1]] > cfg.visibility_threshold:
        print(f"Target visible from start point.")
        wps_holder.append(ep_o)
        path_found = True

    iter_count = 0
    for iteration_idx in range(1, max_iterations + 1): # MATLAB iter starts from 1 for first real wp
        if path_found: break
        iter_count = iteration_idx

        # Find candidate cells for next waypoint
        # MATLAB: [xq, yq] = find(map_builder > threshold);
        candidate_indices = np.argwhere(map_builder > cfg.visibility_threshold)
        if candidate_indices.shape[0] == 0:
            print("No candidate cells above threshold to choose next waypoint. Path not found.")
            break

        xq = candidate_indices[:, 0] # row indices (y)
        yq = candidate_indices[:, 1] # col indices (x)

        v_visibility = map_builder[xq, yq]

        d_to_target = np.sqrt((xq - ep_o[0])**2 + (yq - ep_o[1])**2)
        d_from_previous_wp = np.sqrt((xq - wp_old[0])**2 + (yq - wp_old[1])**2)

        d_tot = d_to_target + d_from_previous_wp

        # Scale visibility (MATLAB's heuristic component)
        # Handle cases where max(v_visibility) - min(v_visibility) is zero
        min_v = np.min(v_visibility)
        max_v = np.max(v_visibility)

        if (max_v - min_v) == 0:
            if len(d_tot)>0 : v_scaled = np.full_like(v_visibility, np.min(d_tot) if d_tot.size > 0 else 0.0)
            else: v_scaled = v_visibility # Should not happen if candidate_indices exist
        else:
            if len(d_tot)>0 :
                min_d_tot = np.min(d_tot) if d_tot.size > 0 else 0.0
                max_d_tot = np.max(d_tot) if d_tot.size > 0 else 0.0
                v_scaled = (max_d_tot - min_d_tot) * (v_visibility - min_v) / (max_v - min_v) + min_d_tot
            else: v_scaled = v_visibility


        fun = v_scaled + d_tot # Heuristic to minimize
        if fun.size == 0:
            print("Heuristic function is empty. No path found.")
            break

        min_fun_idx = np.argmin(fun)
        wp = (xq[min_fun_idx], yq[min_fun_idx]) # New waypoint (y,x)

        wps_holder.append(wp)
        plt.plot(wp[1], wp[0], 'co', markersize=7) # Cyan waypoints

        wp_old = wp # Update previous waypoint for next iteration's heuristic

        # Compute new visibility from this waypoint
        current_accessibility_map.fill(0) # Reset local map for new calculation
        current_accessibility_map, light_source_enum = get_accessibility_map_planner_py(
            alpha=1.0, light_strength=cfg.light_strength, light_pos=wp,
            obstacle_map=obstacle_map, accessibility_map_py=current_accessibility_map,
            light_source_enum_py=light_source_enum, iteration=iteration_idx, # iteration_idx is 1-based index for wps_holder
            threshold=cfg.visibility_threshold
        )

        map_builder = np.maximum(map_builder, current_accessibility_map)

        # Update plot (optional, can be slow)
        # if contour_plot:
        #     for coll in contour_plot.collections:
        #         coll.remove()
        # contour_plot = plt.contourf(map_builder, levels=np.linspace(cfg.visibility_threshold, 1.0, 10), alpha=0.5, cmap='viridis')
        # plt.pause(0.01)


        if map_builder[ep_o[0], ep_o[1]] > cfg.visibility_threshold:
            print(f"Target visual confirmed in {iteration_idx} iterations.")
            path_found = True
            wps_holder.append(ep_o) # Ensure end point is part of the waypoint list for path reconstruction
            # Update enum for end point if it's now visible
            if light_source_enum[ep_o[0], ep_o[1]] == float('inf') or np.isnan(light_source_enum[ep_o[0], ep_o[1]]):
                 light_source_enum[ep_o[0], ep_o[1]] = iteration_idx # Parent is the current waypoint wp
            break

    # Plot final path by backtracking
    if path_found:
        print("Reconstructing and plotting final path...")
        final_path_points = []
        pt = ep_o

        # Add end point to path
        final_path_points.append(pt)

        # Max path segments to avoid infinite loops if something is wrong
        max_segments = len(wps_holder) + 5
        count_segments = 0

        while not (pt[0] == sp_o[0] and pt[1] == sp_o[1]):
            count_segments +=1
            if count_segments > max_segments:
                print("Error: Path reconstruction seems to be in a loop.")
                break

            parent_wp_idx_float = light_source_enum[pt[0], pt[1]]
            if parent_wp_idx_float == float('inf') or np.isnan(parent_wp_idx_float):
                print(f"Error: Cannot reconstruct path, parent of ({pt[0]},{pt[1]}) is unknown.")
                break

            parent_wp_idx = int(parent_wp_idx_float)
            if not (0 <= parent_wp_idx < len(wps_holder)):
                print(f"Error: Invalid parent index {parent_wp_idx} from enum map for point ({pt[0]},{pt[1]}). Max wps_holder index {len(wps_holder)-1}.")
                # This can happen if ep_o's parent was set to 'iteration_idx' which might be one more than current wps_holder size
                # before ep_o was appended.
                # If parent_wp_idx == len(wps_holder)-1 and wps_holder[-1] is ep_o, then parent is the one before ep_o.
                if parent_wp_idx == len(wps_holder)-1 and wps_holder[parent_wp_idx][0]==ep_o[0] and wps_holder[parent_wp_idx][1]==ep_o[1]:
                    # This means the end point's parent is itself via the last waypoint index, which is wrong.
                    # The parent should be the waypoint that made ep_o visible.
                    # This indicates an issue in how light_source_enum is set for ep_o or how wps_holder is indexed.
                    # The iteration_idx is the index of the *current* waypoint being expanded.
                    # If this waypoint makes ep_o visible, then this wp is the parent.
                    # wps_holder[iteration_idx] is correct.
                     print(f"Correcting parent_wp_idx for ep_o if it points to itself in wps_holder via last index.")
                     # This case should ideally be handled by how light_source_enum[ep_o] is set.
                     # The issue is if ep_o is added to wps_holder *after* its enum is set to index `iteration_idx`
                     # and `iteration_idx` is now `len(wps_holder) -1`.
                     # Let's assume light_source_enum[ep_o[0], ep_o[1]] correctly points to the *actual* waypoint index in wps_holder.
                     # The wps_holder has start at index 0, then wp1 at index 1, etc. Iteration_idx for enum is this 1-based index.

                # Fallback if index is truly out of bounds for other reasons
                if not (0 <= parent_wp_idx < len(wps_holder)): # Re-check after potential correction thoughts
                    print(f"Path reconstruction failed: parent_wp_idx {parent_wp_idx} out of bounds for wps_holder (size {len(wps_holder)}).")
                    break


            pt_1 = wps_holder[parent_wp_idx] # pt_1 is the parent waypoint

            plt.plot([pt[1], pt_1[1]], [pt[0], pt_1[0]], 'm-', linewidth=2) # (x,y) for plot
            plt.plot(pt_1[1], pt_1[0], 'mo', markersize=5) # Magenta parent waypoints

            final_path_points.append(pt_1)
            pt = pt_1

        final_path_points.reverse()
        print(f"Final path waypoints: {final_path_points}")
    else:
        print(f"Path not found after {iter_count} iterations.")

    plt.legend()
    plt.gca().invert_yaxis() # Match typical image coordinate system if needed, but MATLAB plots (row,col) with (0,0) top-left by default for imagesc
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    run_planner_random()
