import numpy as np
import heapq

from dataclasses import dataclass, field

@dataclass(order=True)
class AStarNode:
    f_score: float
    g_score: float = field(compare=False)
    y: int = field(compare=False)
    x: int = field(compare=False)

def _heuristic(y1, x1, y2, x2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def _generate_matlab_neighbor_offsets(connecting_distance):
    if connecting_distance == 0: return []

    dim = 2 * connecting_distance + 1
    neighbor_check = np.ones((dim, dim), dtype=bool)
    mid = connecting_distance
    neighbor_check[mid, mid] = False

    for i_p in range(connecting_distance - 1):
        neighbor_check[i_p, i_p] = False
        neighbor_check[dim - 1 - i_p, dim - 1 - i_p] = False
        neighbor_check[i_p, dim - 1 - i_p] = False
        neighbor_check[dim - 1 - i_p, i_p] = False
        neighbor_check[mid, i_p] = False
        neighbor_check[mid, dim - 1 - i_p] = False
        neighbor_check[i_p, mid] = False
        neighbor_check[dim - 1 - i_p, mid] = False

    rows, cols = np.where(neighbor_check)
    neighbors_relative_with_cost = []
    for r, c in zip(rows, cols):
        dy = r - mid; dx = c - mid
        cost = np.sqrt(dx**2 + dy**2)
        neighbors_relative_with_cost.append( ((dy, dx), cost) )
    return neighbors_relative_with_cost

def _is_line_of_sight_clear(y1, x1, y2, x2, map_grid, height, width):
    dy_total = y2 - y1; dx_total = x2 - x1
    if abs(dx_total) == 0 and abs(dy_total) == 0: return True

    jump_cells = 2 * max(abs(dx_total), abs(dy_total)) - 1
    if jump_cells <= 0: return True

    for k_step in range(1, jump_cells):
        y_offset_k = round(k_step * dy_total / jump_cells)
        x_offset_k = round(k_step * dx_total / jump_cells)
        check_y = y1 + int(y_offset_k)
        check_x = x1 + int(x_offset_k)

        if not (0 <= check_y < height and 0 <= check_x < width): return False
        if map_grid[check_y, check_x] == 1: return False
    return True

def astar_path(start_y, start_x, map_grid, goal_register, connecting_distance=1, debug_failing_test=False):
    height, width = map_grid.shape

    # Using float64 for g_score and hn for potentially better precision
    g_score = np.full((height, width), np.inf, dtype=np.float64)
    hn = np.full((height, width), np.inf, dtype=np.float64)

    goal_coords = np.argwhere(goal_register == 1)
    if goal_coords.size == 0: return None

    for r_idx in range(height):
        for c_idx in range(width):
            if map_grid[r_idx, c_idx] == 0:
                hn[r_idx, c_idx] = min(_heuristic(r_idx, c_idx, gy, gx) for gy, gx in goal_coords)

    open_set_heap = []
    closed_set = np.array(map_grid, dtype=bool)
    parent_y = np.zeros((height, width), dtype=np.int16)
    parent_x = np.zeros((height, width), dtype=np.int16)

    g_score[start_y, start_x] = 0
    start_f_score = hn[start_y, start_x] # This will also be float64
    heapq.heappush(open_set_heap, AStarNode(float(start_f_score), 0.0, start_y, start_x)) # Ensure heap stores float

    neighbor_offsets_with_cost = _generate_matlab_neighbor_offsets(connecting_distance)
    expanded_count = 0

    while open_set_heap:
        expanded_count += 1
        current_node = heapq.heappop(open_set_heap)
        cy, cx = current_node.y, current_node.x

        if debug_failing_test:
            print(f"Expanding ({cy},{cx}), g={current_node.g_score:.2f}, h={hn[cy,cx]:.2f}, f={current_node.f_score:.2f}")

        # Using a small tolerance for g_score comparison with heap values
        if current_node.g_score > g_score[cy,cx] + 1e-7:
            if debug_failing_test: print(f"  Skipping outdated node for ({cy},{cx}), heap_g={current_node.g_score}, map_g={g_score[cy,cx]}")
            continue

        if goal_register[cy, cx] == 1:
            if debug_failing_test: print(f"Goal ({cy},{cx}) reached! Expanded {expanded_count} nodes.")
            path = []; curr_y_path, curr_x_path = cy, cx
            while not (curr_y_path == start_y and curr_x_path == start_x):
                path.append((curr_y_path, curr_x_path))
                prev_x = parent_x[curr_y_path, curr_x_path]
                prev_y = parent_y[curr_y_path, curr_x_path]
                curr_y_path, curr_x_path = prev_y, prev_x
            path.append((start_y, start_x)); return path[::-1]

        closed_set[cy, cx] = True

        for (dy_rel, dx_rel), cost_to_neighbor in neighbor_offsets_with_cost:
            ny, nx = cy + dy_rel, cx + dx_rel

            if not (0 <= ny < height and 0 <= nx < width): continue
            if closed_set[ny, nx]: continue

            line_clear = True
            if connecting_distance > 1:
                 if not _is_line_of_sight_clear(cy, cx, ny, nx, map_grid, height, width):
                     line_clear = False

            if not line_clear:
                if debug_failing_test: print(f"    Neighbor ({ny},{nx}): LOS blocked")
                continue

            tentative_g_score = g_score[cy, cx] + cost_to_neighbor

            if tentative_g_score < g_score[ny, nx] - 1e-7: # Using tolerance
                if debug_failing_test: print(f"    Neighbor ({ny},{nx}): new_g={tentative_g_score:.2f} (old_g={g_score[ny,nx]:.2f}), h={hn[ny,nx]:.2f}, new_f={tentative_g_score + hn[ny,nx]:.2f}")
                parent_y[ny, nx] = cy; parent_x[ny, nx] = cx
                g_score[ny, nx] = tentative_g_score
                new_f_score = tentative_g_score + hn[ny, nx]
                heapq.heappush(open_set_heap, AStarNode(float(new_f_score), float(tentative_g_score), ny, nx))

    if debug_failing_test: print(f"Open list empty, goal not reached. Expanded {expanded_count} nodes.")
    return None

if __name__ == '__main__':
    print("Testing A* Pathfinding with MATLAB-style neighbor generation...")

    map_simple = np.array([
        [0,0,0,0,1],[1,1,0,0,0],[0,0,0,1,0],[0,1,1,0,0],[0,0,0,0,0]], dtype=np.int8)
    goal_simple = np.zeros_like(map_simple); goal_simple[4,4]=1
    print(f"\nTest 1: Simple map (0,0) to (4,4), CD=1")
    path_simple = astar_path(0,0,map_simple,goal_simple,1)
    print(f"Path: {path_simple}")

    map_no_path = np.array([[0,1,0],[0,1,0],[0,1,0]],dtype=np.int8)
    goal_no_path = np.zeros_like(map_no_path); goal_no_path[0,2]=1
    print(f"\nTest 2: No path map (0,0) to (0,2), CD=1")
    path_no = astar_path(0,0,map_no_path,goal_no_path,1)
    print(f"Path: {path_no if path_no else 'None (Correct)'}")

    nr,nc=20,20; map_large=np.zeros((nr,nc),dtype=np.int8)
    map_large[5:15,8:12]=1; map_large[8,5:15]=1 # Cross / T-shape obstacle
    goal_large=np.zeros_like(map_large); sy_l,sx_l=1,1; gy_l,gx_l=nr-2,nc-2 # (1,1) to (18,18)
    goal_large[gy_l,gx_l]=1; map_large[sy_l,sx_l]=0; map_large[gy_l,gx_l]=0

    print(f"\nTest 3: Larger map ({sy_l},{sx_l}) to ({gy_l},{gx_l}), CD=1")
    path_large_cd1 = astar_path(sy_l,sx_l,map_large,goal_large,1, debug_failing_test=False) # Debug off for now
    print(f"Path (CD=1) len: {len(path_large_cd1) if path_large_cd1 else 'None'}")

    print(f"\nTest 4: Larger map ({sy_l},{sx_l}) to ({gy_l},{gx_l}), CD=2 (MATLAB-style sparse neighbors)")
    path_large_cd2_matlab = astar_path(sy_l,sx_l,map_large,goal_large,2)
    print(f"Path (CD=2 MATLAB-style) len: {len(path_large_cd2_matlab) if path_large_cd2_matlab else 'None'}")

    map_los = np.array([[0,0,0,0,0],[0,1,1,1,0],[0,0,0,0,0],[0,1,0,1,0],[0,0,0,0,0]],dtype=np.int8)
    goal_los = np.zeros_like(map_los); goal_los[4,4]=1
    print(f"\nTest 5: LOS test (0,0) to (4,4), CD=2 (MATLAB-style)")
    path_los_m = astar_path(0,0,map_los,goal_los,2)
    print(f"Path LOS CD=2 MATLAB-style: {path_los_m}")

    print("\nA* tests completed.")
