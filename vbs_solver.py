import numpy as np
import heapq
import math
import os
import random # For benchmark_series obstacle generation
import time # For benchmark timing

from vbs_parser import VBSConfig
from vbs_field import Field
from vbs_environment import Environment # For type hinting and potentially direct use if needed


# Node structure for the heap (priority queue)
from dataclasses import dataclass, field as dc_field

@dataclass(order=True)
class Node:
    h: float
    x: int = dc_field(compare=False)
    y: int = dc_field(compare=False)


class VisibilityBasedSolver:
    def __init__(self, env: Environment):
        self.config_ = env.config
        self.occupancy_complement_ = env.visibility_field

        self.nx_ = self.occupancy_complement_.nx
        self.ny_ = self.occupancy_complement_.ny
        self.size_ = self.nx_ * self.ny_

        if self.nx_ == 0 or self.ny_ == 0:
            print("Warning: Solver initialized with an empty environment.")
            return

        self.visibility_threshold_ = self.config_.visibility_threshold
        self.light_strength_ = self.config_.light_strength

        self.visibility_global_ = Field(self.nx_, self.ny_, 0.0)
        self.visibility_ = Field(self.nx_, self.ny_, 0.0)
        self.visibility_raycasting_ = Field(self.nx_, self.ny_, 1.0)
        self.came_from_ = Field(self.nx_, self.ny_, float('inf'))
        self.light_sources_ = []
        self.heap_ = []
        self.scale_ = math.sqrt(self.ny_**2 + self.nx_**2) if self.ny_ > 0 or self.nx_ > 0 else 0.0
        self.c_ = 1.0
        self.global_iter_ = 0
        self.max_iter_ = self.config_.max_iter
        self.start_pt_ = (0,0); self.end_pt_ = (0,0)
        self.ls_ = (0,0)
        self.max_x_ = 0; self.max_y_ = 0
        self._reset_solver_state()

    def _reset_solver_state(self):
        if self.nx_ == 0 or self.ny_ == 0: return
        self.visibility_global_.reset(0.0)
        self.visibility_.reset(0.0)
        self.visibility_raycasting_.reset(1.0)
        self.came_from_.reset(float('inf'))
        self.light_sources_ = []
        self.heap_ = []
        heapq.heapify(self.heap_)
        self.global_iter_ = 0

    def _is_valid(self, x: int, y: int) -> bool:
        return 0 <= x < self.nx_ and 0 <= y < self.ny_

    def _eval_d(self, x1: int, y1: int, x2: int, y2: int) -> float:
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def _reset_priority_queue(self):
        self.heap_ = []

    def solve(self):
        if self.nx_ == 0 or self.ny_ == 0:
            print("Error: Cannot solve in an empty environment.")
            return []

        time_start = time.perf_counter()
        self.start_pt_ = self.config_.start
        self.end_pt_ = self.config_.end

        if not self._is_valid(self.start_pt_[0], self.start_pt_[1]) or \
           not self._is_valid(self.end_pt_[0], self.end_pt_[1]) or \
           self.occupancy_complement_.get_value(self.start_pt_[0], self.start_pt_[1]) == 0.0 or \
           self.occupancy_complement_.get_value(self.end_pt_[0], self.end_pt_[1]) == 0.0:
            if not self.config_.silent: print("Start/End point invalid or occupied.")
            return []

        self.ls_ = self.start_pt_
        self.light_sources_.append(self.start_pt_)
        self.came_from_.set_value(self.start_pt_[0], self.start_pt_[1], 0)
        self.visibility_global_.set_value(self.end_pt_[0], self.end_pt_[1], 0.0)

        path_found = False
        while self.visibility_global_.get_value(self.end_pt_[0], self.end_pt_[1]) <= self.visibility_threshold_:
            self._reset_priority_queue()
            self._update_visibility()

            if not self.heap_:
                if not self.config_.silent: print("Solver: Heap is empty. Path not found.")
                break

            best_node = heapq.heappop(self.heap_)
            self.ls_ = (best_node.x, best_node.y)
            self.light_sources_.append(self.ls_)

            if len(self.light_sources_) > self.max_iter_ :
                if not self.config_.silent: print(f"Max iterations ({self.max_iter_}) hit.")
                break

            if self.visibility_global_.get_value(self.end_pt_[0], self.end_pt_[1]) > self.visibility_threshold_:
                path_found = True

        time_stop = time.perf_counter()
        duration_us = (time_stop - time_start) * 1_000_000

        if not self.config_.silent and self.config_.timer:
            print(f"############################## Solver output ##############################\nExecution time in us: {duration_us:.0f}us")

        final_path = []
        if path_found:
            final_path = self._reconstruct_path(self.end_pt_)
            if not self.config_.silent: print(f"Path found with {len(final_path)} points.")
            if self.config_.save_results:
                self.save_solver_results()
                self.save_image_with_path(final_path)
        else:
            if not self.config_.silent: print("Path not found.")
        return final_path

    def _update_visibility(self):
        self.visibility_.reset(0.0)
        current_ls_index = len(self.light_sources_) - 1
        if current_ls_index < 0: return

        ls_x, ls_y = self.ls_
        offset = 0.0

        # Q1
        for i in range(self.nx_ - ls_x):
            cx = ls_x + i;
            for j in range(self.ny_ - ls_y):
                cy = ls_y + j; v = 0.0
                if i == 0 and j == 0: v = self.light_strength_
                elif i == 0: v = self.visibility_.get_value(cx, cy - 1)
                elif j == 0: v = self.visibility_.get_value(cx - 1, cy)
                elif i == j and i > 0: v = self.visibility_.get_value(cx - 1, cy - 1)
                elif i > j:
                    dx = i; dy = j; c_val = (dy + offset) / (dx + offset) if dx + offset != 0 else 0
                    v = self.visibility_.get_value(cx - 1, cy) - c_val * (self.visibility_.get_value(cx - 1, cy) - self.visibility_.get_value(cx - 1, cy - 1))
                elif j > i:
                    dx = i; dy = j; c_val = (dx + offset) / (dy + offset) if dy + offset != 0 else 0
                    v = self.visibility_.get_value(cx, cy - 1) - c_val * (self.visibility_.get_value(cx, cy - 1) - self.visibility_.get_value(cx - 1, cy - 1))
                v *= self.occupancy_complement_.get_value(cx, cy); self.visibility_.set_value(cx, cy, v)
                if v > self.visibility_global_.get_value(cx, cy):
                    self.visibility_global_.set_value(cx, cy, v)
                    if v >= self.visibility_threshold_ and self.came_from_.get_value(cx,cy) == float('inf'): self.came_from_.set_value(cx, cy, current_ls_index)
                if self.visibility_global_.get_value(cx, cy) >= self.visibility_threshold_:
                    parent_idx = int(self.came_from_.get_value(cx,cy)); parent_pt = self.light_sources_[parent_idx]
                    h = (self.scale_ * self.visibility_global_.get_value(cx,cy)) + (self._eval_d(cx,cy,self.end_pt_[0],self.end_pt_[1]) + self._eval_d(cx,cy,parent_pt[0],parent_pt[1]))
                    heapq.heappush(self.heap_, Node(h=-h, x=cx, y=cy))
        # Q2
        for i in range(ls_x + 1):
            cx = ls_x - i
            for j in range(self.ny_ - ls_y):
                cy = ls_y + j; v = 0.0
                if not self._is_valid(cx,cy): continue
                if i == 0 and j == 0: v = self.light_strength_
                elif i == 0: v = self.visibility_.get_value(cx, cy - 1)
                elif j == 0: v = self.visibility_.get_value(cx + 1, cy)
                elif i == j and i > 0: v = self.visibility_.get_value(cx + 1, cy - 1)
                elif i > j:
                    dx = i; dy = j; c_val = (dy + offset) / (dx + offset) if dx + offset != 0 else 0
                    v = self.visibility_.get_value(cx + 1, cy) - c_val * (self.visibility_.get_value(cx + 1, cy) - self.visibility_.get_value(cx + 1, cy - 1))
                elif j > i:
                    dx = i; dy = j; c_val = (dx + offset) / (dy + offset) if dy + offset != 0 else 0
                    v = self.visibility_.get_value(cx, cy - 1) - c_val * (self.visibility_.get_value(cx, cy - 1) - self.visibility_.get_value(cx + 1, cy - 1))
                v *= self.occupancy_complement_.get_value(cx, cy); self.visibility_.set_value(cx, cy, v)
                if v > self.visibility_global_.get_value(cx, cy):
                    self.visibility_global_.set_value(cx, cy, v)
                    if v >= self.visibility_threshold_ and self.came_from_.get_value(cx,cy) == float('inf'): self.came_from_.set_value(cx, cy, current_ls_index)
                if self.visibility_global_.get_value(cx, cy) >= self.visibility_threshold_:
                    parent_idx = int(self.came_from_.get_value(cx,cy)); parent_pt = self.light_sources_[parent_idx]
                    h = (self.scale_ * self.visibility_global_.get_value(cx,cy)) + (self._eval_d(cx,cy,self.end_pt_[0],self.end_pt_[1]) + self._eval_d(cx,cy,parent_pt[0],parent_pt[1]))
                    heapq.heappush(self.heap_, Node(h=-h, x=cx, y=cy))
        # Q3
        for i in range(ls_x + 1):
            cx = ls_x - i
            for j in range(ls_y + 1):
                cy = ls_y - j; v = 0.0
                if not self._is_valid(cx,cy): continue
                if i == 0 and j == 0: v = self.light_strength_
                elif i == 0: v = self.visibility_.get_value(cx, cy + 1)
                elif j == 0: v = self.visibility_.get_value(cx + 1, cy)
                elif i == j and i > 0: v = self.visibility_.get_value(cx + 1, cy + 1)
                elif i > j:
                    dx = i; dy = j; c_val = (dy + offset) / (dx + offset) if dx + offset != 0 else 0
                    v = self.visibility_.get_value(cx + 1, cy) - c_val * (self.visibility_.get_value(cx + 1, cy) - self.visibility_.get_value(cx + 1, cy + 1))
                elif j > i:
                    dx = i; dy = j; c_val = (dx + offset) / (dy + offset) if dy + offset != 0 else 0
                    v = self.visibility_.get_value(cx, cy + 1) - c_val * (self.visibility_.get_value(cx, cy + 1) - self.visibility_.get_value(cx + 1, cy + 1))
                v *= self.occupancy_complement_.get_value(cx, cy); self.visibility_.set_value(cx, cy, v)
                if v > self.visibility_global_.get_value(cx, cy):
                    self.visibility_global_.set_value(cx, cy, v)
                    if v >= self.visibility_threshold_ and self.came_from_.get_value(cx,cy) == float('inf'): self.came_from_.set_value(cx, cy, current_ls_index)
                if self.visibility_global_.get_value(cx, cy) >= self.visibility_threshold_:
                    parent_idx = int(self.came_from_.get_value(cx,cy)); parent_pt = self.light_sources_[parent_idx]
                    h = (self.scale_ * self.visibility_global_.get_value(cx,cy)) + (self._eval_d(cx,cy,self.end_pt_[0],self.end_pt_[1]) + self._eval_d(cx,cy,parent_pt[0],parent_pt[1]))
                    heapq.heappush(self.heap_, Node(h=-h, x=cx, y=cy))
        # Q4
        for i in range(self.nx_ - ls_x):
            cx = ls_x + i
            for j in range(ls_y + 1):
                cy = ls_y - j; v = 0.0
                if not self._is_valid(cx,cy): continue
                if i == 0 and j == 0: v = self.light_strength_
                elif i == 0: v = self.visibility_.get_value(cx, cy + 1)
                elif j == 0: v = self.visibility_.get_value(cx - 1, cy)
                elif i == j and i > 0: v = self.visibility_.get_value(cx - 1, cy + 1)
                elif i > j:
                    dx = i; dy = j; c_val = (dy + offset) / (dx + offset) if dx + offset != 0 else 0
                    v = self.visibility_.get_value(cx - 1, cy) - c_val * (self.visibility_.get_value(cx - 1, cy) - self.visibility_.get_value(cx - 1, cy + 1))
                elif j > i:
                    dx = i; dy = j; c_val = (dx + offset) / (dy + offset) if dy + offset != 0 else 0
                    v = self.visibility_.get_value(cx, cy + 1) - c_val * (self.visibility_.get_value(cx, cy + 1) - self.visibility_.get_value(cx - 1, cy + 1))
                v *= self.occupancy_complement_.get_value(cx, cy); self.visibility_.set_value(cx, cy, v)
                if v > self.visibility_global_.get_value(cx, cy):
                    self.visibility_global_.set_value(cx, cy, v)
                    if v >= self.visibility_threshold_ and self.came_from_.get_value(cx,cy) == float('inf'): self.came_from_.set_value(cx, cy, current_ls_index)
                if self.visibility_global_.get_value(cx, cy) >= self.visibility_threshold_:
                    parent_idx = int(self.came_from_.get_value(cx,cy)); parent_pt = self.light_sources_[parent_idx]
                    h = (self.scale_ * self.visibility_global_.get_value(cx,cy)) + (self._eval_d(cx,cy,self.end_pt_[0],self.end_pt_[1]) + self._eval_d(cx,cy,parent_pt[0],parent_pt[1]))
                    heapq.heappush(self.heap_, Node(h=-h, x=cx, y=cy))

    def _reconstruct_path(self, end_node_pt: tuple) -> list:
        path = []; curr_x, curr_y = end_node_pt
        if not self.light_sources_: return []
        path.append((curr_x, curr_y))
        if self.came_from_.get_value(curr_x, curr_y) == float('inf'):
            return path if (curr_x, curr_y) == self.start_pt_ else []
        current_ls_index = int(self.came_from_.get_value(curr_x, curr_y))
        while True:
            if not (0 <= current_ls_index < len(self.light_sources_)): break
            ls_x, ls_y = self.light_sources_[current_ls_index]; path.append((ls_x, ls_y))
            if (ls_x, ls_y) == self.start_pt_: break
            prev_ls_index = self.came_from_.get_value(ls_x, ls_y)
            if prev_ls_index == float('inf') or int(prev_ls_index) == current_ls_index: break
            current_ls_index = int(prev_ls_index)
        path.reverse()
        return [path[0]] + [p for i, p in enumerate(path[1:]) if p != path[i]] if len(path) > 1 else path

    def _save_text_field(self, field: Field, filename: str, output_dir: str = "./output_py"):
        filepath = os.path.join(output_dir, filename); os.makedirs(output_dir, exist_ok=True)
        try:
            with open(filepath, "w") as f:
                y_indices = range(self.ny_ -1, -1, -1) if self.config_.mode == 2 else range(self.ny_)
                for r_idx in y_indices:
                    f.write(" ".join(str(field.get_value(c_idx, r_idx)) for c_idx in range(self.nx_)) + " \n")
            if not self.config_.silent: print(f"Saved {filename} to {filepath}")
        except Exception as e: print(f"Error saving {filename} to {filepath}: {e}")

    def save_solver_results(self, output_dir: str = "./output_py"):
        if not self.config_.save_results: return
        if self.config_.save_came_from and self.came_from_.size > 0: self._save_text_field(self.came_from_, "cameFrom.txt", output_dir)
        if self.config_.save_global_visibility and self.visibility_global_.size > 0: self._save_text_field(self.visibility_global_, "VisibilityMap.txt", output_dir)
        if self.config_.save_local_visibility and self.visibility_.size > 0: self._save_text_field(self.visibility_, "LocalVisibilityMap.txt", output_dir)
        if self.config_.save_light_sources and self.light_sources_:
            filepath = os.path.join(output_dir, "lightSources.txt"); os.makedirs(output_dir, exist_ok=True)
            try:
                with open(filepath, "w") as f:
                    for x, y_orig in self.light_sources_:
                        f.write(f"{x} {self.ny_ - 1 - y_orig if self.config_.mode == 2 else y_orig}\n")
                if not self.config_.silent: print(f"Saved lightSources.txt to {filepath}")
            except Exception as e: print(f"Error saving lightSources.txt: {e}")

    def save_image_with_path(self, path: list, filename: str = "ResultingPath.png", output_dir: str = "./output_py"):
        if not self.config_.save_results or self.nx_ == 0 or self.ny_ == 0: return
        try:
            from PIL import Image, ImageDraw
            img_arr = np.array(self.occupancy_complement_.data_ * 255, dtype=np.uint8) # White free, Black obs
            img_pil = Image.fromarray(img_arr).convert('RGB'); draw = ImageDraw.Draw(img_pil)
            radius = self.config_.ball_radius
            if len(path) > 1: draw.line([(x,y) for x,y in path], fill=(255,0,255), width=max(1, radius//2+1)) # Magenta path
            for i, (x,y) in enumerate(path): # Cyan nodes, Green start, Red end
                color = (0,255,255)
                if i == 0: color = (0,255,0)
                elif i == len(path)-1: color = (255,0,0)
                draw.ellipse([x-radius, y-radius, x+radius, y+radius], fill=color, outline=color)
            filepath = os.path.join(output_dir,filename); os.makedirs(output_dir,exist_ok=True); img_pil.save(filepath)
            if not self.config_.silent: print(f"Saved path image to {filepath}")
        except Exception as e: print(f"Error saving path image: {e}")

    def _save_visibility_image(self, vis_field: Field, filename: str, output_dir: str ="./output_py"):
        if not self.config_.save_results or vis_field.size == 0: return
        try:
            from PIL import Image
            img_arr = np.array(vis_field.data_ * 255, dtype=np.uint8)
            rgb_arr = np.stack([img_arr]*3, axis=-1)
            occ_mask = self.occupancy_complement_.data_ == 0.0
            rgb_arr[occ_mask] = [255,0,0] # Red obstacles
            ls_x,ls_y=self.ls_; rad=self.config_.ball_radius
            y_coords, x_coords = np.ogrid[:self.ny_, :self.nx_]
            dist_sq_ls = (y_coords - ls_y)**2 + (x_coords - ls_x)**2
            rgb_arr[dist_sq_ls <= rad**2] = [255,255,0] # Yellow LS
            rgb_arr[(dist_sq_ls > rad**2) & (dist_sq_ls <= (rad+1)**2)] = [0,0,0] # Black ring
            img = Image.fromarray(rgb_arr, 'RGB'); filepath=os.path.join(output_dir,filename)
            os.makedirs(output_dir,exist_ok=True); img.save(filepath)
            if not self.config_.silent: print(f"Saved visibility image {filename} to {filepath}")
        except Exception as e: print(f"Error saving visibility image {filename}: {e}")

    def stand_alone_visibility(self):
        if self.nx_ == 0 or self.ny_ == 0: return
        self.start_pt_ = self.config_.start
        if not self._is_valid(self.start_pt_[0], self.start_pt_[1]) or \
           self.occupancy_complement_.get_value(self.start_pt_[0], self.start_pt_[1]) == 0.0:
            if not self.config_.silent: print("StandAloneVisibility: Start point invalid.")
            return
        self.ls_ = self.start_pt_; self._compute_visibility()
        if self.config_.save_results: self._save_visibility_image(self.visibility_, "standAloneVisibility.png")

    def _compute_visibility(self): # Corrected version
        self.visibility_.reset(0.0); ls_x, ls_y = self.ls_; offset = 0.0
        # Q1
        for i in range(self.nx_ - ls_x):
            cx = ls_x + i
            for j_idx in range(self.ny_ - ls_y):
                cy = ls_y + j_idx; v = 0.0
                if i == 0 and j_idx == 0: v = self.light_strength_
                elif i == 0: v = self.visibility_.get_value(cx, cy - 1)
                elif j_idx == 0: v = self.visibility_.get_value(cx - 1, cy)
                elif i == j_idx and i > 0: v = self.visibility_.get_value(cx - 1, cy - 1)
                elif i > j_idx:
                    dx=i; dy=j_idx; c_val=(dy+offset)/(dx+offset) if dx+offset!=0 else 0
                    v = self.visibility_.get_value(cx-1,cy) - c_val*(self.visibility_.get_value(cx-1,cy) - self.visibility_.get_value(cx-1,cy-1))
                elif j_idx > i:
                    dx=i; dy=j_idx; c_val=(dx+offset)/(dy+offset) if dy+offset!=0 else 0
                    v = self.visibility_.get_value(cx,cy-1) - c_val*(self.visibility_.get_value(cx,cy-1) - self.visibility_.get_value(cx-1,cy-1))
                self.visibility_.set_value(cx,cy, v * self.occupancy_complement_.get_value(cx,cy))
        # Q2
        for i in range(ls_x + 1):
            cx = ls_x - i
            for j_idx in range(self.ny_ - ls_y):
                cy = ls_y + j_idx; v = 0.0
                if not self._is_valid(cx,cy): continue
                if i == 0 and j_idx == 0: v = self.light_strength_
                elif i == 0: v = self.visibility_.get_value(cx,cy-1)
                elif j_idx == 0: v = self.visibility_.get_value(cx+1,cy)
                elif i == j_idx and i > 0: v = self.visibility_.get_value(cx+1,cy-1)
                elif i > j_idx:
                    dx=i; dy=j_idx; c_val=(dy+offset)/(dx+offset) if dx+offset!=0 else 0
                    v = self.visibility_.get_value(cx+1,cy) - c_val*(self.visibility_.get_value(cx+1,cy) - self.visibility_.get_value(cx+1,cy-1))
                elif j_idx > i:
                    dx=i; dy=j_idx; c_val=(dx+offset)/(dy+offset) if dy+offset!=0 else 0
                    v = self.visibility_.get_value(cx,cy-1) - c_val*(self.visibility_.get_value(cx,cy-1) - self.visibility_.get_value(cx+1,cy-1))
                self.visibility_.set_value(cx,cy, v * self.occupancy_complement_.get_value(cx,cy))
        # Q3
        for i in range(ls_x + 1):
            cx = ls_x - i
            for j_idx in range(ls_y + 1):
                cy = ls_y - j_idx; v = 0.0
                if not self._is_valid(cx,cy): continue
                if i == 0 and j_idx == 0: v = self.light_strength_
                elif i == 0: v = self.visibility_.get_value(cx,cy+1)
                elif j_idx == 0: v = self.visibility_.get_value(cx+1,cy)
                elif i == j_idx and i > 0: v = self.visibility_.get_value(cx+1,cy+1)
                elif i > j_idx:
                    dx=i; dy=j_idx; c_val=(dy+offset)/(dx+offset) if dx+offset!=0 else 0
                    v = self.visibility_.get_value(cx+1,cy) - c_val*(self.visibility_.get_value(cx+1,cy) - self.visibility_.get_value(cx+1,cy+1))
                elif j_idx > i:
                    dx=i; dy=j_idx; c_val=(dx+offset)/(dy+offset) if dy+offset!=0 else 0
                    v = self.visibility_.get_value(cx,cy+1) - c_val*(self.visibility_.get_value(cx,cy+1) - self.visibility_.get_value(cx+1,cy+1))
                self.visibility_.set_value(cx,cy, v * self.occupancy_complement_.get_value(cx,cy))
        # Q4
        for i in range(self.nx_ - ls_x):
            cx = ls_x + i
            for j_idx in range(ls_y + 1):
                cy = ls_y - j_idx; v = 0.0
                if not self._is_valid(cx,cy): continue
                if i == 0 and j_idx == 0: v = self.light_strength_
                elif i == 0: v = self.visibility_.get_value(cx,cy+1)
                elif j_idx == 0: v = self.visibility_.get_value(cx-1,cy)
                elif i == j_idx and i > 0: v = self.visibility_.get_value(cx-1,cy+1)
                elif i > j_idx:
                    dx=i; dy=j_idx; c_val=(dy+offset)/(dx+offset) if dx+offset!=0 else 0
                    v = self.visibility_.get_value(cx-1,cy) - c_val*(self.visibility_.get_value(cx-1,cy) - self.visibility_.get_value(cx-1,cy+1))
                elif j_idx > i:
                    dx=i; dy=j_idx; c_val=(dx+offset)/(dy+offset) if dy+offset!=0 else 0
                    v = self.visibility_.get_value(cx,cy+1) - c_val*(self.visibility_.get_value(cx,cy+1) - self.visibility_.get_value(cx-1,cy+1))
                self.visibility_.set_value(cx,cy, v * self.occupancy_complement_.get_value(cx,cy))

    def raycasting(self, x0: int, y0: int, x1: int, y1: int) -> bool: # Renamed for clarity
        dx=abs(x1-x0); dy=abs(y1-y0); sx=1 if x0<x1 else -1; sy=1 if y0<y1 else -1
        err=dx-dy; cx,cy=x0,y0
        while True:
            if not self._is_valid(cx,cy) or self.occupancy_complement_.get_value(cx,cy)==0.0:
                if self._is_valid(x1,y1): self.visibility_raycasting_.set_value(x1,y1,0.0)
                # if self._is_valid(cx,cy): self.visibility_raycasting_.set_value(cx,cy,0.0) # C++ also marks current
                return False
            if cx==x1 and cy==y1: break
            e2=2*err
            if e2 > -dy: err-=dy; cx+=sx
            if e2 < dx: err+=dx; cy+=sy
        if self._is_valid(x1,y1): self.visibility_raycasting_.set_value(x1,y1,1.0)
        return True

    def benchmark(self):
        if self.nx_==0 or self.ny_==0: return
        if not self.config_.silent: print("\n--- Benchmark ---")
        self.start_pt_=self.config_.start
        if not self._is_valid(self.start_pt_[0],self.start_pt_[1]) or self.occupancy_complement_.get_value(self.start_pt_[0],self.start_pt_[1])==0.0:
            if not self.config_.silent: print("Benchmark: Start invalid."); return
        self.ls_=self.start_pt_

        ts=time.perf_counter(); self._compute_visibility(); te=time.perf_counter(); dur_vis=(te-ts)*1e6
        if self.config_.save_results: self._save_visibility_image(self.visibility_,"standAloneVisibility_benchmark.png")

        self.visibility_raycasting_.reset(1.0); ts_rc=time.perf_counter()
        for r in range(self.ny_):
            for c in range(self.nx_):
                if self.occupancy_complement_.get_value(c,r)==1.0: self.raycasting(self.ls_[0],self.ls_[1],c,r)
        te_rc=time.perf_counter(); dur_rc=(te_rc-ts_rc)*1e6
        if self.config_.save_results: self._save_visibility_image(self.visibility_raycasting_,"rayCastingVisibility_benchmark.png")

        if not self.config_.silent:
            print(f"Vis time: {dur_vis:.0f}us, RC time: {dur_rc:.0f}us")
            if dur_vis>0 and dur_rc>0: print(f"Ratio (RC/Prop): {dur_rc/dur_vis:.2f}x faster.")
            density=np.sum(self.occupancy_complement_.data_==0.0)/self.size_*100 if self.size_>0 else 0
            print(f"Density: {density:.2f}%")

    def benchmark_series(self, output_dir:str="./output_py"):
        if not self.config_.silent: print("\n--- Benchmark Series ---")
        pts=5; sz_s=50; sz_e=250; log_pts=np.logspace(np.log10(sz_s),np.log10(sz_e),num=pts,dtype=int).tolist(); results=[]
        orig_nx,orig_ny=self.nx_,self.ny_; orig_occ=self.occupancy_complement_.data_.copy() if self.nx_>0 else None; orig_ls=self.ls_
        for sz in log_pts:
            if not self.config_.silent: print(f"\nBenchmarking: {sz}x{sz}")
            self.nx_=sz; self.ny_=sz; self.size_=sz*sz
            cur_occ=Field(sz,sz,1.0)
            if sz>10:
                n_obs=sz*sz//100; o_w=max(1,sz//20); o_h=max(1,sz//20)
                for _ in range(n_obs):
                    rs=random.randint(0,sz-o_h-1 if sz-o_h-1>=0 else 0); cs=random.randint(0,sz-o_w-1 if sz-o_w-1>=0 else 0)
                    for r_o in range(rs,rs+o_h):
                        for c_o in range(cs,cs+o_w):
                            if 0<=c_o<sz and 0<=r_o<sz : cur_occ.set_value(c_o,r_o,0.0) # Check bounds
            self.occupancy_complement_=cur_occ; self.visibility_.resize(sz,sz,0.0); self.visibility_raycasting_.resize(sz,sz,1.0)
            self.ls_=(sz//2,sz//2)

            t1=time.perf_counter();self._compute_visibility();t2=time.perf_counter();dv=(t2-t1)*1e6
            self.visibility_raycasting_.reset(1.0);t1r=time.perf_counter()
            for r_ in range(sz):
                for c_ in range(sz):
                    if self.occupancy_complement_.get_value(c_,r_)==1.0: self.raycasting(self.ls_[0],self.ls_[1],c_,r_)
            t2r=time.perf_counter();dr=(t2r-t1r)*1e6; ratio=dr/dv if dv>0 else float('inf')
            if not self.config_.silent: print(f"Size:{sz}x{sz},Vis:{dv:.0f}us,RC:{dr:.0f}us,Ratio:{ratio:.2f}")
            results.append((sz,dv,dr,ratio))
        self.nx_,self.ny_=orig_nx,orig_ny; self.size_=self.nx_*self.ny_
        if self.nx_>0 and self.ny_>0 and orig_occ is not None:
            self.occupancy_complement_=Field(self.nx_,self.ny_,0.0); self.occupancy_complement_.data_=orig_occ
            self.visibility_.resize(self.nx_,self.ny_,0.0); self.visibility_raycasting_.resize(self.nx_,self.ny_,1.0)
        self.ls_=orig_ls
        fp=os.path.join(output_dir,"benchmark_results_py.txt"); os.makedirs(output_dir,exist_ok=True)
        try:
            with open(fp,"w") as f:
                f.write("Size VisTimeUS RaycastTimeUS Ratio\n")
                for s,tv,tr,rt in results: f.write(f"{s}x{s} {tv:.0f} {tr:.0f} {rt:.4f}\n")
            if not self.config_.silent: print(f"Saved benchmark series to {fp}")
        except Exception as e: print(f"Error saving benchmark series: {e}")

# --- Main test section (conditional execution) ---
if __name__ == '__main__':
    print("Testing VisibilityBasedSolver (core logic + viz/benchmark)...")

    cfg_test_full = VBSConfig(
        mode=1, ncols=30, nrows=20, nb_of_obstacles=8,
        min_width=3, max_width=5, min_height=3, max_height=5,
        random_seed=False, seed_value=42,
        start=(2,2), end=(28,18),
        max_iter=50, visibility_threshold=0.3, light_strength=1.0,
        silent=False, timer=True,
        save_results=True,
        save_came_from=True, save_global_visibility=True, save_local_visibility=True,
        save_light_sources=True
    )

    print("\n--- Full Test 1: Random Environment (with saving) ---")
    env_test_full = Environment(cfg_test_full)

    if env_test_full.visibility_field.get_value(cfg_test_full.start[0], cfg_test_full.start[1]) == 0.0:
        env_test_full.visibility_field.set_value(cfg_test_full.start[0], cfg_test_full.start[1], 1.0)
    if env_test_full.visibility_field.get_value(cfg_test_full.end[0], cfg_test_full.end[1]) == 0.0:
        env_test_full.visibility_field.set_value(cfg_test_full.end[0], cfg_test_full.end[1], 1.0)

    if env_test_full.nx > 0 and env_test_full.ny > 0:
        solver_full = VisibilityBasedSolver(env_test_full)
        path_full = solver_full.solve()

        solver_full.stand_alone_visibility()
        solver_full.benchmark()
        solver_full.benchmark_series()
    else:
        print("Skipping full random environment solver test due to empty environment.")

    print("\nVisibilityBasedSolver full tests completed.")
