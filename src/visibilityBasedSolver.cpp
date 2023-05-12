#include "solver/visibilityBasedSolver.h"

#include <iostream>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <chrono>

namespace vbs {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
visibilityBasedSolver::visibilityBasedSolver(environment& env) : sharedConfig_(env.getConfig()) { 
  occupancyComplement_ = env.getVisibilityField();
  ncols_ = occupancyComplement_.ncols();
  nrows_ = occupancyComplement_.nrows();
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;
  
  // Init maps
  reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void visibilityBasedSolver::reset() {
  visibility_global_ = Field<double, 0>(ncols_, nrows_, 0.0);
  visibility_ = Field<double, 0>(ncols_, nrows_, 0.0);
  cameFrom_ = Field<size_t, 0>(ncols_, nrows_, 1e5);
  lightSources_.reset(new point[ncols_ * nrows_]);
  scale_ = sqrt(nrows_ * nrows_ + ncols_ * ncols_);
  heap_.reset();

  // Reserve heap_
  std::vector<Node> container;
  container.reserve(ncols_*nrows_);
  std::priority_queue<Node, std::vector<Node>, std::less<Node>> heap(std::less<Node>(), std::move(container));
  heap_ = std::make_unique<std::priority_queue<Node>>(heap); 

  nb_of_sources_ = 0;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void visibilityBasedSolver::resetQueue() {
  std::vector<Node> container;
  container.reserve(nrows_*ncols_);
  std::priority_queue<Node, std::vector<Node>, std::less<Node>> heap(std::less<Node>(), std::move(container));
  heap_ = std::make_unique<std::priority_queue<Node>>(heap);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void visibilityBasedSolver::solve() {
  auto time_start = std::chrono::high_resolution_clock::now();

  // Init
  auto start = sharedConfig_->start;
  auto end = sharedConfig_->end;

  if (sharedConfig_->mode == 2) {
    start.second = nrows_ - 1 - start.second;
    end.second = nrows_ - 1 - end.second;
  }

  ls_ = start;
  end_ = end;

  lightSources_[nb_of_sources_] = start;
  cameFrom_(start.first, start.second) = nb_of_sources_;
  visibility_global_(end.first, end.second) = 0;
  max_iter_ = sharedConfig_->max_iter;
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  

  while (visibility_global_(end.first, end.second) <= visibilityThreshold_) {
    resetQueue();
    updateVisibility();
    auto node = heap_->top();
    ls_ = {node.x, node.y};
    ++nb_of_sources_;
    lightSources_[nb_of_sources_] = ls_;
    if (nb_of_sources_ > max_iter_) {
      std::cout << "Max iters hit." << std::endl;
      break;
    }
  }
  lightSources_[nb_of_sources_] = end;

  auto time_stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_stop - time_start);
  if (!sharedConfig_->silent) {
    if (sharedConfig_->timer) {
      std::cout << "############################## Solver output ##############################" << "\n"
        << "Execution time in us: " << duration.count() << "us" << std::endl;
    }
  }
  saveResults();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void visibilityBasedSolver::updateVisibility() {
  size_t currentX, currentY;
  double v = 0.0;
  double h;
  point parent;
  double offset = 1.0;

  visibility_.reset();
  
  // Q1
  max_col_ = ncols_ - ls_.first;
  max_row_ = nrows_ - ls_.second;
  
  // std::cout << max_col_ << ", " << max_row_ << std::endl;
  
  for (size_t i = 0; i < max_col_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_row_; ++j) {
      currentY = ls_.second + j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY-1);
      } else if (j == 0) {
        v = visibility_(currentX-1, currentY);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second + offset) / (currentX - ls_.first + offset);
        v = visibility_(currentX-1, currentY) - c_ * (visibility_(currentX-1, currentY) - visibility_(currentX-1, currentY-1));
      } else if (j > i) {
        c_ = (double)(currentX - ls_.first + offset) / (currentY - ls_.second + offset);
        v = visibility_(currentX, currentY-1) - c_ * (visibility_(currentX, currentY-1) - visibility_(currentX-1, currentY-1));
      }
      v = v * occupancyComplement_(currentX, currentY);
      visibility_(currentX, currentY) = v;
      visibility_global_(currentX, currentY) = std::max(v, visibility_global_(currentX,currentY));
      if (v >= visibilityThreshold_) {
        if (cameFrom_(currentX, currentY) == 1e5) {
          cameFrom_(currentX, currentY) = nb_of_sources_; 
        }
      }
      if (visibility_global_(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_(currentX, currentY)];
        h = (scale_ * visibility_global_(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
         eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q2
  max_col_ = ls_.first;
  max_row_ = nrows_ - ls_.second;
  for (size_t i = 0; i < max_col_; ++i) {
    currentX = ls_.first - i;
    for (size_t j = 0; j < max_row_; ++j) {
      currentY = ls_.second + j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY-1);
      } else if (j == 0) {
        v = visibility_(currentX+1, currentY);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second + offset) / (ls_.first - currentX + offset);
        v = visibility_(currentX+1, currentY) - c_ * (visibility_(currentX+1, currentY) - visibility_(currentX+1, currentY-1));
      } else if (j > i) {
        c_ = (double)(ls_.first - currentX + offset) / (currentY - ls_.second + offset);
        v = visibility_(currentX, currentY-1) - c_ * (visibility_(currentX, currentY-1) - visibility_(currentX+1, currentY-1));
      }
      v = v * occupancyComplement_(currentX, currentY);
      visibility_(currentX, currentY) = v;
      visibility_global_(currentX, currentY) = std::max(v, visibility_global_(currentX,currentY));
      if (v >= visibilityThreshold_) {
        if (cameFrom_(currentX, currentY) == 1e5) {
          cameFrom_(currentX, currentY) = nb_of_sources_;
        }
      }
      if (visibility_global_(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_(currentX, currentY)];
        h = (scale_ * visibility_global_(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
         eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q3
  max_col_ = ls_.first;
  max_row_ = ls_.second;
  for (size_t i = 0; i < max_col_; ++i) {
    currentX = ls_.first - i;
    for (size_t j = 0; j < max_row_; ++j) {
      currentY = ls_.second - j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY+1);
      } else if (j == 0) {
        v = visibility_(currentX+1, currentY);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY + offset) / (ls_.first - currentX + offset);
        v = visibility_(currentX+1, currentY) - c_ * (visibility_(currentX+1, currentY) - visibility_(currentX+1, currentY+1));
      } else if (j > i) {
        c_ = (double)(ls_.first - currentX + offset) / (ls_.second - currentY + offset);
        v = visibility_(currentX, currentY+1) - c_ * (visibility_(currentX, currentY+1) - visibility_(currentX+1, currentY+1));
      }
      v = v * occupancyComplement_(currentX, currentY);
      visibility_(currentX, currentY) = v;
      visibility_global_(currentX, currentY) = std::max(v, visibility_global_(currentX,currentY));
      if (v >= visibilityThreshold_) {
        if (cameFrom_(currentX, currentY) == 1e5) {
          cameFrom_(currentX, currentY) = nb_of_sources_;
        }
      }
      if (visibility_global_(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_(currentX, currentY)];
        h = (scale_ * visibility_global_(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
         eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q4
  max_col_ = ncols_ - ls_.first;
  max_row_ = ls_.second;
  for (size_t i = 0; i < max_col_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_row_; ++j) {
      currentY = ls_.second - j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_(currentX, currentY+1);
      } else if (j == 0) {
        v = visibility_(currentX-1, currentY);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY + offset) / (currentX - ls_.first + offset);
        v = visibility_(currentX-1, currentY) - c_ * (visibility_(currentX-1, currentY) - visibility_(currentX-1, currentY+1));
      } else if (j > i) {
        c_ = (double)(currentX - ls_.first + offset) / (ls_.second - currentY + offset);
        v = visibility_(currentX, currentY+1) - c_ * (visibility_(currentX, currentY+1) - visibility_(currentX-1, currentY+1));
      }
      v = v * occupancyComplement_(currentX, currentY);
      visibility_(currentX, currentY) = v;
      visibility_global_(currentX, currentY) = std::max(v, visibility_global_(currentX,currentY));
      if (v >= visibilityThreshold_) {
        if (cameFrom_(currentX, currentY) == 1e5) {
          cameFrom_(currentX, currentY) = nb_of_sources_;
        }
      }
      if (visibility_global_(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_(currentX, currentY)];
        h = (scale_ * visibility_global_(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
         eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void visibilityBasedSolver::saveResults() {
  namespace fs = std::filesystem;
  // Define the path to the output file
  std::string path = "./output/cameFrom.txt";
  // Check if the directory exists, and create it if it doesn't
  fs::path dir = fs::path(path).parent_path();
  if (!fs::exists(dir)) {
    if (!fs::create_directories(dir)) {
      std::cerr << "Failed to create directory " << dir.string() << std::endl;
      return;
    }
  }

  if (sharedConfig_->saveCameFrom) {
    std::fstream of(path, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << path << std::endl;
      return;
    }
    if (of.is_open()) {
      std::ostream& os = of;
      if (sharedConfig_->mode == 2) {
        for (int j = nrows_ - 1; j >= 0; --j){
          for (size_t i = 0; i < ncols_; ++i) {
            os << cameFrom_(i,j) << " "; 
          }
          os << "\n";
        }
      } else {
        for (int j = 0; j < nrows_; ++j){
          for (size_t i = 0; i < ncols_; ++i) {
            os << cameFrom_(i,j) << " "; 
          }
          os << "\n";
        }
      }
      of.close();
    }
    if (!sharedConfig_->silent) {
      std::cout << "Saved cameFrom_" << std::endl;
    }
  }
  if (sharedConfig_->saveLightSources) {
    path = "./output/lightSources.txt";
    std::fstream of(path, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << path << std::endl;
      return;
    }
    if (of.is_open()) {
      std::ostream& os = of;
      for (int i = 0; i < nb_of_sources_; ++i) { 
        if (sharedConfig_->mode == 2) {
          os << lightSources_[i].first << " " << nrows_ - 1 - lightSources_[i].second; 
        } else {
          os << lightSources_[i].first << " " << lightSources_[i].second; 
        }
        os << "\n";
      }
      of.close();
    }
    if (!sharedConfig_->silent) {
      std::cout << "Saved lightSources" << std::endl;
    }
  }
  if (sharedConfig_->saveGlobalVisibility) {
    path = "./output/VisibilityMap.txt";
    std::fstream of(path, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << path << std::endl;
      return;
    }
    if (of.is_open()) {
      std::ostream& os = of;
      if (sharedConfig_->mode == 2) {
        for (int j = nrows_ - 1; j >= 0; --j){
          for (size_t i = 0; i < ncols_; ++i) {
            os << visibility_global_(i, j) << " "; 
          }
          os << "\n";
        }
      } else {
        for (int j = 0; j < nrows_; ++j){
          for (size_t i = 0; i < ncols_; ++i) {
            os << visibility_global_(i, j) << " "; 
          }
          os << "\n";
        }
      }
      of.close();
    }
    if (!sharedConfig_->silent) {
      std::cout << "Saved GlobalVisibility" << std::endl;
    }
  }
  if (sharedConfig_->saveLocalVisibility) {
    path = "./output/LocalVisibilityMap.txt";
    std::fstream of(path, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << path << std::endl;
      return;
    }
    if (of.is_open()) {
      std::ostream& os = of;
      if (sharedConfig_->mode == 2) {
        for (int j = nrows_ - 1; j >= 0; --j){
          for (size_t i = 0; i < ncols_; ++i) {
            os << visibility_(i, j) << " "; 
          }
          os << "\n";
        }
      } else {
        for (int j = 0; j < nrows_; ++j){
          for (size_t i = 0; i < ncols_; ++i) {
            os << visibility_(i, j) << " "; 
          }
          os << "\n";
        }
      }
      of.close();
    }
    if (!sharedConfig_->silent) {
      std::cout << "Saved LocalVisibility" << std::endl;
    }
  }
  if (sharedConfig_->saveVisibilityField) {
    path = "./output/visibilityField.txt";
    std::fstream of(path, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << path << std::endl;
      return;
    }
    if (of.is_open()) {
      std::ostream& os = of;
      if (sharedConfig_->mode == 2) {
        for (int j = nrows_ - 1; j >= 0; --j){
          for (size_t i = 0; i < ncols_; ++i) {
            os << occupancyComplement_(i, j) << " "; 
          }
          os << "\n";
        }
      } else {
        for (int j = 0; j < nrows_; ++j){
          for (size_t i = 0; i < ncols_; ++i) {
            os << occupancyComplement_(i, j) << " "; 
          }
          os << "\n";
        }
      }
      of.close();
    }
    if (!sharedConfig_->silent) {
      std::cout << "Saved OccupancyComplement" << std::endl;
    }
  }
}

} // namespace vbs