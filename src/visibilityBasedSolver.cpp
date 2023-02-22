#include "solver/visibilityBasedSolver.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

namespace vbs {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
visibilityBasedSolver::visibilityBasedSolver(environment& env) : config_(env.getConfig()) { 
  occupancyComplement_ = env.getVisibilityField();
  nrows_ = occupancyComplement_->rows();
  ncols_ = occupancyComplement_->cols();
  visibility_global_ = std::make_unique<DoubleField>(nrows_, ncols_, 0.0);
  visibility_ = std::make_unique<DoubleField>(nrows_, ncols_, 0.0);
  scale_ = sqrt(nrows_ * nrows_ + ncols_ * ncols_);

  lightSource_enum_ = std::make_unique<SizeField>(nrows_, ncols_, 1e5);
  lightSources_.reset(new std::pair<size_t, size_t>[nrows_ * ncols_]);
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
  auto start = config_->start;
  auto end = config_->end;
  ls_ = start;
  end_ = end;
  lightSources_.get()[nb_of_sources_] = start;
  lightSource_enum_->at(start.first, start.second) = nb_of_sources_;
  visibility_global_->at(end.first, end.second) = 0;
  max_iter_ = config_->max_iter;
  threshold_ = config_->threshold;

  while (visibility_global_->at(end.first, end.second) <= threshold_) {
    resetQueue();
    updateVisibility();
    auto node = heap_->top();
    ls_ = {node.x, node.y};
    ++nb_of_sources_;
    lightSources_.get()[nb_of_sources_] = ls_;
    if (nb_of_sources_ > max_iter_) {
      std::cout << "Max iters hit." << std::endl;
      break;
    }
  }
  lightSources_.get()[nb_of_sources_] = end;

  auto time_stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_stop - time_start);
  if (!config_->silent) {
    if (config_->timer) {
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

  visibility_->Reset();
  // visibility_->Fill(0.0);
  
  // Q1
  max_col_ = nrows_ - ls_.first;
  max_row_ = ncols_ - ls_.second;
  for (size_t i = 0; i < max_col_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_row_; ++j) {
      currentY = ls_.second + j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_->at(currentX, currentY-1);
      } else if (j == 0) {
        v = visibility_->at(currentX-1, currentY);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second) / (currentX - ls_.first);
        v = visibility_->at(currentX-1, currentY) - c_ * (visibility_->at(currentX-1, currentY) - visibility_->at(currentX-1, currentY-1));
      } else if (j > i) {
        c_ = (double)(currentX - ls_.first) / (currentY - ls_.second);
        v = visibility_->at(currentX, currentY-1) - c_ * (visibility_->at(currentX, currentY-1) - visibility_->at(currentX-1, currentY-1));
      }
      v = v * occupancyComplement_->at(currentX, currentY);
      visibility_->at(currentX, currentY) = v;
      visibility_global_->at(currentX, currentY) = std::max(v, visibility_global_->at(currentX,currentY));
      if (v >= threshold_) {
        if (lightSource_enum_->at(currentX, currentY) == 1e5) {
          lightSource_enum_->at(currentX, currentY) = nb_of_sources_; 
        }
      }
      if (visibility_global_->at(currentX, currentY) >= threshold_) {
        parent = lightSources_.get()[lightSource_enum_->at(currentX, currentY)];
        h = (scale_ * visibility_global_->at(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
         eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q2
  max_col_ = ls_.first;
  max_row_ = ncols_ - ls_.second;
  for (size_t i = 0; i < max_col_; ++i) {
    currentX = ls_.first - i;
    for (size_t j = 0; j < max_row_; ++j) {
      currentY = ls_.second + j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_->at(currentX, currentY-1);
      } else if (j == 0) {
        v = visibility_->at(currentX+1, currentY);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second) / (ls_.first - currentX);
        v = visibility_->at(currentX+1, currentY) - c_ * (visibility_->at(currentX+1, currentY) - visibility_->at(currentX+1, currentY-1));
      } else if (j > i) {
        c_ = (double)(ls_.first - currentX) / (currentY - ls_.second);
        v = visibility_->at(currentX, currentY-1) - c_ * (visibility_->at(currentX, currentY-1) - visibility_->at(currentX+1, currentY-1));
      }
      v = v * occupancyComplement_->at(currentX, currentY);
      visibility_->at(currentX, currentY) = v;
      visibility_global_->at(currentX, currentY) = std::max(v, visibility_global_->at(currentX,currentY));
      if (v >= threshold_) {
        if (lightSource_enum_->at(currentX, currentY) == 1e5) {
          lightSource_enum_->at(currentX, currentY) = nb_of_sources_;
        }
      }
      if (visibility_global_->at(currentX, currentY) >= threshold_) {
        parent = lightSources_.get()[lightSource_enum_->at(currentX, currentY)];
        h = (scale_ * visibility_global_->at(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
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
        v = visibility_->at(currentX, currentY+1);
      } else if (j == 0) {
        v = visibility_->at(currentX+1, currentY);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY) / (ls_.first - currentX);
        v = visibility_->at(currentX+1, currentY) - c_ * (visibility_->at(currentX+1, currentY) - visibility_->at(currentX+1, currentY+1));
      } else if (j > i) {
        c_ = (double)(ls_.first - currentX) / (ls_.second - currentY);
        v = visibility_->at(currentX, currentY+1) - c_ * (visibility_->at(currentX, currentY+1) - visibility_->at(currentX+1, currentY+1));
      }
      v = v * occupancyComplement_->at(currentX, currentY);
      visibility_->at(currentX, currentY) = v;
      visibility_global_->at(currentX, currentY) = std::max(v, visibility_global_->at(currentX,currentY));
      if (v >= threshold_) {
        if (lightSource_enum_->at(currentX, currentY) == 1e5) {
          lightSource_enum_->at(currentX, currentY) = nb_of_sources_;
        }
      }
      if (visibility_global_->at(currentX, currentY) >= threshold_) {
        parent = lightSources_.get()[lightSource_enum_->at(currentX, currentY)];
        h = (scale_ * visibility_global_->at(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
         eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q4
  max_col_ = nrows_ - ls_.first;
  max_row_ = ls_.second;
  for (size_t i = 0; i < max_col_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_row_; ++j) {
      currentY = ls_.second - j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_->at(currentX, currentY+1);
      } else if (j == 0) {
        v = visibility_->at(currentX-1, currentY);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY) / (currentX - ls_.first);
        v = visibility_->at(currentX-1, currentY) - c_ * (visibility_->at(currentX-1, currentY) - visibility_->at(currentX-1, currentY+1));
      } else if (j > i) {
        c_ = (double)(currentX - ls_.first) / (ls_.second - currentY);
        v = visibility_->at(currentX, currentY+1) - c_ * (visibility_->at(currentX, currentY+1) - visibility_->at(currentX-1, currentY+1));
      }
      v = v * occupancyComplement_->at(currentX, currentY);
      visibility_->at(currentX, currentY) = v;
      visibility_global_->at(currentX, currentY) = std::max(v, visibility_global_->at(currentX,currentY));
      if (v >= threshold_) {
        if (lightSource_enum_->at(currentX, currentY) == 1e5) {
          lightSource_enum_->at(currentX, currentY) = nb_of_sources_;
        }
      }
      if (visibility_global_->at(currentX, currentY) >= threshold_) {
        parent = lightSources_.get()[lightSource_enum_->at(currentX, currentY)];
        h = (scale_ * visibility_global_->at(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
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
  if (config_->saveLightSourceEnum) {
    std::fstream of("./output/LightSourceEnum.txt", std::ios::out | std::ios::trunc);
    if (of.is_open()) {
      std::ostream& os = of;
      for (int i = 0; i < nrows_; ++i) { 
        for (int j = 0; j < ncols_; ++j) {
          os << lightSource_enum_->at(i,j) << " "; 
        }
        os<< "\n";
      }
      of.close();
    }
    if (!config_->silent) {
      std::cout << "Saved LightSourceEnum" << std::endl;
    }
  }
  if (config_->saveLightSources) {
    std::fstream of("./output/lightSources.txt", std::ios::out | std::ios::trunc);
    if (of.is_open()) {
      std::ostream& os = of;
      for (int i = 0; i < nb_of_sources_; ++i) { 
        os << lightSources_.get()[i].first << " " << lightSources_.get()[i].second ; 
        os<< "\n";
      }
      of.close();
    }
    if (!config_->silent) {
      std::cout << "Saved lightSources" << std::endl;
    }
  }
  if (config_->saveGlobalVisibility) {
    std::fstream of("./output/VisibilityMap.txt", std::ios::out | std::ios::trunc);
    if (of.is_open()) {
      std::ostream& os = of;
      for (size_t i = 0; i < nrows_; ++i) { 
        for (size_t j = 0; j < ncols_; ++j) {
          os << visibility_global_->at(i, j) << " "; 
        }
        os << "\n";
      }
      of.close();
    }
    if (!config_->silent) {
      std::cout << "Saved GlobalVisibility" << std::endl;
    }
  }
  if (config_->saveLocalVisibility) {
    std::fstream of("./output/LocalVisibilityMap.txt", std::ios::out | std::ios::trunc);
    if (of.is_open()) {
      std::ostream& os = of;
      for (size_t i = 0; i < nrows_; ++i) { 
        for (size_t j = 0; j < ncols_; ++j) {
          os << visibility_->at(i, j) << " "; 
        }
        os << "\n";
      }
      of.close();
    }
    if (!config_->silent) {
      std::cout << "Saved LocalVisibility" << std::endl;
    }
  }
  if (config_->saveVisibilityMapEnv) {
    std::fstream of("./output/VisibilityMap_env.txt", std::ios::out | std::ios::trunc);
    if (of.is_open()) {
      std::ostream& os = of;
      for (size_t i = 0; i < nrows_; ++i) { 
        for (size_t j = 0; j < ncols_; ++j) {
          os << occupancyComplement_->at(i, j) << " "; 
        }
        os << "\n";
      }
      of.close();
    }
    if (!config_->silent) {
      std::cout << "Saved OccupancyComplement" << std::endl;
    }
  }
}

} // namespace vbs