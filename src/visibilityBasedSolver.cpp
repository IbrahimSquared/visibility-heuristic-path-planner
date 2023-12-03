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
visibilityBasedSolver::visibilityBasedSolver(environment& env) 
  : sharedConfig_(env.getConfig()), 
  occupancyComplement_(env.getVisibilityField()) { 
  nx_ = occupancyComplement_->nx();
  ny_ = occupancyComplement_->ny();
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;
  
  // Init environment image
  uniqueLoadedImage_.reset(std::make_unique<sf::Image>().release());
  uniqueLoadedImage_->create(nx_, ny_, sf::Color::Black);
  sf::Color color;
  color.a = 1;
  for (size_t j = ny_ - 1; j > 0; --j) {
    for (size_t i = 0; i < nx_; ++i) {
      if (occupancyComplement_->get(i,j) < 1) {
        uniqueLoadedImage_->setPixel(i, ny_ - 1 - j, color.Black);
      } else {
        uniqueLoadedImage_->setPixel(i, ny_ - 1 - j, color.White);
      }
    }
  }
  
  // Init maps
  reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void visibilityBasedSolver::reset() {
  visibility_global_ = std::make_unique<Field<double>>(nx_, ny_, 0.0);
  visibility_ = std::make_unique<Field<double>>(nx_, ny_, 0.0);
  cameFrom_ = std::make_unique<Field<size_t>>(nx_, ny_, 1e15);

  lightSources_.reset(new point[nx_ * ny_]);
  scale_ = sqrt(ny_ * ny_ + nx_ * nx_);
  heap_.reset();

  // Reserve heap_
  std::vector<Node> container;
  container.reserve(nx_*ny_);
  std::priority_queue<Node, std::vector<Node>, std::less<Node>> heap(std::less<Node>(), std::move(container));
  heap_ = std::make_unique<std::priority_queue<Node>>(heap); 

  nb_of_sources_ = 0;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void visibilityBasedSolver::resetQueue() {
  std::vector<Node> container;
  container.reserve(ny_*nx_);
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
    start.second = ny_ - 1 - start.second;
    end.second = ny_ - 1 - end.second;
  }

  // check if start and end are valid
  if (start.first < 0 || start.first >= nx_ || start.second < 0 || start.second >= ny_) {
    std::cout << "Start point is out of bounds." << std::endl;
    return;
  }
  if (end.first < 0 || end.first >= nx_ || end.second < 0 || end.second >= ny_) {
    std::cout << "End point is out of bounds." << std::endl;
    return;
  }
  if (occupancyComplement_->get(start.first, start.second) == 0) {
    std::cout << "Start point is not valid." << std::endl;
    return;
  }
  if (occupancyComplement_->get(end.first, end.second) == 0) {
    std::cout << "End point is not valid." << std::endl;
    return;
  }

  ls_ = start;
  end_ = end;

  lightSources_[nb_of_sources_] = start;
  cameFrom_->set(start.first, start.second, nb_of_sources_);
  visibility_global_->set(end.first, end.second, 0);
  max_iter_ = sharedConfig_->max_iter;
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  while (visibility_global_->get(end.first, end.second) <= visibilityThreshold_) {
    resetQueue();
    updateVisibility();
    auto node = heap_->top();
    ls_ = {node.x, node.y};
    ++nb_of_sources_;
    lightSources_[nb_of_sources_] = ls_;
    if (nb_of_sources_ > max_iter_) {
      std::cout << "Max iters hit. Solution could not be found. Try lowering visibility threshold." << std::endl;
      return;
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
  std::vector<point> path;
  reconstructPath(Node{static_cast<size_t>(end_.first), static_cast<size_t>(end_.second), 0}, path);
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

  visibility_->reset();
  
  // Q1
  max_x_ = nx_ - ls_.first;
  max_y_ = ny_ - ls_.second;

  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second + j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_->get(currentX, currentY-1);
      } else if (j == 0) {
        v = visibility_->get(currentX-1, currentY);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second + offset) / (currentX - ls_.first + offset);
        v = visibility_->get(currentX-1, currentY) - c_ * (visibility_->get(currentX-1, currentY) - visibility_->get(currentX-1, currentY-1));
      } else if (j > i) {
        c_ = (double)(currentX - ls_.first + offset) / (currentY - ls_.second + offset);
        v = visibility_->get(currentX, currentY-1) - c_ * (visibility_->get(currentX, currentY-1) - visibility_->get(currentX-1, currentY-1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_->set(currentX, currentY, v);
      visibility_global_->set(currentX, currentY, std::max(v, visibility_global_->get(currentX,currentY)));
      if (v >= visibilityThreshold_) {
        if (cameFrom_->get(currentX, currentY) == 1e15) {
          cameFrom_->set(currentX, currentY, nb_of_sources_); 
        }
      }
      if (visibility_global_->get(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_->get(currentX, currentY)];
        h = (scale_ * visibility_global_->get(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
         eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q2
  max_x_ = ls_.first;
  max_y_ = ny_ - ls_.second;
  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first - i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second + j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_->get(currentX, currentY-1);
      } else if (j == 0) {
        v = visibility_->get(currentX+1, currentY);
      } else if (i > j) {
        c_ = (double)(currentY - ls_.second + offset) / (ls_.first - currentX + offset);
        v = visibility_->get(currentX+1, currentY) - c_ * (visibility_->get(currentX+1, currentY) - visibility_->get(currentX+1, currentY-1));
      } else if (j > i) {
        c_ = (double)(ls_.first - currentX + offset) / (currentY - ls_.second + offset);
        v = visibility_->get(currentX, currentY-1) - c_ * (visibility_->get(currentX, currentY-1) - visibility_->get(currentX+1, currentY-1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_->set(currentX, currentY, v);
      visibility_global_->set(currentX, currentY, std::max(v, visibility_global_->get(currentX,currentY)));
      if (v >= visibilityThreshold_) {
        if (cameFrom_->get(currentX, currentY) == 1e15) {
          cameFrom_->set(currentX, currentY, nb_of_sources_);
        }
      }
      if (visibility_global_->get(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_->get(currentX, currentY)];
        h = (scale_ * visibility_global_->get(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
         eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q3
  max_x_ = ls_.first;
  max_y_ = ls_.second;
  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first - i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second - j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_->get(currentX, currentY+1);
      } else if (j == 0) {
        v = visibility_->get(currentX+1, currentY);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY + offset) / (ls_.first - currentX + offset);
        v = visibility_->get(currentX+1, currentY) - c_ * (visibility_->get(currentX+1, currentY) - visibility_->get(currentX+1, currentY+1));
      } else if (j > i) {
        c_ = (double)(ls_.first - currentX + offset) / (ls_.second - currentY + offset);
        v = visibility_->get(currentX, currentY+1) - c_ * (visibility_->get(currentX, currentY+1) - visibility_->get(currentX+1, currentY+1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_->set(currentX, currentY, v);
      visibility_global_->set(currentX, currentY, std::max(v, visibility_global_->get(currentX,currentY)));
      if (v >= visibilityThreshold_) {
        if (cameFrom_->get(currentX, currentY) == 1e15) {
          cameFrom_->set(currentX, currentY, nb_of_sources_);
        }
      }
      if (visibility_global_->get(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_->get(currentX, currentY)];
        h = (scale_ * visibility_global_->get(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
         eval_d(currentX, currentY, parent.first, parent.second));
        heap_->push(Node{currentX, currentY, h});
      }
    }
  }
  // Q4
  max_x_ = nx_ - ls_.first;
  max_y_ = ls_.second;
  for (size_t i = 0; i < max_x_; ++i) {
    currentX = ls_.first + i;
    for (size_t j = 0; j < max_y_; ++j) {
      currentY = ls_.second - j;
      if (i == 0 && j == 0) {
        v = lightStrength_;
      } else if (i == 0) {
        v = visibility_->get(currentX, currentY+1);
      } else if (j == 0) {
        v = visibility_->get(currentX-1, currentY);
      } else if (i > j) {
        c_ = (double)(ls_.second - currentY + offset) / (currentX - ls_.first + offset);
        v = visibility_->get(currentX-1, currentY) - c_ * (visibility_->get(currentX-1, currentY) - visibility_->get(currentX-1, currentY+1));
      } else if (j > i) {
        c_ = (double)(currentX - ls_.first + offset) / (ls_.second - currentY + offset);
        v = visibility_->get(currentX, currentY+1) - c_ * (visibility_->get(currentX, currentY+1) - visibility_->get(currentX-1, currentY+1));
      }
      v = v * occupancyComplement_->get(currentX, currentY);
      visibility_->set(currentX, currentY, v);
      visibility_global_->set(currentX, currentY, std::max(v, visibility_global_->get(currentX,currentY)));
      if (v >= visibilityThreshold_) {
        if (cameFrom_->get(currentX, currentY) == 1e15) {
          cameFrom_->set(currentX, currentY, nb_of_sources_);
        }
      }
      if (visibility_global_->get(currentX, currentY) >= visibilityThreshold_) {
        parent = lightSources_[cameFrom_->get(currentX, currentY)];
        h = (scale_ * visibility_global_->get(currentX, currentY)) + (eval_d(currentX, currentY, end_.first, end_.second) +
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
        for (int j = ny_ - 1; j >= 0; --j){
          for (size_t i = 0; i < nx_; ++i) {
            os << cameFrom_->get(i,j) << " "; 
          }
          os << "\n";
        }
      } else {
        for (int j = 0; j < ny_; ++j){
          for (size_t i = 0; i < nx_; ++i) {
            os << cameFrom_->get(i,j) << " "; 
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
          os << lightSources_[i].first << " " << ny_ - 1 - lightSources_[i].second; 
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
        for (int j = ny_ - 1; j >= 0; --j){
          for (size_t i = 0; i < nx_; ++i) {
            os << visibility_global_->get(i, j) << " "; 
          }
          os << "\n";
        }
      } else {
        for (int j = 0; j < ny_; ++j){
          for (size_t i = 0; i < nx_; ++i) {
            os << visibility_global_->get(i, j) << " "; 
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
        for (int j = ny_ - 1; j >= 0; --j){
          for (size_t i = 0; i < nx_; ++i) {
            os << visibility_->get(i, j) << " "; 
          }
          os << "\n";
        }
      } else {
        for (int j = 0; j < ny_; ++j){
          for (size_t i = 0; i < nx_; ++i) {
            os << visibility_->get(i, j) << " "; 
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
        for (int j = ny_ - 1; j >= 0; --j){
          for (size_t i = 0; i < nx_; ++i) {
            os << occupancyComplement_->get(i, j) << " "; 
          }
          os << "\n";
        }
      } else {
        for (int j = 0; j < ny_; ++j){
          for (size_t i = 0; i < nx_; ++i) {
            os << occupancyComplement_->get(i, j) << " "; 
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void visibilityBasedSolver::reconstructPath(const Node& current, std::vector<point>& resultingPath) {
  int x = current.x, y = current.y;
  double t = cameFrom_->get(x, y);
  double t_old = INFINITY;
  while (t != t_old) {
    resultingPath.push_back({x, y});
    t_old = t;
    x = lightSources_[t].first; y = lightSources_[t].second;
    t = cameFrom_->get(x, y);
  }
  resultingPath.push_back({x, y});
  std::reverse(resultingPath.begin(), resultingPath.end());
  
  // compute total distance
  double totalDistance = 0;
  for (size_t i = 0; i < resultingPath.size() - 1; ++i) {
    totalDistance += eval_d(resultingPath[i].first, resultingPath[i].second, resultingPath[i+1].first, resultingPath[i+1].second);
  }
  if (!sharedConfig_->silent) {
    std::cout << "Path length: " << totalDistance << std::endl;
  }

  if (sharedConfig_->saveResults) {
    saveImageWithPath(resultingPath);
  }
  return;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void visibilityBasedSolver::saveImageWithPath(const std::vector<point>& path) {
  sf::Image image;
  image = *uniqueLoadedImage_;
  sf::Color color;
  color.a = 1;
  int x0, y0, x1, y1;
  int dx, dy, sx, sy, err;
  
  for (size_t i = 0; i < path.size() - 1; ++i) {
    x0 = path[i].first; y0 = ny_ - 1 - path[i].second;
    x1 = path[i+1].first; y1 = ny_ - 1 - path[i+1].second;

    dx = std::abs(x1 - x0);
    dy = std::abs(y1 - y0);
    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;
    err = dx - dy;
    
    while (x0 != x1 || y0 != y1) {
      image.setPixel(x0, y0, color.Magenta);
      int e2 = 2 * err;
      if (e2 > -dy) {
          err -= dy;
          x0 += sx;
      }
      if (e2 < dx) {
          err += dx;
          y0 += sy;
      }
    }
  }
  std::string imageName = "output/ResultingPath.png";
  image.saveToFile(imageName);
}

} // namespace vbs