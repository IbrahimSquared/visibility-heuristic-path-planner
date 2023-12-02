#ifndef FIELD_H
#define FIELD_H

#include <cstddef>
#include <memory>

namespace vbs {

template <typename T>
class Field {
  
  public:
    using size_t = std::size_t;
    Field() : nx_(0), ny_(0), size_(0), data_(nullptr) {}

    Field(size_t nx, size_t ny, T default_value) : nx_(nx), ny_(ny), size_(nx * ny) {
      data_ = std::make_unique<T[]>(size_);
      for (size_t i = 0; i < size_; ++i) {
        data_[i] = default_value;
      }
    }

    void set(size_t x, size_t y, T value) { data_[x + y * nx_] = value; }
    T get(size_t x, size_t y) const { return data_[x + y * nx_]; }
    size_t nx() const { return nx_; }
    size_t ny() const { return ny_; }
    size_t size() const { return size_; }

    void reset() { 
      data_ = std::make_unique<T[]>(size_);
    }

  private:
    size_t nx_;
    size_t ny_;
    size_t size_;
    std::unique_ptr<T[]> data_;
};

} // namespace vbs

#endif // FIELD_H