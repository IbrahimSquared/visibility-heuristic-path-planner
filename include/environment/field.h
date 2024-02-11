#ifndef FIELD_H
#define FIELD_H

#include <cstddef>
#include <memory>

namespace vbs {

template <typename T> class Field {

public:
  using size_t = std::size_t;
  Field() : nx_(0), ny_(0), size_(0), data_(nullptr) {}

  explicit Field(const size_t nx, const size_t ny, const T default_value)
      : nx_(nx), ny_(ny), size_(nx * ny) {
    data_ = std::make_unique<T[]>(size_);
    for (size_t i = 0; i < size_; ++i) {
      data_[i] = default_value;
    }
  }

  void set(const size_t x, const size_t y, const T value) {
    data_[x + y * nx_] = value;
  }
  T get(const size_t x, const size_t y) const { return data_[x + y * nx_]; }
  size_t nx() const { return nx_; }
  size_t ny() const { return ny_; }
  size_t size() const { return size_; }

  void reset() { data_ = std::make_unique<T[]>(size_); }

  void resize(const size_t nx, const size_t ny, const T default_value) {
    nx_ = nx;
    ny_ = ny;
    size_ = nx * ny;
    data_ = std::make_unique<T[]>(size_);
    for (size_t i = 0; i < size_; ++i) {
      data_[i] = default_value;
    }
  }

private:
  size_t nx_;
  size_t ny_;
  size_t size_;
  std::unique_ptr<T[]> data_;
};

} // namespace vbs

#endif // FIELD_H
