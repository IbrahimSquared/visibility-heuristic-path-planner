#ifndef FIELD_H
#define FIELD_H

#include <cstddef>
#include <memory>

namespace vbs {

template<typename T, int unique>
class Field {
 public:
  using size_t = std::size_t;
  // Default constructor
  Field() : ncols_(0), nrows_(0), size_(0), data_(nullptr) {}

  // Constructor with specified dimensions and initial value
  Field(size_t ncols, size_t nrows, T default_value) : ncols_{ncols}, nrows_{nrows}, size_{ncols * nrows} {
    if constexpr (unique == 0)
      data_ = std::make_unique<T[]>(size_);
    else
      data_ = std::make_shared<T[]>(size_);
    fill(default_value);
  }

  // Copy constructor
  Field(const Field& other)
    : ncols_(other.ncols_),
      nrows_(other.nrows_),
      size_(other.size_),
      data_(other.data_) // Copy the ptr instead of the data
  {}

  // Copy assignment operator
  Field& operator=(const Field& other) {
    // Check for self-assignment
    if (this != &other) {
      // Create a new temporary unique_ptr that will manage the new data array
      std::unique_ptr<T[]> new_data(new T[other.size_]);
      // Copy the data from the other Field object to the new data array
      std::copy(other.data_.get(), other.data_.get() + other.size_, new_data.get());
      // Update this object's data_ pointer to the new data array
      data_ = std::move(new_data);
      // Copy other's metadata to this object's metadata
      ncols_ = other.ncols_;
      nrows_ = other.nrows_;
      size_ = other.size_;
    }
    return *this;
  }

  // Access methods
  // xth row, yth column - row-major order
  inline T& operator()(size_t y, size_t x) { return data_[x * ncols_ + y]; }
  inline const T& operator()(size_t y, size_t x) const { return data_[x * ncols_ + y]; }
    
  // Reinitialize with new size & value
  void reinitialize(size_t ncols, size_t nrows, T default_value) {
    ncols_ = ncols;
    nrows_ = nrows;
    size_ = ncols * nrows;
    if constexpr (unique == 0)
      data_ = std::make_unique<T[]>(size_);
    else
      data_ = std::make_shared<T[]>(size_);
    fill(default_value);
  }

  // Reset data using new
  void reset() {
    if constexpr (unique == 0)
      data_ = std::make_unique<T[]>(size_);
    else
      data_ = std::make_shared<T[]>(size_);
  }

  void fill(T value) {
    for (size_t i = 0; i < size_; ++i)
        data_[i] = value;
  }
  
  size_t ncols() const { return ncols_; }
  size_t nrows() const { return nrows_; }
  size_t size() const { return size_; }
  const auto& data() const { return data_; }

private:
  size_t ncols_;
  size_t nrows_;
  size_t size_;
  std::conditional_t<unique == 0, std::unique_ptr<T[]>, std::shared_ptr<T[]>> data_;
};

} // namespace vbs

#endif // FIELD_H