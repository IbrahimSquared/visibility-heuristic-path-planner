#ifndef FIELD_H_
#define FIELD_H_

#include <cstddef>

namespace vbs {

using size_t = std::size_t;

// Defines a 2D field of type T
template <typename T>
class Field {
 public:
  /*!
   * Constructor.
   * @brief Initialize a Field.
   * @param [in] nrows Number of rows.
   * @param [in] ncols Number of cols.
   * @param [in] value Default value of the field.
   */
  Field(size_t nrows, size_t ncols, T value) : nrows_(nrows), ncols_(ncols), size_(nrows * ncols) {
    data_ = new T[size_];
    Fill(value);
  }

  // Deconstructor
  ~Field() { delete[] data_; }

  // Getters
  /*!
   * @brief Get value of an element in the field.
   * @param [in] x position.
   * @param [in] y position.
   */
  inline T& operator()(size_t y, size_t x) const { return data_[y * ncols_ + x]; }
  
  /*!
   * @brief Get value of an element in the field.
   * @param [in] x position.
   * @param [in] y position.
   */
  inline T& at(size_t y, size_t x) const { return data_[y * ncols_ + x]; }

  // Get nb of rows.
  inline size_t rows() const { return nrows_; };
  // Get nb of cols.
  inline size_t cols() const { return ncols_; };
  // Get size of field.
  inline size_t size() const { return size_; };


  // Setters
  /*!
   * @brief Sets field to a defined value
   * @param [in] value Value to which the field is set.
   */
  inline void Fill(const T& value) {
    for (size_t i = 0; i < size_; ++i) {
      data_[i] = value;
    }
  }

  /*!
   * @brief Set value of an element in the field.
   * @param [in] x position.
   * @param [in] y position.
   * @param [in] value.
   */
  inline void Set(size_t y, size_t x, const T& value) {
    data_[y * ncols_ + x] = value;
  }

  // Reset data
  inline void Reset() {
    data_ = new T[size_];
  }

 private:
  // raw array containing data of type T.
  T* data_;
  // Dimensions.
  size_t nrows_; size_t ncols_; size_t size_;
};

} // namespace vbs

#endif  // FIELD_H_