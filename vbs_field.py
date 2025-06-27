import numpy as np

class Field:
    def __init__(self, nx: int = 0, ny: int = 0, default_value=0.0):
        self.nx_ = nx
        self.ny_ = ny
        self.size_ = nx * ny
        if nx > 0 and ny > 0:
            # Default to float64 to match C++ double, unless default_value suggests otherwise
            dtype = type(default_value) if not isinstance(default_value, float) else np.float64
            self.data_ = np.full((ny, nx), default_value, dtype=dtype)
        else:
            # Allow creation of an empty field, data can be set by resize
            self.data_ = np.array([], dtype=np.float64)

    def set_value(self, x: int, y: int, value):
        if not (0 <= x < self.nx_ and 0 <= y < self.ny_):
            raise IndexError(f"Field set_value: Index ({x}, {y}) out of bounds for size ({self.nx_}, {self.ny_})")
        self.data_[y, x] = value # Note: NumPy uses (row, col) which is (y, x)

    def get_value(self, x: int, y: int):
        if not (0 <= x < self.nx_ and 0 <= y < self.ny_):
            raise IndexError(f"Field get_value: Index ({x}, {y}) out of bounds for size ({self.nx_}, {self.ny_})")
        return self.data_[y, x] # Note: NumPy uses (row, col) which is (y, x)

    def __call__(self, x: int, y: int, value=None):
        """
        Allows using field(x,y) to get and field(x,y,value) to set.
        Matches C++ `operator()` behavior.
        """
        if value is None:
            return self.get_value(x, y)
        else:
            self.set_value(x, y, value)

    @property
    def nx(self) -> int:
        return self.nx_

    @property
    def ny(self) -> int:
        return self.ny_

    @property
    def size(self) -> int:
        return self.size_

    @property
    def dtype(self) -> np.dtype:
        return self.data_.dtype

    def reset(self, default_value=0.0):
        """Resets the field to default_value, maintaining current dimensions and dtype."""
        if self.size_ > 0 :
            # Use current dtype if not specified by default_value's type
            dtype_to_use = type(default_value) if not isinstance(default_value, float) else self.data_.dtype
            self.data_ = np.full((self.ny_, self.nx_), default_value, dtype=dtype_to_use)
        # If size is 0, it remains an empty array, consistent with C++ unique_ptr behavior after reset if size was 0

    def resize(self, nx: int, ny: int, default_value=0.0):
        self.nx_ = nx
        self.ny_ = ny
        self.size_ = nx * ny
        if nx > 0 and ny > 0:
            dtype = type(default_value) if not isinstance(default_value, float) else np.float64
            self.data_ = np.full((self.ny_, self.nx_), default_value, dtype=dtype)
        else:
            self.data_ = np.array([], dtype=np.float64)


if __name__ == '__main__':
    print("Testing Field class...")

    # Test basic initialization
    field1 = Field(nx=3, ny=2, default_value=1.0)
    assert field1.nx == 3
    assert field1.ny == 2
    assert field1.size == 6
    assert field1.dtype == np.float64
    print(f"Field1 (3x2, default 1.0):\n{field1.data_}")

    # Test set and get
    field1.set_value(0, 0, 5.5)
    assert field1.get_value(0, 0) == 5.5
    print(f"Field1 after set_value(0,0, 5.5):\n{field1.data_}")

    # Test __call__ for get and set
    assert field1(1, 0) == 1.0 # Get
    field1(1, 0, 7.7)          # Set
    assert field1(1, 0) == 7.7
    print(f"Field1 after field1(1,0, 7.7):\n{field1.data_}")

    # Test with integer default_value
    field_int = Field(nx=2, ny=2, default_value=10)
    assert field_int.dtype == np.int64 or field_int.dtype == np.int32 # Depending on system
    assert field_int(0,0) == 10
    print(f"Field_int (2x2, default 10, dtype {field_int.dtype}):\n{field_int.data_}")
    field_int(0,0,100)
    assert field_int(0,0) == 100

    # Test reset
    field1.reset(default_value=0.5)
    assert field1.get_value(0, 0) == 0.5
    assert field1.get_value(1, 0) == 0.5
    assert field1.nx == 3 # Dimensions should be maintained
    assert field1.ny == 2
    assert field1.dtype == np.float64 # Dtype should be maintained if not changed by default_value
    print(f"Field1 after reset(0.5):\n{field1.data_}")

    # Test resize
    field1.resize(nx=2, ny=1, default_value=2.2)
    assert field1.nx == 2
    assert field1.ny == 1
    assert field1.size == 2
    assert field1.get_value(0, 0) == 2.2
    assert field1.dtype == np.float64
    print(f"Field1 after resize(2,1, 2.2):\n{field1.data_}")

    # Test initialization with 0 size
    field_empty = Field()
    assert field_empty.nx == 0
    assert field_empty.ny == 0
    assert field_empty.size == 0
    print(f"Field_empty (initial):\n{field_empty.data_}")
    field_empty.resize(1,1,5)
    assert field_empty.nx == 1
    assert field_empty.get_value(0,0) == 5
    print(f"Field_empty after resize(1,1,5):\n{field_empty.data_}")


    # Test index out of bounds
    try:
        field1.get_value(10, 10)
    except IndexError as e:
        print(f"Caught expected error: {e}")
    try:
        field1.set_value(10,10,0)
    except IndexError as e:
        print(f"Caught expected error: {e}")

    print("Field class tests passed.")
