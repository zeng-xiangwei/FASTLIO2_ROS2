#pragma once

#include <Eigen/Eigen>

namespace utils {

struct CellLimits {
  CellLimits() = default;
  CellLimits(int init_num_x_cells, int init_num_y_cells)
      : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

  int num_x_cells = 0;
  int num_y_cells = 0;
};

class XYIndexRangeIterator
    : public std::iterator<std::input_iterator_tag, Eigen::Array2i> {
public:
    // Constructs a new iterator for the specified range.
    XYIndexRangeIterator(const Eigen::Array2i& min_xy_index,
                         const Eigen::Array2i& max_xy_index)
        : min_xy_index_(min_xy_index), max_xy_index_(max_xy_index),
          xy_index_(min_xy_index) {}

    // Constructs a new iterator for everything contained in 'cell_limits'.
    explicit XYIndexRangeIterator(const CellLimits& cell_limits)
        : XYIndexRangeIterator(Eigen::Array2i::Zero(),
                               Eigen::Array2i(cell_limits.num_x_cells - 1,
                                              cell_limits.num_y_cells - 1)) {}

    XYIndexRangeIterator& operator++() {
        // This is a necessary evil. Bounds checking is very expensive and needs
        // to be avoided in production. We have unit tests that exercise this
        // check in debug mode.
        if (xy_index_.x() < max_xy_index_.x()) {
            ++xy_index_.x();
        } else {
            xy_index_.x() = min_xy_index_.x();
            ++xy_index_.y();
        }
        return *this;
    }

    Eigen::Array2i& operator*() { return xy_index_; }

    bool operator==(const XYIndexRangeIterator& other) const {
        return (xy_index_ == other.xy_index_).all();
    }

    bool operator!=(const XYIndexRangeIterator& other) const {
        return !operator==(other);
    }

    XYIndexRangeIterator begin() {
        return XYIndexRangeIterator(min_xy_index_, max_xy_index_);
    }

    XYIndexRangeIterator end() {
        XYIndexRangeIterator it = begin();
        it.xy_index_ = Eigen::Array2i(min_xy_index_.x(), max_xy_index_.y() + 1);
        return it;
    }

private:
    Eigen::Array2i min_xy_index_;
    Eigen::Array2i max_xy_index_;
    Eigen::Array2i xy_index_;
};

// 表示栅格地图的限制，以左上角为栅格数据存储原点（x_min, y_max）
class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& xmin_ymax, const CellLimits& cell_limits)
      : resolution_(resolution), xmin_ymax_(xmin_ymax), cell_limits_(cell_limits) {}

  // Returns the cell size in meters. All cells are square and the resolution
  // is the length of one side.
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  const Eigen::Vector2d& xmin_ymax() const { return xmin_ymax_; }

  // Returns the limits of the grid in number of cells.
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the 'point' which may be outside
  // the map, i.e., negative or too large indices that will return false for
  // Contains().
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    // Index values are row major and the top left has
    // Eigen::Array2i::Zero() and contains (centered_max_x, centered_max_y).
    // We need to flip and rotate.
    return Eigen::Array2i(std::floor((point.x() - xmin_ymax_.x()) / resolution_),
                          std::floor((xmin_ymax_.y() - point.y()) / resolution_));
  }

  // Returns the center of the cell at 'cell_index'.
  Eigen::Vector2f GetCellCenter(const Eigen::Array2i cell_index) const {
    return {xmin_ymax_.x() + cell_index.x() * resolution_ + resolution_ / 2.0f,
            xmin_ymax_.y() - cell_index.y() * resolution_ - resolution_ / 2.0f};
  }

  // Returns true if the ProbabilityGrid contains 'cell_index'.
  bool Contains(const Eigen::Array2i& cell_index) const {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index < Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells)).all();
  }

 private:
  double resolution_;
  Eigen::Vector2d xmin_ymax_;
  CellLimits cell_limits_;
};

}  // namespace utils