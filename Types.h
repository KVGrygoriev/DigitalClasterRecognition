#pragma once

#include <opencv2/core/types.hpp>

namespace types {
struct Line {
  cv::Point start_coord;
  cv::Point end_coord;
};
} // namespace types
