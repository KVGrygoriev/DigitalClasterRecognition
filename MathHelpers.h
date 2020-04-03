#pragma once

#include <vector>

#include "opencv2/core/matx.hpp"

#include "Types.h"

int GetMedianValue(std::vector<int> &points) {
  size_t amount = points.size();

  if (amount == 0)
    return 0; // undefined

  std::sort(points.begin(), points.end());
  if (amount % 2 == 0) {
    return (points[amount / 2 - 1] + points[amount / 2]) / 2;
  } else {
    return points[amount / 2];
  }
}

types::Line InterpolateToSingleLine(const std::vector<cv::Vec4i> &lines) {
  if (lines.size() == 1)
    return types::Line{{lines[0][0], lines[0][1]}, {lines[0][2], lines[0][3]}};

  std::vector<int> start_x, start_y, end_x, end_y;

  for (const auto &points : lines) {
    start_x.push_back(points[0]);
    start_y.push_back(points[1]);
    end_x.push_back(points[2]);
    end_y.push_back(points[3]);
  }

  return types::Line{{GetMedianValue(start_x), GetMedianValue(start_y)},
                     {GetMedianValue(end_x), GetMedianValue(end_y)}};
}
