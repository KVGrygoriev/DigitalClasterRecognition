#pragma once

#include "opencv2/core/matx.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct Line {
  cv::Point start_coord;
  cv::Point end_coord;
};

class AnalogMeterDetector {
public:
  AnalogMeterDetector(cv::Rect analog_speed_meter_coordinates,
                      const cv::MorphTypes &morph_type,
                      std::string headline_hint);
  ~AnalogMeterDetector() = default;

  void SetImage(const cv::Mat &image);
  void SetMorphTypeTransformation(const cv::MorphTypes &morph_type,
                                  std::string headline_hint);

  void ApplyHoughLines();
  void ApplyHoughLinesP();

private:
  const cv::Rect analog_meter_coordinates_;
  cv::Point analog_meter_start_coordinates_;
  Line reference_line_;

  cv::Mat origin_image_;
  cv::Mat grey_edges_;

  cv::MorphTypes morph_type_;
  std::string headline_hint_;
  const cv::Mat kKernel = cv::Mat(5, 5, CV_8UC1, cv::Scalar(1));
};
