#pragma once

#include "opencv2/core/matx.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Types.h"

class AnalogMeterDetector {
public:
  AnalogMeterDetector(cv::Rect analog_speed_meter_coordinates,
                      const cv::MorphTypes &morph_type,
                      std::string headline_hint);
  ~AnalogMeterDetector() = default;

  void SetImage(const cv::Mat &image);
  void SetMorphTypeTransformation(const cv::MorphTypes &morph_type,
                                  std::string headline_hint);
  int GetAngle() const;

  /**
   * The method performs image processing
   */
  void Process();

private:
  void ApplyHoughLinesP();

  types::Line
  TurnLineInOppositeDirectionToReferenceLine(const types::Line &in) const;
  int CalculateAngleRelativeToReferenceLine(const types::Line &line) const;

private:
  // a rect to cut only desired image area
  const cv::Rect analog_meter_coordinates_;
  // for restore global image's coordinates after detection
  cv::Point analog_meter_start_coordinates_;
  // a line to help calculate arrow angle
  types::Line reference_line_;

  cv::Mat origin_image_;
  cv::Mat grey_edges_;

  cv::MorphTypes morph_type_;
  std::string headline_hint_;
  const cv::Mat kKernel = cv::Mat(5, 5, CV_8UC1, cv::Scalar(1));

  int detected_angle_ = 0;
};
