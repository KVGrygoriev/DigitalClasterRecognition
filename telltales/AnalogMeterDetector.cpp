#include "AnalogMeterDetector.h"

#include <algorithm>
#include <iostream>
#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "MathHelpers.h"

using namespace types;

namespace {
cv::Mat ApplyCannyAlgorithm(const cv::Mat &frame) {
  cv::Mat result;
  cv::cvtColor(frame, result, CV_BGR2GRAY);
  cv::Canny(result, result, 50, 200);
  return result;
}

bool IsLineAtDigitalMeterArea(const cv::Point &center_point,
                              const Line &detected_line) {
  static const int kSpeedDigitsArea = 100;

  if ((detected_line.start_coord.x > center_point.x - kSpeedDigitsArea) &&
      (detected_line.start_coord.x < center_point.x + kSpeedDigitsArea) &&
      (detected_line.start_coord.y > center_point.y - kSpeedDigitsArea) &&
      (detected_line.start_coord.y < center_point.y + kSpeedDigitsArea)) {
    return true;
  } else {
    return false;
  }
}

} // namespace

AnalogMeterDetector::AnalogMeterDetector(
    cv::Rect analog_speed_meter_coordinates, const cv::MorphTypes &morph_type,
    std::string headline_hint)
    : analog_meter_coordinates_(analog_speed_meter_coordinates),
      morph_type_(morph_type), headline_hint_(std::move(headline_hint)) {}

void AnalogMeterDetector::SetImage(const cv::Mat &image) {
  // origin_image_ = image.clone();
  origin_image_ = image;
  grey_edges_ = ApplyCannyAlgorithm(image);

  analog_meter_start_coordinates_ =
      cv::Point{grey_edges_.cols + analog_meter_coordinates_.x,
                grey_edges_.rows + analog_meter_coordinates_.y};

  // cut only desired zone
  grey_edges_ = grey_edges_(cv::Rect{
      analog_meter_start_coordinates_.x, analog_meter_start_coordinates_.y,
      analog_meter_coordinates_.width, analog_meter_coordinates_.height});

  static const int kShiftByAxisY = 35;
  reference_line_ = {
      cv::Point(analog_meter_start_coordinates_.x +
                    analog_meter_coordinates_.width / 2,
                analog_meter_start_coordinates_.y +
                    analog_meter_coordinates_.height / 2 + kShiftByAxisY),
      cv::Point(analog_meter_start_coordinates_.x +
                    analog_meter_coordinates_.width / 2 + 100,
                analog_meter_start_coordinates_.y +
                    analog_meter_coordinates_.height / 2 + kShiftByAxisY)};
}

void AnalogMeterDetector::SetMorphTypeTransformation(
    const cv::MorphTypes &morph_type, std::string headline_hint) {
  morph_type_ = morph_type;
  headline_hint_ = std::move(headline_hint);
}

void AnalogMeterDetector::Process() { ApplyHoughLinesP(); }

Line AnalogMeterDetector::TurnLineInOppositeDirectionToReferenceLine(
    const Line &in) const {

  // top hemisphere
  if (reference_line_.start_coord.y >= in.start_coord.y) {
    // top left side
    if (reference_line_.start_coord.x > in.start_coord.x) {
      if (in.start_coord.x < in.end_coord.x) {
        return Line{in.end_coord, in.start_coord};
      } else {
        return in;
      }
    }

    // top right side
    if (reference_line_.start_coord.x < in.start_coord.x) {
      if (in.start_coord.x > in.end_coord.x) {
        return Line{in.end_coord, in.start_coord};
      } else {
        return in;
      }
    }
  } else { // bottom hemisphere

    // bottom right side
    if (reference_line_.start_coord.x < in.start_coord.x) {
      if (in.start_coord.x > in.end_coord.x) {
        return Line{in.end_coord, in.start_coord};
      } else {
        return in;
      }
    }

    // bottom left side
    if (reference_line_.start_coord.x > in.start_coord.x) {
      if (in.start_coord.x < in.end_coord.x) {
        return Line{in.end_coord, in.start_coord};
      } else {
        return in;
      }
    }
  }

  return in;
}

int AnalogMeterDetector::CalculateAngleRelativeToReferenceLine(
    const Line &line) const {
  static const int kHalfOfTheSphere = 180;

  cv::Point a_vector(line.end_coord.x - line.start_coord.x,
                     line.end_coord.y - line.start_coord.y);

  cv::Point b_vector(
      reference_line_.end_coord.x - reference_line_.start_coord.x,
      reference_line_.end_coord.y - reference_line_.start_coord.y);

  // get the angle between vectors
  auto value = (a_vector.x * b_vector.x + a_vector.y * b_vector.y) /
               (sqrt(pow(a_vector.x, 2) + pow(a_vector.y, 2)) *
                sqrt(pow(b_vector.x, 2) + pow(b_vector.y, 2)));

  int angle_at_degrees = static_cast<int>(acos(value) * kHalfOfTheSphere /
                                          CV_PI); // radians to degrees

  // make a correlation in depend on a semi-sphere
  if (line.start_coord.y > reference_line_.start_coord.y) {
    angle_at_degrees = kHalfOfTheSphere + (kHalfOfTheSphere - angle_at_degrees);
  }

  return angle_at_degrees;
}

void CleanNoiseValues(const types::Line &reference_line,
                      std::vector<cv::Vec4i> &lines) {
  // remove lines detected at numbers area
  lines.erase(
      remove_if(lines.begin(), lines.end(),
                [reference_line = reference_line](const cv::Vec4i &points) {
                  return IsLineAtDigitalMeterArea(
                      reference_line.start_coord,
                      {{points[0], points[1]}, {points[2], points[3]}});
                }),
      lines.end());
}

void AnalogMeterDetector::ApplyHoughLinesP() {
  cv::Mat line_edges;
  cv::morphologyEx(grey_edges_, line_edges, morph_type_, kKernel);
  // cv::imshow("morphologyEx for " + headline_hint_, line_edges);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(line_edges, lines, 1, CV_PI / 180, 90, 95, 5);

  if (lines.empty())
    return;

  // returning to global coordinates. Needs only for drawing
  for (auto &line_coordinates : lines) {
    line_coordinates[0] += analog_meter_start_coordinates_.x;
    line_coordinates[1] += analog_meter_start_coordinates_.y;
    line_coordinates[2] += analog_meter_start_coordinates_.x;
    line_coordinates[3] += analog_meter_start_coordinates_.y;
  }

  CleanNoiseValues(reference_line_, lines);

  if (lines.empty())
    return;

  Line interpolated_line = TurnLineInOppositeDirectionToReferenceLine(
      InterpolateToSingleLine(lines));

  cv::line(origin_image_, interpolated_line.start_coord,
           interpolated_line.end_coord, cv::Scalar(255, 0, 255), 3,
           cv::LINE_AA);

  detected_angle_ = CalculateAngleRelativeToReferenceLine(interpolated_line);

  /*
  // Print line's coordinates
  std::cout << "(x,y) = (" << interpolated_line.start_coord.x << ","
            << interpolated_line.start_coord.y << ")"
            << "; (x,y) = (" << interpolated_line.end_coord.x << ","
            << interpolated_line.end_coord.y << ")"
            << "; cos = " << detected_angle_ << std::endl;
            */

  /*
  // Draw all detected lines
  for (size_t i = 0; i < lines.size(); i++) {
    cv::line(origin_image_, detected_line.start_coord, detected_line.end_coord,
             cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
  }
  */
}

int AnalogMeterDetector::GetAngle() const { return detected_angle_; }
