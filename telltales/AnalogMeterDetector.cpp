#include "AnalogMeterDetector.h"

#include <iostream>
#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>

cv::Mat ApplyCannyAlgorithm(const cv::Mat &frame) {
  cv::Mat result;
  cv::cvtColor(frame, result, CV_BGR2GRAY);
  cv::Canny(result, result, 50, 200);
  return result;
}

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

  grey_edges_ = grey_edges_(cv::Rect{
      analog_meter_start_coordinates_.x, analog_meter_start_coordinates_.y,
      analog_meter_coordinates_.width, analog_meter_coordinates_.height});

  static const int kShiftByAxisY = 25;
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

void AnalogMeterDetector::ApplyHoughLines() {
  cv::Mat line_edges;
  cv::morphologyEx(grey_edges_, line_edges, morph_type_, kKernel);
  // cv::imshow("morphologyEx for " + headline_hint_, line_edges);

  std::vector<cv::Vec2f> lines;
  cv::HoughLines(line_edges, lines, 1, CV_PI / 180, 10);

  for (const auto &line : lines) {
    float rho = line[0], theta = line[1];
    cv::Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));
    cv::line(origin_image_, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);

    /*
    std::cout << "rho = " << rho << "; theta = " << theta * 180.0 / CV_PI
               << "; (x,y) = (" << pt1.x << "," << pt1.y << ")"
               << "; (x,y) = (" << pt2.x << "," << pt2.y << ")" << std::endl;
               */
  }

  cv::imshow(headline_hint_ + " HoughLines", origin_image_);
  // cv::waitKey(0);
}

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

  // reference_line_.x
  if (in.start_coord.x < in.end_coord.x)
    return in;

  // under
  if ((in.end_coord.x > in.start_coord.x) &&
      (in.end_coord.y < in.start_coord.y))
    return in;

  // below
  if ((in.end_coord.x > in.start_coord.x) &&
      (in.end_coord.y > in.start_coord.y))
    return in;

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

void AnalogMeterDetector::ApplyHoughLinesP() {
  cv::Mat line_edges;
  cv::morphologyEx(grey_edges_, line_edges, morph_type_, kKernel);
  // cv::imshow("morphologyEx for " + headline_hint_, line_edges);

  Line detected_line;
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(line_edges, lines, 1, CV_PI / 180, 90, 70, 3);

  for (size_t i = 0; i < lines.size(); i++) {

    detected_line = TurnLineInOppositeDirectionToReferenceLine(
        {cv::Point(analog_meter_start_coordinates_.x + lines[i][0],
                   analog_meter_start_coordinates_.y + lines[i][1]),
         cv::Point(analog_meter_start_coordinates_.x + lines[i][2],
                   analog_meter_start_coordinates_.y + lines[i][3])});

    cv::line(origin_image_, detected_line.start_coord, detected_line.end_coord,
             cv::Scalar(0, 0, 255), 3, cv::LINE_AA);

    //--------------------------- visual DBG section
    cv::circle(origin_image_, detected_line.start_coord, 5,
               cv::Scalar(0, 255, 0));

    cv::line(origin_image_, reference_line_.start_coord,
             reference_line_.end_coord, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

    cv::circle(origin_image_, reference_line_.start_coord, 5,
               cv::Scalar(0, 0, 255));

    //-------------------

    int angle = CalculateAngleRelativeToReferenceLine(detected_line);

    std::cout << "(x,y) = (" << lines[i][0] << "," << lines[i][1] << ")"
              << "; (x,y) = (" << lines[i][2] << "," << lines[i][3] << ")"
              << "; cos = " << angle << std::endl;
  }

  // cv::imshow(headline_hint_ + " HoughLinesP", origin_image_);
  // cv::waitKey(0);
}
