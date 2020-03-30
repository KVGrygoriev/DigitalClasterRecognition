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

void AnalogMeterDetector::ApplyHoughLinesP() {
  cv::Mat line_edges;
  cv::morphologyEx(grey_edges_, line_edges, morph_type_, kKernel);
  // cv::imshow("morphologyEx for " + headline_hint_, line_edges);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(line_edges, lines, 1, CV_PI / 180, 90, 70, 3);

  for (size_t i = 0; i < lines.size(); i++) {
    cv::Vec4i l = lines[i];

    cv::line(origin_image_,
             cv::Point(analog_meter_start_coordinates_.x + l[0],
                       analog_meter_start_coordinates_.y + l[1]),
             cv::Point(analog_meter_start_coordinates_.x + l[2],
                       analog_meter_start_coordinates_.y + l[3]),
             cv::Scalar(0, 0, 255), 3, cv::LINE_AA);

    //--------------------------- visual DBG section
    cv::circle(origin_image_,
               cv::Point(analog_meter_start_coordinates_.x + l[0],
                         analog_meter_start_coordinates_.y + l[1]),
               5, cv::Scalar(0, 255, 0));

    cv::line(origin_image_, reference_line_.first, reference_line_.second,
             cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

    cv::circle(origin_image_, reference_line_.first, 5, cv::Scalar(0, 0, 255));

    //------------------- angle calculation
    cv::Point a_start(l[0], l[1]);
    cv::Point a_end(l[2], l[3]);
    cv::Point a_vector(a_end.x - a_start.x, a_end.y - a_start.y);

    cv::Point b_vector(reference_line_.second.x - reference_line_.first.x,
                       reference_line_.second.y - reference_line_.first.y);

    auto value = (a_vector.x * b_vector.x + a_vector.y * b_vector.y) /
                 (sqrt(pow(a_vector.x, 2) + pow(a_vector.y, 2)) *
                  sqrt(pow(b_vector.x, 2) + pow(b_vector.y, 2)));

    std::cout << "(x,y) = (" << l[0] << "," << l[1] << ")"
              << "; (x,y) = (" << l[2] << "," << l[3] << ")"
              << "; cos = " << acos(value) * 180 / CV_PI
              << "; sin = " << asin(value) * 180 / CV_PI << std::endl;
  }

  // cv::imshow(headline_hint_ + " HoughLinesP", origin_image_);
  // cv::waitKey(0);
}
