#pragma once

#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

class VideoManager {
public:
  VideoManager(const std::string &video_source);
  ~VideoManager() = default;

  bool GetFrame(cv::Mat &frame);

private:
  cv::VideoCapture source_;
};
