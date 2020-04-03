#include "VideoManager.h"

#include <iostream>
#include <stdexcept>

VideoManager::VideoManager(const std::string &video_source) {
  source_.open(video_source);
  if (!source_.isOpened()) throw std::runtime_error("Unable to open video");
}

bool VideoManager::GetFrame(cv::Mat &frame) {
  source_.read(frame);

  if (frame.empty()) {
    std::cerr << "ERROR! blank frame grabbed\n";
    return false;
  }

  return true;
}