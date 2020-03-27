#include "TelltalesDetector.h"

#include <future>
#include <iostream>
#include <mutex>

#include <experimental/filesystem>

#include <opencv2/imgproc/imgproc.hpp>

namespace {
std::mutex g_mutex_;

inline void DrawRectAtFrame(cv::Mat &frame, const cv::Rect &rect) {
  cv::rectangle(frame, rect, CV_RGB(0, 255, 0), 2);
}

cv::Rect DetectPattern(const cv::Mat &frame, const cv::Mat &pattern) {
  // verification of size compatibility (pattern should be less than frame)
  assert(frame.rows > pattern.rows);
  assert(frame.cols > pattern.cols);

  // Perform the size of result matrix
  // Magic number 1 needs for ability to matching all possible locations for it.
  const auto rows = frame.rows - pattern.rows + 1;
  const auto cols = frame.cols - pattern.cols + 1;

  cv::Mat frame_gray;
  cv::Mat pattern_gray;

  // The result of comparation between frame and pattern, must be
  // single-channel 32-bit floating-point.
  // If the frame is W×H and pattern is w×h, then the result is (W−w+1)×(H−h+1).
  cv::Mat value(rows, cols, CV_32FC1);
  cv::matchTemplate(frame, pattern, value, cv::TM_CCORR_NORMED);

  double minVal{}, maxVal{};
  cv::Point minLoc;
  cv::Point matchLoc;

  cv::minMaxLoc(value, &minVal, &maxVal, &minLoc, &matchLoc);

  // If template has not detected return empty Rect
  if (maxVal > 0.8) {
    return cv::Rect{matchLoc, cv::Size{pattern.cols, pattern.rows}};
  }

  return cv::Rect();
}

void ParallelTelltalesMatch(cv::Mat &screen, const cv::Mat &screen_gray,
                            std::vector<IconData>::iterator begin,
                            std::vector<IconData>::iterator end) {
  // lambda for drawing matched icon
  auto draw_matched = [&screen](std::vector<IconData>::iterator iter,
                                const cv::Rect &rect) {
    IconData &icon = *iter;
    if (!rect.empty()) {
      DrawRectAtFrame(screen, rect);
      if (IconData::kVisible != icon.state) {
        icon.state = IconData::kVisible;
        icon.match_cached = rect;
        std::cout << icon.name << " is visible\n";
      }
    } else {
      if (IconData::kInvisible != icon.state) {
        icon.state = IconData::kInvisible;
        icon.match_cached = cv::Rect();
        std::cout << icon.name << " is invisible\n";
      }
    }
  };

  auto len = std::distance(begin, end);
  if (len < 3) {
    while (begin != end) {
      IconData &icon = *begin;
      cv::Rect rect = DetectPattern(screen_gray, icon.frame);
      {
        std::lock_guard<std::mutex> lock(g_mutex_);
        draw_matched(begin, rect);
      }
      ++begin;
    }
  } else {
    std::vector<IconData>::iterator mid = std::next(begin, len / 2);

    auto handle =
        std::async(std::launch::async, [&screen, &screen_gray, mid, end]() {
          ParallelTelltalesMatch(screen, screen_gray, mid, end);
        });

    (void)handle;

    ParallelTelltalesMatch(screen, screen_gray, begin, mid);
  }
}
} // namespace

namespace fs = std::experimental::filesystem;

cv::Mat ApplyCannyAlgorithm_(const cv::Mat &frame) {
  cv::Mat result;
  cv::cvtColor(frame, result, CV_BGR2GRAY);
  cv::Canny(result, result, 50, 200);
  return result;
}

TelltalesDetector::TelltalesDetector() { LoadTellTalesIcons(); }

void TelltalesDetector::SetImage(const cv::Mat &image) {
  //origin_image_ = image.clone();
  origin_image_ = image;
  grey_edges_ = ApplyCannyAlgorithm_(image);
}

size_t TelltalesDetector::GetTellTalesCount() const {
  return telltales_icons_.size();
}

void TelltalesDetector::LoadTellTalesIcons() {
  for (const auto &path : fs::directory_iterator("poc/icons")) {
    if (path.path().extension() == ".png") {
      telltales_icons_.push_back(
          IconData{path.path().filename().string(),
                   ApplyCannyAlgorithm_(cv::imread(path.path().string())),
                   IconData::kNone});
    }
  }
}

void TelltalesDetector::Detect() {
  ParallelTelltalesMatch(origin_image_, grey_edges_, telltales_icons_.begin(),
                         telltales_icons_.end());
}
