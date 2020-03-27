#include <iostream>
#include <stdio.h>

#include <experimental/filesystem>
#include <future>
#include <iterator>
#include <mutex>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "VideoManager.h"

namespace fs = std::experimental::filesystem;

struct IconData {
  enum State : u_int8_t { kNone, kVisible, kInvisible };

  IconData(std::string icon_name, cv::Mat mat, State icon_state)
      : name(std::move(icon_name)), frame(std::move(mat)), state(icon_state) {}

  std::string name;
  cv::Mat frame{};
  State state{kNone};
  cv::Rect match_cached{};
};

namespace {

std::mutex g_mutex;

cv::Mat ProcessFrame(const cv::Mat &frame) {
  cv::Mat result;
  cv::cvtColor(frame, result, CV_BGR2GRAY);
  cv::Canny(frame, result, 50, 200);
  return result;
}

cv::Rect Match(const cv::Mat &frame, const cv::Mat &pattern) {
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

inline void SetRectForFrame(cv::Mat &screen, const cv::Rect &rect) {
  cv::rectangle(screen, rect, CV_RGB(0, 255, 0), 2);
}

template <typename Iter>
void ParallelImageMatch(cv::Mat &screen, const cv::Mat &screen_gray, Iter begin,
                   Iter end) {
  // lambda for drawing matched icon
  auto draw_matched = [&screen](Iter iter, const cv::Rect &rect) {
    IconData &icon = *iter;
    if (!rect.empty()) {
      SetRectForFrame(screen, rect);
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
      cv::Rect rect = Match(screen_gray, icon.frame);
      {
        std::lock_guard<std::mutex> lock(g_mutex);
        draw_matched(begin, rect);
      }
      ++begin;
    }
  } else {
    Iter mid = std::next(begin, len / 2);

    auto handle =
        std::async(std::launch::async, [&screen, &screen_gray, mid, end]() {
          ParallelImageMatch<Iter>(screen, screen_gray, mid, end);
        });
    (void)handle;
    ParallelImageMatch(screen, screen_gray, begin, mid);
  }
}

std::vector<IconData> LoadTellTalesIcons() {
  std::vector<IconData> tell_tales_icons;
  for (const auto &path : fs::directory_iterator("poc/icons")) {
    if (path.path().extension() == ".png") {
      tell_tales_icons.push_back(IconData{
          path.path().filename().string(),
          ProcessFrame(cv::imread(path.path().string())), IconData::kNone});
    }
  }

  return tell_tales_icons;
}

} // namespace

int main() {
  VideoManager video_manager("poc/video/1.mp4");
  cv::Mat frame;
  
  std::vector<IconData> icon_frames = LoadTellTalesIcons();

  if (icon_frames.empty()) {
    std::cerr << "Icon list is empty";
    return 1;
  }

  cv::Mat frame_gray;
  //--- GRAB AND WRITE LOOP
  std::cout << "Start grabbing" << std::endl
            << "Press Esc key to terminate" << std::endl;
  long long frame_index = 0;
  while (video_manager.GetFrame(frame)) {
    
    frame_gray = ProcessFrame(frame);

    ParallelImageMatch(frame, frame_gray, icon_frames.begin(), icon_frames.end());

    // show live and wait for a key with timeout long enough to show images
    cv::imshow("Cluster", frame);

    /*
        if (!(frame_index % 50)) {
          std::cout << frame_index << std::endl;
          cv::waitKey(10000);
        }
        */

    ++frame_index;
    if (cv::waitKey(1) >= 0)
      break;
  }

  return 0;
}
