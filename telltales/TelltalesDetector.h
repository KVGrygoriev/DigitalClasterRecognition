#pragma once

#include <string>
#include <iterator>
#include <vector>

#include <opencv2/highgui.hpp>

struct IconData {
  enum State : u_int8_t { kNone, kVisible, kInvisible };

  IconData(std::string icon_name, cv::Mat mat, State icon_state)
      : name(std::move(icon_name)), frame(std::move(mat)), state(icon_state) {}

  std::string name;
  cv::Mat frame{};
  State state{kNone};
  cv::Rect match_cached{};
};

class TelltalesDetector {
public:
  TelltalesDetector();
  ~TelltalesDetector() = default;

  void SetImage(const cv::Mat &image);
  size_t GetTellTalesCount() const;

  void Detect();

private:
  void LoadTellTalesIcons();
  
private:
  cv::Mat origin_image_;
  cv::Mat grey_edges_;

  std::vector<IconData> telltales_icons_;
};
