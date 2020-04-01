#include <iostream>
#include <vector>

#include "AnalogMeterDetector.h"
#include "TelltalesDetector.h"
#include "VideoManager.h"

namespace {
const std::vector<std::pair<int, int>> kAnalogMeterFrameBoundaries{
    {250, 300}, {460, 580}, {670, 810}, {890, 915}, {925, 965}};

const std::vector<std::pair<cv::MorphTypes, std::string>>
    transformation_type_array{
        //{cv::MORPH_CLOSE, "MORPH_CLOSE"}, // has worse results than
        // MORPH_CLOSE
        {cv::MORPH_TOPHAT, "MORPH_TOPHAT"}};

//const cv::Rect kAnalogSpeedMeterCoordinates{-415, -520, 415, 360};
const cv::Rect kAnalogSpeedMeterCoordinates{-615, -770, 590, 510};
} // namespace

int main() {
  AnalogMeterDetector asm_detector(kAnalogSpeedMeterCoordinates,
                                   cv::MORPH_TOPHAT, "MORPH_TOPHAT");

  TelltalesDetector telltales_detector;
  if (!telltales_detector.GetTellTalesCount()) {
    std::cerr << "Icon list is empty";
    return 1;
  }

  VideoManager video_manager("poc/video/1080.mp4");
  cv::Mat frame;

  //--- GRAB AND WRITE LOOP
  std::cout << "Start grabbing" << std::endl
            << "Press Esc key to terminate" << std::endl;
  long long frame_index = 0;
  auto analog_meter_frame_window = kAnalogMeterFrameBoundaries.begin();

  while (video_manager.GetFrame(frame)) {

    telltales_detector.SetImage(frame);
    telltales_detector.Detect();

    // perform analog meter widget recognition only when it visible
    if (analog_meter_frame_window != kAnalogMeterFrameBoundaries.end()) {
      if (analog_meter_frame_window->first < frame_index &&
          analog_meter_frame_window->second > frame_index) {
        asm_detector.SetImage(frame);
        asm_detector.ApplyHoughLinesP();
      }

      if (analog_meter_frame_window->second < frame_index) {
        ++analog_meter_frame_window;
      }
    }

    // show live and wait for a key with timeout long enough to show images
    cv::imshow("Cluster", frame);

    /*
    if (!(frame_index % 10)) {
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
