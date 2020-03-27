#include <iostream>
#include <vector>

#include "AnalogMeterDetector.h"
#include "TelltalesDetector.h"
#include "VideoManager.h"

int main() {
  const std::vector<std::pair<cv::MorphTypes, std::string>>
      transformation_type_array{
          /*
          {cv::MORPH_ERODE, "MORPH_ERODE"},     // nothing after HoughLinesP
          {cv::MORPH_HITMISS, "MORPH_HITMISS"}, // nothing after HoughLinesP
          {cv::MORPH_OPEN, "MORPH_OPEN"},       // nothing after HoughLinesP
          {cv::MORPH_DILATE, "MORPH_DILATE"},   // a lot of false-positive cases
                                                // or empty at all
          {cv::MORPH_GRADIENT, "MORPH_GRADIENT"}, // a lot of false-positive
                                                  // cases or empty at all
          {cv::MORPH_BLACKHAT, "MORPH_BLACKHAT"}, // a lot of false-positive
                                                  // cases or empty at all
          */
          {cv::MORPH_CLOSE, "MORPH_CLOSE"},
          {cv::MORPH_TOPHAT, "MORPH_TOPHAT"}};
  const cv::Rect kAnalogSpeedMeterCoordinates{-415, -520, 415, 360};
  AnalogMeterDetector asm_detector(kAnalogSpeedMeterCoordinates,
                                   cv::MORPH_CLOSE, "MORPH_CLOSE");

  TelltalesDetector telltales_detector;
  if (!telltales_detector.GetTellTalesCount()) {
    std::cerr << "Icon list is empty";
    return 1;
  }

  VideoManager video_manager("poc/video/1.mp4");
  cv::Mat frame;

  //--- GRAB AND WRITE LOOP
  std::cout << "Start grabbing" << std::endl
            << "Press Esc key to terminate" << std::endl;
  long long frame_index = 0;
  while (video_manager.GetFrame(frame)) {

    telltales_detector.SetImage(frame);
    telltales_detector.Detect();

    asm_detector.SetImage(frame);

    for (const auto &algorithm : transformation_type_array) {
      asm_detector.SetMorphTypeTransformation(algorithm.first,
                                              algorithm.second);
      // asm_detector.ApplyHoughLines();
      asm_detector.ApplyHoughLinesP();
    }

    // show live and wait for a key with timeout long enough to show images
    // cv::imshow("Cluster", frame);

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
