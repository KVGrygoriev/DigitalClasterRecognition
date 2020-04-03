#include <iostream>
#include <math.h>
#include <vector>

#include "AnalogMeterDetector.h"
#include "TelltalesDetector.h"
#include "VideoManager.h"
#include "DetectionSettings.h"

namespace {
void DrawTextValue(cv::Mat &image, const cv::Point &left_bottom_point,
                   const std::string &text) {
  cv::putText(image, text, left_bottom_point, cv::FONT_HERSHEY_DUPLEX, 1.0,
              CV_RGB(118, 185, 0), 2);
}

inline int AngleToSpeed(int angle) {
  static const double kOneDegreeToSpeed = 1.75;
  int speed_value = static_cast<int>(
      round((settings::kZeroAnglePointAtAnalogMeter - angle) / kOneDegreeToSpeed));
  return speed_value < 0 ? 0 : speed_value;
}

inline int AngleToRPM(int angle) {
  static const double kOneDegreeToRPM = 0.035;
  int rpm_value = static_cast<int>(
      round((settings::kZeroAnglePointAtAnalogMeter - angle) / kOneDegreeToRPM));
  return rpm_value < 0 ? 0 : rpm_value;
}

} // namespace

int main() {
  AnalogMeterDetector speed_detector(settings::kAnalogSpeedMeterCoordinates,
                                     cv::MORPH_TOPHAT, "MORPH_TOPHAT");
  AnalogMeterDetector rpm_detector(settings::kAnalogRPMMeterCoordinates, cv::MORPH_TOPHAT,
                                   "MORPH_TOPHAT");
  rpm_detector.UseOnlyLeftHemisphere();

  TelltalesDetector telltales_detector(settings::kTelltalesPanelCoordinates);
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
  auto analog_meter_frame_window = settings::kAnalogMeterFrameBoundaries.begin();

  while (video_manager.GetFrame(frame)) {
    if (frame.size() != settings::kFrameSize) {
      std::cerr << "Frame size is too small for current settings!";
      return 1;
    }

    telltales_detector.SetImage(frame);
    telltales_detector.Detect();

    // perform analog meter widget recognition only when it visible
    if (analog_meter_frame_window != settings::kAnalogMeterFrameBoundaries.end()) {
      if (analog_meter_frame_window->first < frame_index &&
          analog_meter_frame_window->second > frame_index) {
        // speed detection
        speed_detector.SetImage(frame);
        speed_detector.Process();

        DrawTextValue(frame, cv::Point{frame.cols - 500, 70},
                      "Detected angle is " +
                          std::to_string(speed_detector.GetAngle()));
        DrawTextValue(
            frame, cv::Point{frame.cols - 500, 100},
            "Approximately speed is " +
                std::to_string(AngleToSpeed(speed_detector.GetAngle())));

        // RPM detection
        rpm_detector.SetImage(frame);
        rpm_detector.Process();

        DrawTextValue(frame, cv::Point{100, 70},
                      "Detected angle is " +
                          std::to_string(rpm_detector.GetAngle()));
        DrawTextValue(frame, cv::Point{100, 100},
                      "Approximately RPM is " +
                          std::to_string(AngleToRPM(rpm_detector.GetAngle())));

        // cv::waitKey(150);
      }

      if (analog_meter_frame_window->second < frame_index) {
        ++analog_meter_frame_window;
      }
    }

    // show live and wait for a key with timeout long enough to show images
    cv::imshow("Cluster", frame);

    ++frame_index;
    if (cv::waitKey(1) >= 0)
      break;
  }

  return 0;
}
