#include <iostream>
#include <math.h>
#include <vector>

#include "AnalogMeterDetector.h"
#include "DetectionSettings.h"
#include "TelltalesDetector.h"
#include "VideoManager.h"

namespace {
void DrawTextValue(cv::Mat &image, const cv::Point &left_bottom_point,
                   const std::string &text) {
  cv::putText(image, text, left_bottom_point, cv::FONT_HERSHEY_DUPLEX, 1.0,
              CV_RGB(118, 185, 0), 2);
}

inline int AngleToSpeed(int angle, double degree_coef) {
  int speed_value = static_cast<int>(
      round((settings::kZeroAnglePointAtAnalogMeter - angle) / degree_coef));
  return speed_value < 0 ? 0 : speed_value;
}

inline int AngleToRPM(int angle, double degree_coef) {
  int rpm_value = static_cast<int>(
      round((settings::kZeroAnglePointAtAnalogMeter - angle) / degree_coef));
  return rpm_value < 0 ? 0 : rpm_value;
}

std::vector<settings::RecognitionInfo> GetRecognitionInfo() {
  std::vector<settings::RecognitionInfo> recognition_settings;
  recognition_settings.emplace_back(settings::RecognitionInfo{
      250, 300, true, true, settings::kDegreeToSpeedCoef,
      settings::kDegreeToRPMCoefFor360});
  recognition_settings.emplace_back(settings::RecognitionInfo{
      460, 560, true, true, settings::kDegreeToSpeedCoef,
      settings::kDegreeToRPMCoefFor360});
  recognition_settings.emplace_back(settings::RecognitionInfo{
      560, 580, true, true, settings::kDegreeToSpeedCoef,
      settings::kDegreeToRPMCoefFor180}); //
  recognition_settings.emplace_back(settings::RecognitionInfo{
      670, 750, true, true, settings::kDegreeToSpeedCoef,
      settings::kDegreeToRPMCoefFor180}); //
  recognition_settings.emplace_back(settings::RecognitionInfo{
      750, 800, true, true, settings::kDegreeToSpeedCoef,
      settings::kDegreeToRPMCoefFor360});
  recognition_settings.emplace_back(settings::RecognitionInfo{
      800, 810, true, true, settings::kDegreeToSpeedCoef,
      settings::kDegreeToRPMCoefFor180}); //
  recognition_settings.emplace_back(settings::RecognitionInfo{
      890, 900, true, true, settings::kDegreeToSpeedCoef,
      settings::kDegreeToRPMCoefFor360});
  recognition_settings.emplace_back(settings::RecognitionInfo{
      900, 915, true, true, settings::kDegreeToSpeedCoef,
      settings::kDegreeToRPMCoefFor180}); //
  recognition_settings.emplace_back(settings::RecognitionInfo{
      925, 965, true, true, settings::kDegreeToSpeedCoef,
      settings::kDegreeToRPMCoefFor180}); //
  return recognition_settings;
}

} // namespace

int main() {
  auto recognition_settings = GetRecognitionInfo();

  AnalogMeterDetector speed_detector(settings::kAnalogSpeedMeterCoordinates,
                                     cv::MORPH_TOPHAT, "MORPH_TOPHAT");
  AnalogMeterDetector rpm_detector(settings::kAnalogRPMMeterCoordinates,
                                   cv::MORPH_TOPHAT, "MORPH_TOPHAT");
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
  auto recognition_settings_it = recognition_settings.begin();

  while (video_manager.GetFrame(frame)) {
    if (frame.size() != settings::kFrameSize) {
      std::cerr << "Frame size is too small for current settings!";
      return 1;
    }

    telltales_detector.SetImage(frame);
    telltales_detector.Detect();

    // perform analog meter widget recognition only when it visible
    if (recognition_settings_it != recognition_settings.end()) {
      if (recognition_settings_it->first_frame < frame_index &&
          recognition_settings_it->last_frame > frame_index) {
        // speed detection
        if (recognition_settings_it->speed_recognition) {
          speed_detector.SetImage(frame);
          speed_detector.Process();

          DrawTextValue(frame, cv::Point{frame.cols - 500, 70},
                        "Detected angle is " +
                            std::to_string(speed_detector.GetAngle()));
          DrawTextValue(frame, cv::Point{frame.cols - 500, 100},
                        "Approximately speed is " +
                            std::to_string(AngleToSpeed(
                                speed_detector.GetAngle(),
                                recognition_settings_it->speed_degree_coef)));
        }

        // RPM detection
        if (recognition_settings_it->rpm_recognition) {
          rpm_detector.SetImage(frame);
          rpm_detector.Process();

          DrawTextValue(frame, cv::Point{100, 70},
                        "Detected angle is " +
                            std::to_string(rpm_detector.GetAngle()));
          DrawTextValue(frame, cv::Point{100, 100},
                        "Approximately RPM is " +
                            std::to_string(AngleToRPM(
                                rpm_detector.GetAngle(),
                                recognition_settings_it->rpm_degree_coef)));
        }

        //cv::waitKey(100);
      }

      if (recognition_settings_it->last_frame < frame_index) {
        ++recognition_settings_it;
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
