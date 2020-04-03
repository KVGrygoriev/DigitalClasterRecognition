#pragma once

#include <vector>

#include <opencv2/imgproc.hpp>

namespace settings {
struct RecognitionInfo {
  int first_frame;
  int last_frame;

  bool speed_recognition;
  bool rpm_recognition;

  double speed_degree_coef;
  double rpm_degree_coef;
};

const std::vector<std::pair<cv::MorphTypes, std::string>>
    transformation_type_array{
        //{cv::MORPH_CLOSE, "MORPH_CLOSE"}, // has worse results than
        // MORPH_CLOSE
        {cv::MORPH_TOPHAT, "MORPH_TOPHAT"}};

const double kDegreeToSpeedCoef = 1.75;
const double kDegreeToRPMCoefFor360= 0.035;
const double kDegreeToRPMCoefFor180= 0.044;
const int kZeroAnglePointAtAnalogMeter = 228;
const cv::Size kFrameSize = {1920, 1080};
const int kStartSpeedMeterZoneX = -615;
const int kStartSpeedMeterZoneY = -770;
const int kStartRPMMeterZoneX = -1900;
const int kStartRPMMeterZoneY = -770;
const int kStartTelltalesZoneX = -1800;
const int kStartTelltalesZoneY = -400;
const cv::Rect kAnalogSpeedMeterCoordinates{kStartSpeedMeterZoneX,
                                            kStartSpeedMeterZoneY, 590, 510};
const cv::Rect kAnalogRPMMeterCoordinates{kStartRPMMeterZoneX,
                                          kStartRPMMeterZoneY, 590, 510};
const cv::Rect kTelltalesPanelCoordinates{kStartTelltalesZoneX,
                                          kStartTelltalesZoneY, 1700, 300};
} // namespace settings
