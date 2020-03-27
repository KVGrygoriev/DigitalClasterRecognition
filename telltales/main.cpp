#include <iostream>
#include <vector>

#include "TelltalesDetector.h"
#include "VideoManager.h"

int main() {
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
