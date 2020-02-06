#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

#include "match.h"
#include "scan.h"

const std::string RECTFILE = "scan.rects";

// For some reasons these IDs have flipped recently ...
const int UID = 2;
const int DID = 0;

int main() {
  init_match();
  std::vector<std::vector<cv::Rect>> rects(N_FACELETS);

  std::ifstream f(RECTFILE);
  std::string l;
  for (int i = 0; i < N_FACELETS; i++) {
    if (!std::getline(f, l))
      break;
    std::istringstream line(l);
    while (line) {
      cv::Rect r;
      line >> r.x >> r.y >> r.width >> r.height;
      rects[i].push_back(r);
    }
  }
  for (std::vector<cv::Rect>& rs : rects) {
    if (rs.empty()) {
      std::cout << "Invalid `scan.rects`." << std::endl;
      return 0;
    }
  }

  DoubleCam cam(UID, DID);

  std::string cmd;
  cv::Mat frame;
  int bgrs[N_FACELETS][3];

  while (std::cin) {
    std::cout << "Ready!" << std::endl;
    std::cin >> cmd;

    if (cmd == "start")
      cam.start();
    else if (cmd == "stop")
      cam.stop();
    else if (cmd == "scan") {
      cam.frame(frame);
      std::vector<cv::Scalar> means;
      extract_means(frame, rects, means);
      for (int i = 0; i < N_FACELETS; i++) {
        for (int j = 0; j < 3; j++)
          bgrs[i][j] = means[i][j];
      }
      std::string facecube = match_colors(bgrs);
      std::cout << ((facecube == "") ? "Scan Error." : facecube) << std::endl;
    } else if (cmd == "save") {
      std::string file;
      std::cin >> file;
      cam.frame(frame);
      cv::imwrite(file, frame);
    } else
      std::cout << "Error." << std::endl;
  }

  return 0;
}
