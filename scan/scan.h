/**
 * Utilities for accessing the webcams and extracting facelet color values.
 */

#ifndef __SCAN__
#define __SCAN__

#include <thread>
#include <opencv2/opencv.hpp>
#include "match.h"

class DoubleCam {
  cv::VideoCapture ucam;
  cv::VideoCapture dcam;
  cv::Mat uframe;
  cv::Mat dframe;
  std::thread thread;

  // Much faster (safe) stopping and frame reading with a 2-lock system
  std::mutex rlock;
  std::mutex wlock;
  bool rec;

  static void open(cv::VideoCapture& cam, int id);

  public:
    DoubleCam(int uid, int did);
    void start();
    void stop();
    void frame(cv::Mat& dst);
};

void extract_means(
  const cv::Mat& image, const std::vector<std::vector<cv::Rect>>& rects, std::vector<cv::Scalar>& res
);

#endif
