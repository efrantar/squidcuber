#include "scan.h"

#include <thread>

void DoubleCam::open(cv::VideoCapture &cam, int id) {
  cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  if (!cam.open(id))
    throw std::runtime_error("Error opening camera.");
}

DoubleCam::DoubleCam(int uid, int did) : rec(false) {
  open(ucam, uid);
  open(dcam, did);
  ucam >> uframe;
  dcam >> dframe;
}

void DoubleCam::start() {
  if (rec) // no double start
    return;
  rec = true;

  thread = std::thread([&]() {
    cv::Mat uframe1;
    cv::Mat dframe1;
    while (rec) {
      rlock.lock();
      ucam >> uframe1;
      dcam >> dframe1;
      rlock.unlock(); // need to release the lock here to make stopping possible
      std::unique_lock<std::mutex> l2(wlock);
      uframe = std::move(uframe1); // we don't need the temporary frame buffers anymore
      dframe = std::move(dframe1);
    }
  });
}

void DoubleCam::stop() {
  {
    std::unique_lock<std::mutex> l1(wlock); // much easier to acquire
    std::unique_lock<std::mutex> l2(rlock);
    rec = false;
  } // release locks to actually terminate
  thread.join();
}

void DoubleCam::frame(cv::Mat& dst) {
  std::unique_lock<std::mutex> l(wlock);
  cv::hconcat(uframe, dframe, dst);
}

void extract_means(
  const cv::Mat& image, const std::vector<std::vector<cv::Rect>>& rects, std::vector<cv::Scalar>& res
) {
  res.resize(rects.size());
  for (int i = 0; i < rects.size(); i++) {
    res[i] = 0;
    for (int j = 0; j < rects[i].size(); j++) // can't foreach here for some reason
       res[i] += cv::mean(image(rects[i][j]));
    res[i] /= int(rects[i].size());
  }
}
