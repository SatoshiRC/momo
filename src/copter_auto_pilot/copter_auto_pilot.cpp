#include "copter_auto_pilot/copter_auto_pilot.hpp"

copter_auto_pilot::copter_auto_pilot() {
  thread_ARTag = nullptr;
  thread_Line = nullptr;

  imageForARtag = cv::Mat(480, 640, CV_8UC3);
  imageForLine = cv::Mat(480, 640, CV_8UC3);

  thread_mainTask.reset(new std::thread(&copter_auto_pilot::mainTask, this));
  thread_autoModeTask.reset(
      new std::thread(&copter_auto_pilot::autoModeTask, this));
}

copter_auto_pilot::~copter_auto_pilot() {
  thread_autoModeTask.get()->detach();
  thread_autoModeTask.get()->~thread();

  thread_ARTag.get()->~thread();
  thread_Line.get()->~thread();
  thread_mainTask.get()->detach();
  thread_mainTask.get()->~thread();
}

void copter_auto_pilot::setImage(cv::Mat& sourceImage) {
  imageForARtag = sourceImage.clone();
  imageForLine = sourceImage.clone();

  thread_ARTag.reset(
      new std::thread(&copter_auto_pilot::handleImage_ARTag, this));
  thread_ARTag.reset(
      new std::thread(&copter_auto_pilot::handleImage_Line, this));

  thread_ARTag.get()->detach();
  thread_Line.get()->detach();
}

void copter_auto_pilot::mainTask() {
  while (true) {
  }
}

void copter_auto_pilot::autoModeTask() {
  std::unique_lock<std::mutex> lock(mutex_autoModeTask);
  cond_autoModeTask.wait(lock, [this] { return this->isAutoMode; });
  while (isAutoMode) {
  }
  thread_autoModeTask.reset(
      new std::thread(&copter_auto_pilot::autoModeTask, this));
}