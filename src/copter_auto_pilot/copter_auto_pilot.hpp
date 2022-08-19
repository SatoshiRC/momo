#ifndef COPTER_AUTO_HPP
#define COPTER_AUTO_HPP

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include "opencv2/opencv.hpp"

class copter_auto_pilot {
 public:
  copter_auto_pilot();
  ~copter_auto_pilot();
  /*
    SDL_Renderから画像を受け取る関数
    autoモードのときは画像処理のthreadを立てる．
  */
  void setImage(cv::Mat& image);

  /*
    常に実行するタスク
    @task1 telemtryを監視する
        自動モードに入ったかどうか
  */
  void mainTask();

  /*
    autoModeのときのみ実行するタスク
    @task1 自己位置推定の結果を送信する
    @task2 終了判定をする

    if(isAutoMode == false) なにもしない
  */
  void autoModeTask();

 private:
  /*
    ARマーカーを検知するタスク
  */
  void handleImage_ARTag();

  /*
    白線を検知するタスク
  */
  void handleImage_Line();

  std::mutex mutex_autoModeTask;
  std::condition_variable cond_autoModeTask;

  std::shared_ptr<std::thread> thread_ARTag;
  std::shared_ptr<std::thread> thread_Line;
  std::shared_ptr<std::thread> thread_mainTask;
  std::shared_ptr<std::thread> thread_autoModeTask;

  cv::Mat imageForARtag;
  cv::Mat imageForLine;

  bool isAutoMode;
};

#endif  //COPTER_AUTO_HPP