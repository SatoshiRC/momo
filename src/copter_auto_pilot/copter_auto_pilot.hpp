#ifndef COPTER_AUTO_HPP
#define COPTER_AUTO_HPP

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <functional>
#include "mavsdk/mavsdk.h"
#include "mavsdk/plugins/action/action.h"
#include "mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h"
#include "mavsdk/plugins/mocap/mocap.h"
#include "mavsdk/plugins/telemetry/telemetry.h"
#include "mavlink_include.h"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

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

  void isAutoMode(bool arg) { isAutoMode_ = arg; }

 private:
  /*
    ARマーカーを検知するタスク
  */
  void handleImage_ARTag();

  /*
    白線を検知するタスク
  */
  void handleImage_Line(){};

  void estimatePosition();

  struct result_handleImage {
    mavsdk::Mocap::VisionPositionEstimate positionEstimate;
    bool isValid=false;
    std::mutex mutex_;
  };
  result_handleImage result_ARTag;
  result_handleImage result_Line;

    const cv::Mat cameraMatrix =
      (cv::Mat_<double>(3, 3) << 1062.2296, 0, 719.5, 0, 1062.2296, 539.5,
       0, 0, 1);
  const cv::Mat distCoeffs =
        (cv::Mat_<double>(1, 5) << -0.260092, 0.0655098, 0, 0, 0);
  const cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);



  std::shared_ptr<std::thread> thread_mainTask;
  std::shared_ptr<std::thread> thread_autoModeTask;

  cv::Mat imageForARtag;
  cv::Mat imageForLine;


  struct  {
    std::mutex mutex_autoModeTask;
    std::condition_variable cond_autoModeTask;
    bool isAutoMode_;
    bool operator==(bool arg) { return isAutoMode_ == arg; }
    void operator=(bool arg) { isAutoMode_ = arg; }
  } isAutoMode_;

  //For MAVSDK
  std::shared_ptr<mavsdk::Mavsdk> mavsdk;
  std::shared_ptr<mavsdk::Telemetry> telemetry;
  std::shared_ptr<mavsdk::Action> action;
  std::shared_ptr<mavsdk::Mocap> mocap;
  std::shared_ptr<mavsdk::System> system;
  std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough;

  enum class MAVLINK_MESSAGE_ID : uint16_t{
    HEARTBEAT = 0,
    RC_CHANNELS = 65,
    DISTANCE_SENSOR = 132,
    ATTITUDE = 30,
    ALTITUDE = 141,
    VISION_POSITION_ESTIMATE  = 102,
    SET_POSITION_TARGET_LOCAL_NED = 84,
    MAV_CMD_NAV_TAKEOFF_LOCAL = 24,
    MAV_CMD_NAV_LAND_LOCAL = 23,
  };
};

#endif  //COPTER_AUTO_HPP
