#include "copter_auto_pilot/copter_auto_pilot.hpp"

copter_auto_pilot::copter_auto_pilot() : MARKER_SIZE(0.148) {
  imageForARtag = cv::Mat(480, 640, CV_8UC3);
  imageForLine = cv::Mat(480, 640, CV_8UC3);

  #ifdef IS_ENABLE_MAVLINK

  mavsdk.reset(new mavsdk::Mavsdk);

  mavsdk::ConnectionResult connection_result =
      mavsdk.get()->add_any_connection("tcp://192.168.137.29:5760");
  if (connection_result != mavsdk::ConnectionResult::Success) {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return;
  }

  struct {
    std::shared_ptr<mavsdk::System> operator()(mavsdk::Mavsdk& mavsdk) {
      std::cout << "Waiting to discover system...\n";
      auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
      auto fut = prom.get_future();

      // We wait for new systems to be discovered, once we find one that has an
      // autopilot, we decide to use it.
      mavsdk::Mavsdk::NewSystemHandle handle =
          mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
            auto system = mavsdk.systems().back();

            if (system->has_autopilot()) {
              std::cout << "Discovered autopilot\n";

              // Unsubscribe again as we only want to find one system.
              mavsdk.unsubscribe_on_new_system(handle);
              prom.set_value(system);
            }
          });

      // We usually receive heartbeats at 1Hz, therefore we should find a
      // system after around 3 seconds max, surely.
      if (fut.wait_for(std::chrono::seconds(3)) ==
          std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
      }

      // Get discovered system now.
      return fut.get();
    }
  } get_system;

  system = get_system(*mavsdk);

  telemetry.reset(new mavsdk::Telemetry(system));
  action.reset(new mavsdk::Action(system));
  mocap.reset(new mavsdk::Mocap(system));
  passthrough.reset(new mavsdk::MavlinkPassthrough(system));

  passthrough.get()->subscribe_message((uint16_t)MAVLINK_MESSAGE_ID::RC_CHANNELS,
                                       [this](const mavlink_message_t& message){
    this->isAutoMode(mavlink_msg_rc_channels_get_chan7_raw(&message) >= 1500);});

  telemetry.get()->subscribe_position([this](mavsdk::Telemetry::Position position) {
    std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
  });
  passthrough.get()->subscribe_message(
      (uint16_t)MAVLINK_MESSAGE_ID::DISTANCE_SENSOR,
      [this](const mavlink_message_t& message) {
        this->isAutoMode(mavlink_msg_rc_channels_get_chan7_raw(&message) >=
                         2000);
      });

  thread_mainTask.reset(new std::thread(&copter_auto_pilot::mainTask, this));
  thread_autoModeTask.reset(
      new std::thread(&copter_auto_pilot::autoModeTask, this));

  #endif
}

copter_auto_pilot::~copter_auto_pilot() {
  thread_autoModeTask.get()->detach();
  thread_autoModeTask.get()->~thread();

  thread_mainTask.get()->detach();
  thread_mainTask.get()->~thread();
}

void copter_auto_pilot::setImage(cv::Mat& sourceImage) {
  imageForARtag = sourceImage.clone();
  imageForLine = sourceImage.clone();

  //if (isAutoMode_ == true) {
  if (true) {
    std::thread(&copter_auto_pilot::estimatePosition, this).detach();
  }
}

void copter_auto_pilot::mainTask() {
  while (true) {
  }
}

void copter_auto_pilot::autoModeTask() {
  std::unique_lock<std::mutex> lock(isAutoMode_.mutex_autoModeTask);
  isAutoMode_.cond_autoModeTask.wait(lock, [this] { return this->isAutoMode_==true; });


  while (isAutoMode_==true) {

  }
  thread_autoModeTask.reset(
      new std::thread(&copter_auto_pilot::autoModeTask, this));
}

void copter_auto_pilot::estimatePosition() {
  std::thread ARTag = std::thread(&copter_auto_pilot::handleImage_ARTag, this);
  std::thread line = std::thread(&copter_auto_pilot::handleImage_Line, this);

  ARTag.join();
  if (result_ARTag.isValid) {
    position = result_ARTag.positionEstimate;
  } else {
    line.join();
    if (result_Line.isValid) {
      position = result_Line.positionEstimate;
    } else {
    
    }
#ifdef IS_ENABLE_MAVLINK
    mocap.get()->set_vision_position_estimate(position);
#endif
  }

 }

void copter_auto_pilot::handleImage_ARTag() {


  cv::Mat outputArry;
  std::vector<int> ids;  //uint8_t だとバグる
  std::vector<std::vector<cv::Point2f>> corners;

  cv::aruco::detectMarkers(imageForARtag, dictionary, corners, ids);

  // if at least one marker detected
  if (ids.size() > 0) {
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE, cameraMatrix,
                                         distCoeffs, rvecs, tvecs, outputArry);

    //camera座標 → marekr座標-------------------------------------------------------------------------------------------------------------------
    uint8_t kyoriGaTikai = 0;             //配列のn番目
    std::vector<float> relativeDistance;  //相対距離
    std::vector<cv::Mat> rvecs_Mat, tvecs_Mat, vector3_0, R, Rt,
        MarkerToCameraPosition;

    for (uint8_t j = 0; j < ids.size(); j++) {
      //初期化
      R.push_back((cv::Mat::eye(3, 3, CV_32F)));
      Rt.push_back((cv::Mat::eye(3, 3, CV_32F)));
      vector3_0.push_back((cv::Mat_<float>(1, 3) << 0, 0, 0));
      MarkerToCameraPosition.push_back((cv::Mat_<float>(1, 3) << 0, 0, 0));

      //rvecs,tvces --  std::vector → cv::Mat  (y,X)
      rvecs_Mat.push_back((cv::Mat_<float>(1, 3) << rvecs.at(j)[0],
                           rvecs.at(j)[1], rvecs.at(j)[2]));
      tvecs_Mat.push_back((cv::Mat_<float>(1, 3) << tvecs.at(j)[0],
                           tvecs.at(j)[1], tvecs.at(j)[2]));

      cv::Rodrigues(rvecs_Mat.at(j), Rt.at(j));
      R.at(j) = Rt.at(j).t();

      //エラー回避の関係で　縦ベクトル+他0の3*3
      tvecs_Mat.at(j) = (cv::Mat_<float>(3, 3) << tvecs.at(j)[0], 0, 0,
                         tvecs.at(j)[1], 0, 0, tvecs.at(j)[2], 0, 0);
      MarkerToCameraPosition.at(j) = (-R.at(j)) * (tvecs_Mat.at(j));
    }
    //-------------------------------------------------------------------------------------------------------------------------------------------
    //最小距離のmarker比較-----------------------------------------------------------------------------------------------------------------------
    float nearestRelativeDistance = 0;
    uint8_t nearestNumber = 0;
    for (uint8_t k = 0; k < ids.size(); k++) {
        float relativeDistance = std::pow(MarkerToCameraPosition.at(k).at<float>(1, 0), 2) +
           std::pow(MarkerToCameraPosition.at(k).at<float>(1, 0), 2) +
           std::pow(MarkerToCameraPosition.at(k).at<float>(2, 0), 2);
      //相対距離の二乗比較
      //relativeDistance.push_back(
      //    (std::pow(MarkerToCameraPosition.at(k).at<float>(1, 0), 2) +
      //     std::pow(MarkerToCameraPosition.at(k).at<float>(1, 0), 2) +
      //     std::pow(MarkerToCameraPosition.at(k).at<float>(2, 0), 2)));
      if ((k != 0) && (nearestRelativeDistance > relativeDistance)) {
        nearestNumber = k;
      }
    }
    //---------------------------------------------------------------------------------------------------------------------------------------------
    //マーカーidからマーカー固有のフィールド座標を付与  //フィールド座標=相対距離が最も近いマーカーidの座標+相対距離
    {
      std::unique_lock<std::mutex> lock(result_ARTag.mutex_);
      result_ARTag.positionEstimate.position_body.x_m =
          MarkerToCameraPosition.at(kyoriGaTikai).at<float>(0, 0);
      result_ARTag.positionEstimate.position_body.y_m =
          MarkerToCameraPosition.at(kyoriGaTikai).at<float>(1, 0);
      result_ARTag.positionEstimate.position_body.z_m =
          MarkerToCameraPosition.at(kyoriGaTikai).at<float>(2, 0);
      result_ARTag.positionEstimate.angle_body.roll_rad = std::atan2(
          static_cast<float>(-(R.at(kyoriGaTikai).at<float>(2, 1))),
          static_cast<float>(
              R.at(kyoriGaTikai).at<float>(2, 2)));  //コンパイルエラー対策
      result_ARTag.positionEstimate.angle_body.pitch_rad =
          std::asin(static_cast<float>(R.at(kyoriGaTikai).at<float>(2, 0)));
      result_ARTag.positionEstimate.angle_body.yaw_rad =
          std::atan2(static_cast<float>(-(R.at(kyoriGaTikai).at<float>(1, 0))),
                     static_cast<float>(R.at(kyoriGaTikai).at<float>(0, 0)));
      result_ARTag.isValid = true;
    }
    
    std::cout << "[x]=" << result_ARTag.positionEstimate.position_body.x_m
              << "\t";
    std::cout << "[y]=" << result_ARTag.positionEstimate.position_body.y_m
              << "\t";
    std::cout << "[z]=" << result_ARTag.positionEstimate.position_body.z_m
              << "\t";
    std::cout << "[roll]=" << result_ARTag.positionEstimate.angle_body.roll_rad
              << "\t";
    std::cout << "[pitch]="
              << result_ARTag.positionEstimate.angle_body.pitch_rad << "\t";
    std::cout << "[yaw]=" << result_ARTag.positionEstimate.angle_body.yaw_rad
              << "\n";
  } else {
    {
      std::unique_lock<std::mutex> lock(result_ARTag.mutex_);
      result_ARTag.isValid = false;
      return;
    }
  }
}
