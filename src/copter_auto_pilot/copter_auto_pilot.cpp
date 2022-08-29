#include "copter_auto_pilot/copter_auto_pilot.hpp"

copter_auto_pilot::copter_auto_pilot() {
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

  std::thread(&copter_auto_pilot::handleImage_ARTag, this).detach();
  std::thread(&copter_auto_pilot::handleImage_Line, this).detach();
}

void copter_auto_pilot::mainTask() {
  while (true) {
  }
}

void copter_auto_pilot::autoModeTask() {
  std::unique_lock<std::mutex> lock(isAutoMode_.mutex_autoModeTask);
  isAutoMode_.cond_autoModeTask.wait(lock, [this] { return this->isAutoMode_==true; });

  std::thread(&copter_auto_pilot::estimatePosition, this);
  while (isAutoMode_==true) {

  }
  thread_autoModeTask.reset(
      new std::thread(&copter_auto_pilot::autoModeTask, this));
}

void copter_auto_pilot::estimatePosition() {
  while (isAutoMode_==true) {
  }
 }

void copter_auto_pilot::handleImage_ARTag() {
  cv::Mat outputArry;

  std::vector<int> ids;

  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(imageForARtag, dictionary, corners, ids);

  // if at least one marker detected
  if (ids.size() > 0) {
    if (imageForARtag.rows == 480) {
      imageSize = (uint8_t)IMAGE_SIZE::VGA;
    }
    cv::aruco::drawDetectedMarkers(imageForARtag, corners, ids);
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, 0.200,
                                         cameraMatrix[imageSize], distCoeffs[imageSize], rvecs,
        tvecs,
                                         outputArry);

    // draw axis for each marker
    for (int i = 0; i < ids.size(); i++)
      cv::drawFrameAxes(imageForARtag, cameraMatrix[imageSize],
                        distCoeffs[imageSize],
                        rvecs[i],
                        tvecs[i],
                        0.1);  //evecs回転　tvecs並進

    //rvecs,tvces --  std::vector → cv::Mat  (y,x)

    cv::Mat rvecs_Mat = (cv::Mat_<float>(1, 3) << rvecs.at(0)[0],
                         rvecs.at(0)[1], rvecs.at(0)[2]);
    cv::Mat tvecs_Mat = (cv::Mat_<float>(1, 3) << tvecs.at(0)[0],
                         tvecs.at(0)[1], tvecs.at(0)[2]);
    cv::Mat vector3_0 = (cv::Mat_<float>(1, 3) << 0, 0, 0);

    //初期化
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F), Rt = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat MarkerToCameraPosition;  // = (cv::Mat_<float>(1, 3) << 0, 0, 0);

    //ロドリゲス変換 カメラ座標系のrvecsをマーカー座標系に変換------------------------------------------------------
    cv::Rodrigues(rvecs_Mat, Rt);
    R = Rt.t();

    //エラー回避の関係で　縦ベクトル+他0の3*3
    tvecs_Mat = (cv::Mat_<float>(3, 3) << tvecs.at(0)[0], 0, 0, tvecs.at(0)[1],
                 0, 0, tvecs.at(0)[2], 0, 0);

    MarkerToCameraPosition = (-R) * (tvecs_Mat);


    {
        std::unique_lock<std::mutex> lock(result_ARTag.mutex_);
        result_ARTag.positionEstimate.position_body.x_m = MarkerToCameraPosition.at<float>(0, 0);
        result_ARTag.positionEstimate.position_body.y_m =
            MarkerToCameraPosition.at<float>(1, 0);
        result_ARTag.positionEstimate.position_body.z_m =
            MarkerToCameraPosition.at<float>(2, 0);
        result_ARTag.positionEstimate.angle_body.roll_rad = std::atan2(
            static_cast<float>(-(R.at<float>(2, 1))),
            static_cast<float>(-R.at<float>(2, 2)));  //コンパイルエラー対策
        result_ARTag.positionEstimate.angle_body.pitch_rad =
            std::asin(static_cast<float>(R.at<float>(2, 0)));
        result_ARTag.positionEstimate.angle_body.yaw_rad =
            std::atan2(static_cast<float>(-(R.at<float>(1, 0))),
            static_cast<float>(R.at<float>(0, 0)));
    }
    //チェック
    if (tvecs.size() > 0) {

      std::cout << "[x]=" << result_ARTag.positionEstimate.position_body.x_m
                << "\t";
      std::cout << "[y]=" << result_ARTag.positionEstimate.position_body.y_m
                << "\t";
      std::cout << "[z]=" << result_ARTag.positionEstimate.position_body.z_m
                << "\t";
      std::cout << "[roll]=" << result_ARTag.positionEstimate.angle_body.roll_rad
                << "\t";
      std::cout << "[pitch]=" << result_ARTag.positionEstimate.angle_body.pitch_rad
                << "\t";
      std::cout << "[yaw]=" << result_ARTag.positionEstimate.angle_body.yaw_rad
                << "\n";
      //std::cout << MarkerToCameraPosition << std::endl;
    }
  }
}
