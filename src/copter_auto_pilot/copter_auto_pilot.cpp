#include "copter_auto_pilot/copter_auto_pilot.hpp"

copter_auto_pilot::copter_auto_pilot() {
  thread_ARTag = nullptr;
  thread_Line = nullptr;

  imageForARtag = cv::Mat(480, 640, CV_8UC3);
  imageForLine = cv::Mat(480, 640, CV_8UC3);

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
    this->isAutoMode(mavlink_msg_rc_channels_get_chan7_raw(&message) >= 2000);});

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
  cond_autoModeTask.wait(lock, [this] { return this->isAutoMode_; });
  while (isAutoMode_) {
  }
  thread_autoModeTask.reset(
      new std::thread(&copter_auto_pilot::autoModeTask, this));
}