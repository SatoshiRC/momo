#include <atomic>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// WebRTC
#include <rtc_base/log_sinks.h>
#include <rtc_base/string_utils.h>

#if USE_SCREEN_CAPTURER
#include "rtc/screen_video_capturer.h"
#endif

#if defined(__APPLE__)
#include "mac_helper/mac_capturer.h"
#elif defined(__linux__)
#if USE_MMAL_ENCODER
#include "hwenc_mmal/mmal_v4l2_capturer.h"
#elif USE_JETSON_ENCODER
#include "hwenc_jetson/jetson_v4l2_capturer.h"
#elif USE_NVCODEC_ENCODER
#include "hwenc_nvcodec/nvcodec_v4l2_capturer.h"
#endif
#include "v4l2_video_capturer/v4l2_video_capturer.h"
#else
#include "rtc/device_video_capturer.h"
#endif

#include "serial_data_channel/serial_data_manager.h"

#if USE_SDL2
#include "sdl_renderer/sdl_renderer.h"
#endif

#include "ayame/ayame_client.h"
#include "metrics/metrics_server.h"
#include "p2p/p2p_server.h"
#include "rtc/rtc_manager.h"
#include "sora/sora_client.h"
#include "sora/sora_server.h"
#include "util.h"

#ifdef _WIN32
#include <rtc_base/win/scoped_com_initializer.h>
#endif

#if defined(__linux__) && USE_NVCODEC_ENCODER
#include "cuda/cuda_context.h"
#endif

const size_t kDefaultMaxLogFileSize = 10 * 1024 * 1024;

int main(int argc, char* argv[]) {
#ifdef _WIN32
  webrtc::ScopedCOMInitializer com_initializer(
      webrtc::ScopedCOMInitializer::kMTA);
  if (!com_initializer.Succeeded()) {
    std::cerr << "CoInitializeEx failed" << std::endl;
    return 1;
  }
#endif

  MomoArgs args;

  bool use_test = false;
  bool use_ayame = false;
  bool use_sora = false;
  int log_level = rtc::LS_NONE;

  Util::ParseArgs(argc, argv, use_test, use_ayame, use_sora, log_level, args);
  args.use_sdl = true;
  args.no_audio_device = true;
  args.no_video_device = true;

  rtc::LogMessage::LogToDebug((rtc::LoggingSeverity)log_level);
  rtc::LogMessage::LogTimestamps();
  rtc::LogMessage::LogThreads();

  std::unique_ptr<rtc::FileRotatingLogSink> log_sink(
      new rtc::FileRotatingLogSink("./", "webrtc_logs", kDefaultMaxLogFileSize,
                                   10));
  if (!log_sink->Init()) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to open log file";
    log_sink.reset();
    return 1;
  }
  rtc::LogMessage::AddLogToStream(log_sink.get(), rtc::LS_INFO);

#if USE_NVCODEC_ENCODER
  std::shared_ptr<CudaContext> cuda_context;
  try {
    cuda_context = CudaContext::Create();
  } catch (...) {
  }
#endif

  auto capturer = ([&]() -> rtc::scoped_refptr<ScalableVideoTrackSource> {
    if (args.no_video_device) {
      return nullptr;
    }

    auto size = args.GetSize();

    return DeviceVideoCapturer::Create(size.width, size.height, args.framerate,
                                       args.video_device);
  })();

  if (!capturer && !args.no_video_device) {
    std::cerr << "failed to create capturer" << std::endl;
    return 1;
  }

  RTCManagerConfig rtcm_config;
  rtcm_config.insecure = args.insecure;

  rtcm_config.no_video_device = args.no_video_device;
  rtcm_config.no_audio_device = args.no_audio_device;

  rtcm_config.fixed_resolution = args.fixed_resolution;
  rtcm_config.show_me = args.show_me;
  rtcm_config.simulcast = args.sora_simulcast;
  rtcm_config.hardware_encoder_only = args.hw_mjpeg_decoder;

  rtcm_config.disable_echo_cancellation = args.disable_echo_cancellation;
  rtcm_config.disable_auto_gain_control = args.disable_auto_gain_control;
  rtcm_config.disable_noise_suppression = args.disable_noise_suppression;
  rtcm_config.disable_highpass_filter = args.disable_highpass_filter;
  rtcm_config.disable_residual_echo_detector =
      args.disable_residual_echo_detector;

  rtcm_config.vp8_encoder = args.vp8_encoder;
  rtcm_config.vp8_decoder = args.vp8_decoder;
  rtcm_config.vp9_encoder = args.vp9_encoder;
  rtcm_config.vp9_decoder = args.vp9_decoder;
  rtcm_config.av1_encoder = args.av1_encoder;
  rtcm_config.av1_decoder = args.av1_decoder;
  rtcm_config.h264_encoder = args.h264_encoder;
  rtcm_config.h264_decoder = args.h264_decoder;

  rtcm_config.priority = args.priority;

#if USE_NVCODEC_ENCODER
  rtcm_config.cuda_context = cuda_context;
#endif

  rtcm_config.proxy_url = args.proxy_url;
  rtcm_config.proxy_username = args.proxy_username;
  rtcm_config.proxy_password = args.proxy_password;

#if USE_SDL2
  std::unique_ptr<SDLRenderer> sdl_renderer = nullptr;
  if (args.use_sdl) {
    sdl_renderer.reset(new SDLRenderer(args.window_width, args.window_height,
                                       args.fullscreen));
  }

  std::unique_ptr<RTCManager> rtc_manager(new RTCManager(
      std::move(rtcm_config), std::move(capturer), sdl_renderer.get()));
#else
  std::unique_ptr<RTCManager> rtc_manager(
      new RTCManager(std::move(rtcm_config), std::move(capturer), nullptr));
#endif

  {
    boost::asio::io_context ioc{1};
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type>
        work_guard(ioc.get_executor());

    std::shared_ptr<RTCDataManager> data_manager = nullptr;
    if (!args.serial_device.empty()) {
      data_manager = std::shared_ptr<RTCDataManager>(
          SerialDataManager::Create(ioc, args.serial_device, args.serial_rate)
              .release());
      if (!data_manager) {
        return 1;
      }
      rtc_manager->AddDataManager(data_manager);
    }

    boost::asio::signal_set signals(ioc, SIGINT, SIGTERM);
    signals.async_wait(
        [&](const boost::system::error_code&, int) { ioc.stop(); });

    std::shared_ptr<SoraClient> sora_client;
    std::shared_ptr<AyameClient> ayame_client;
    std::shared_ptr<P2PServer> p2p_server;

    MetricsServerConfig metrics_config;
    std::shared_ptr<StatsCollector> stats_collector;

    if (use_ayame) {
      AyameClientConfig config;
      config.insecure = args.insecure;
      config.no_google_stun = args.no_google_stun;
      config.client_cert = args.client_cert;
      config.client_key = args.client_key;
      config.signaling_url = args.ayame_signaling_url;
      config.room_id = args.ayame_room_id;
      config.client_id = args.ayame_client_id;
      config.signaling_key = args.ayame_signaling_key;

      ayame_client =
          AyameClient::Create(ioc, rtc_manager.get(), std::move(config));
      ayame_client->Connect();

      stats_collector = ayame_client;
    }

    sdl_renderer->SetDispatchFunction([&ioc](std::function<void()> f) {
      if (ioc.stopped())
        return;
      boost::asio::dispatch(ioc.get_executor(), f);
    });

    //ここからウインドウに画像を表示する
    ioc.run();

    //ここでウインドウ削除時の処理
    sdl_renderer->SetDispatchFunction(nullptr);
  }

  //この順番は綺麗に落ちるけど、あまり安全ではない
#if USE_SDL2
  sdl_renderer = nullptr;
#endif
  rtc_manager = nullptr;

  return 0;
}
