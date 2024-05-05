#pragma once
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <hik_ros1_camera_driver/CameraDynReconfConfig.h>

#include "hik_ros1_camera.hpp"

using hik_camera = HikvisionCameraWrapper::HikvisionIndustrialCamera;

class HikROS1CameraNodelet : public nodelet::Nodelet {
 private:
  std::string frame_id_ {"camera_link"};
  std::string cinfo_path_;
  std::string camera_type_ {"USB"};
  std::string camera_serial_;
  bool is_cinfo_validated {false};

  ros::Publisher img_pub_;
  ros::Publisher cinfo_pub_;

  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfom_;
  std::unique_ptr<hik_camera> camera_;

 public:
  HikROS1CameraNodelet();
  ~HikROS1CameraNodelet();
  virtual void onInit();
  void cameraInit();
  void imagePubLoop();
  void dynReconfCB(hik_ros1_camera_driver::CameraDynReconfConfig &config,
                   uint32_t level);
};

HikROS1CameraNodelet::HikROS1CameraNodelet() {}

HikROS1CameraNodelet::~HikROS1CameraNodelet() {}

template <typename Func, typename... Args>
auto callWithLog(Func func, Args&&... args) -> decltype(func(args...)) {
  try {
    return func(std::forward<Args>(args)...);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    return decltype(func(args...))(-1);
  }
}
