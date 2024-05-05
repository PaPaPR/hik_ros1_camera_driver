#include "hik_ros1_camera_nodelet.hpp"

#include <pluginlib/class_list_macros.hpp>

void HikROS1CameraNodelet::onInit() {
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle private_nh = getPrivateNodeHandle();

  private_nh.getParam("camera_frame_id", frame_id_);
  private_nh.getParam("camera_info_path", cinfo_path_);
  private_nh.getParam("camera_type", camera_type_);
  private_nh.getParam("camera_serial", camera_serial_);

  cameraInit();

  cinfom_ = std::make_unique<camera_info_manager::CameraInfoManager>(nh);
  if (cinfom_->validateURL(cinfo_path_)) {
    if (cinfom_->loadCameraInfo(cinfo_path_)) is_cinfo_validated = true;
  }

  dynamic_reconfigure::Server<hik_ros1_camera_driver::CameraDynReconfConfig>
      dyn_reconf_server;
  dyn_reconf_server.setCallback(
      boost::bind(&HikROS1CameraNodelet::dynReconfCB, this, _1, _2));

  img_pub_ = nh.advertise<sensor_msgs::Image>("camera_image", 1);
  cinfo_pub_ = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

  imagePubLoop();
}

void HikROS1CameraNodelet::cameraInit() {
  camera_ = std::make_unique<hik_camera>(camera_type_, camera_serial_);
  ROS_INFO_STREAM("Camera type:" + camera_type_ + ", serial:" + camera_serial_ +
                  ", frame id:" + frame_id_);

  while (ros::ok()) {
    try {
      ROS_INFO_STREAM("Camera initializing.");
      camera_->initialize();
      camera_->startCapture();
      ROS_INFO_STREAM("Camera initialized.");
      break;
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM(e.what());
      ros::Rate(ros::Duration(1.)).sleep();
    }
  }
}

void HikROS1CameraNodelet::imagePubLoop() {
  int failed_retry_times{-1};

  while (ros::ok()) {
    ros::spinOnce();
    try {
      sensor_msgs::ImagePtr img(new sensor_msgs::Image);
      img->header.frame_id = frame_id_;
      img->header.stamp = ros::Time::now();
      auto frame = camera_->captureImage();
      img->height = frame->height;
      img->width = frame->width;
      img->step = frame->width * frame->channels;
      img->encoding = frame->format;
      img->data = std::move(frame->imageData);
      img_pub_.publish(img);

      if (is_cinfo_validated) {
        sensor_msgs::CameraInfo info = cinfom_->getCameraInfo();
        sensor_msgs::CameraInfoPtr cinfo =
            boost::make_shared<sensor_msgs::CameraInfo>(info);
        cinfo->header.stamp = img->header.stamp;
        cinfo->header.frame_id = frame_id_;
        cinfo_pub_.publish(cinfo);
      }
    } catch (const std::exception& e) {
      failed_retry_times++;
      ROS_ERROR_STREAM(e.what());
      if (failed_retry_times > 0)
        ROS_ERROR_STREAM("Camera capture image failed, retrying...");
      if (failed_retry_times >= 5) {
        ROS_ERROR_STREAM("Camera reinitializing...");
        callWithLog([&]() { camera_->stopCapture(); });
        callWithLog([&]() { camera_->release(); });
        cameraInit();
        failed_retry_times = 0;
      }
    }
  }
}

void HikROS1CameraNodelet::dynReconfCB(
    hik_ros1_camera_driver::CameraDynReconfConfig& _config, uint32_t _level) {
  callWithLog([&]() { camera_->setExposureAuto(_config.exposure_auto); });
  if (!callWithLog([&]() { return camera_->getExposureAuto(); }))
    callWithLog([&]() { camera_->setExposureTime(_config.exposure_time); });
  callWithLog([&]() { camera_->setGainAuto(_config.gain_auto); });
  if (!callWithLog([&]() { return camera_->getGainAuto(); }))
    callWithLog([&]() { camera_->setGain(_config.gain); });
  callWithLog([&]() { camera_->setGammaEnable(_config.gamma_enable); });
  if (callWithLog([&]() { return camera_->getGammaEnable(); }))
    callWithLog([&]() { camera_->setGamma(_config.gamma); });
}

PLUGINLIB_EXPORT_CLASS(HikROS1CameraNodelet, nodelet::Nodelet);