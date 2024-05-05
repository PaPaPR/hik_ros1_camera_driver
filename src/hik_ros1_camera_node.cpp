#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hik_ros1_camera_node");

  nodelet::Loader manager(false);
  nodelet::M_string remappings(ros::names::getRemappings());
  nodelet::V_string my_argv(argv + 1, argv + argc);

  manager.load(ros::this_node::getName(), "hik_ros1_camera_driver/HikROS1CameraNodelet", remappings, my_argv);

  ros::spin();
  return 0;
}