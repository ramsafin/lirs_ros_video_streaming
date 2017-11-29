#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sstream>
#include <boost/assign/list_of.hpp>

#include "lirs_video_stream/video_stream_api.hpp"

#include <chrono>

sensor_msgs::CameraInfo getDefaultCAMInfo(sensor_msgs::ImagePtr img){

    ROS_INFO("Get camera info");

    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = img->header.frame_id;
    // Fill image size
    cam_info_msg.height = img->height;
    cam_info_msg.width = img->width;
    // Add the most common distortion model as sensor_msgs/CameraInfo says
    cam_info_msg.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    cam_info_msg.D.resize(5, 0.0);
    // Give a reasonable default intrinsic camera matrix
    cam_info_msg.K = boost::assign::list_of(1.0) (0.0) (img->width/2.0)
                                           (0.0) (1.0) (img->height/2.0)
                                           (0.0) (0.0) (1.0);
    // Give a reasonable default rectification matrix
    cam_info_msg.R = boost::assign::list_of (1.0) (0.0) (0.0)
                                            (0.0) (1.0) (0.0)
                                            (0.0) (0.0) (1.0);
    // Give a reasonable default projection matrix
    cam_info_msg.P = boost::assign::list_of (1.0) (0.0) (img->width/2.0) (0.0)
                                            (0.0) (1.0) (img->height/2.0) (0.0)
                                           (0.0) (0.0) (1.0) (0.0);
    return cam_info_msg;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "lirs_video_stream_pulisher");

  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");

  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher pub = it.advertiseCamera("image", 1);

  std::string deviceName;

  if (_nh.getParam("device_name", deviceName)) {
    ROS_INFO_STREAM("Video stream device name: " << deviceName);
  } else {
    ROS_ERROR("Failed to get 'device_name' parameter");
  }

  std::string cameraName;

  _nh.param("camera_name", cameraName, std::string("camera"));
  ROS_INFO_STREAM("Camera name: " << cameraName);

  int fps;

  _nh.getParam("fps", fps);
  ROS_INFO_STREAM("Frame per second: " << fps);

  std::string frameId;

  _nh.getParam("frame_id", frameId);
  ROS_INFO_STREAM("Frame id: " << frameId);

  std::string cameraInfoUrl;

  _nh.getParam("camera_info_url", cameraInfoUrl);
  ROS_INFO_STREAM("Camera info url: " << cameraInfoUrl);

  int width;
  int height;

  _nh.getParam("width", width);
  _nh.getParam("height", height);
  ROS_INFO_STREAM("Image width x height: " << width << " x " << height);

  std::string imageFormat;

  _nh.getParam("image_format", imageFormat);
  ROS_INFO_STREAM("Image fromat: " << imageFormat);

  lirs::V4L2Capture cap(deviceName);

  if (!cap.isOpened()) {
    ROS_ERROR_STREAM("Couldn't open the video device: " << deviceName);
    return -1;
  }

  uint8_t* rawData;

  std_msgs::Header header;
  sensor_msgs::ImagePtr msg;
  sensor_msgs::CameraInfo cam_info_msg;

  camera_info_manager::CameraInfoManager cam_info_manager(nh, cameraName, cameraInfoUrl);

  // init
  cam_info_msg = cam_info_manager.getCameraInfo();
  header.frame_id = frameId;

  // to Image message
  msg = boost::make_shared<sensor_msgs::Image>();
  msg->height = cap.getHeight();
  msg->width  = cap.getWidth();
  msg->step   = cap.getStep();
  msg->is_bigendian = false;
  msg->encoding     = imageFormat;

  msg->data.resize(cap.getStep() * cap.getHeight());
  msg->header = header;

  ROS_INFO("Opened stream, starting to publish");

  ros::Rate r(fps);

  while (nh.ok()) {

    if (pub.getNumSubscribers() > 0) {

      auto status = cap.mainloop();

      if (status) {

        rawData = (uint8_t*) cap.getCurrentFrameData();

        msg->data.assign(rawData, rawData + cap.getHeight() * cap.getStep());

        if (cam_info_msg.distortion_model.empty()) {
          cam_info_msg = getDefaultCAMInfo(msg);
          cam_info_manager.setCameraInfo(cam_info_msg);
          ROS_INFO("Warning");
        }

        pub.publish(*msg, cam_info_msg, ros::Time::now());

      }

      ros::spinOnce();

    }

    r.sleep();
  }
}
