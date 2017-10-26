#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xphoto.hpp>

#include <sstream>
#include <boost/assign/list_of.hpp>

#include "test/video_test.h"

#include <chrono>

sensor_msgs::CameraInfo getDefaultCAMInfo(sensor_msgs::ImagePtr img){
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

  ros::init(argc, argv, "lirs_video_stream");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher pub = it.advertiseCamera("image_raw", 1);

  lirs::V4L2Capture cap("/dev/video1");

  std::string camera_name = "camera1";
  std::string frame_id = "cam1_frame_id";
  std::string camera_info_url;

  if (!cap.isOpened()) {
    ROS_ERROR_STREAM("Couldn't open the video device");
    return -1;
  }

  uchar* rawData;

  std_msgs::Header header;
  sensor_msgs::ImagePtr msg;
  sensor_msgs::CameraInfo cam_info_msg;

  camera_info_manager::CameraInfoManager cam_info_manager(nh, camera_name, camera_info_url);

  // init
  cam_info_msg = cam_info_manager.getCameraInfo();
  header.frame_id = frame_id;

  // to Image message
  msg = boost::make_shared<sensor_msgs::Image>();
  msg->height = cap.getHeight();
  msg->width  = cap.getWidth();
  msg->step   = cap.getStep();
  msg->is_bigendian = false;
  msg->encoding     = sensor_msgs::image_encodings::BAYER_GRBG8;

  msg->data.resize(cap.getStep() * cap.getHeight());

  header.frame_id = "camera_frame_id";
  msg->header = header;

  ROS_INFO("Opened stream, starting to publish");

  ros::Rate r(DEFAULT_FPS);

  while (nh.ok()) {

    if (pub.getNumSubscribers() > 0) {

      auto status = cap.mainloop();

      if (status) {

        rawData = (uchar*) cap.getCurrentFrameData();

        msg->data.assign(rawData, rawData + cap.getHeight() * cap.getStep()); // set ROS image data

        if (cam_info_msg.distortion_model.empty()) {
          cam_info_msg = getDefaultCAMInfo(msg);
          cam_info_manager.setCameraInfo(cam_info_msg);
        }

        pub.publish(*msg, cam_info_msg, ros::Time::now());

      }

      ros::spinOnce();

    }

    r.sleep();
  }
}
