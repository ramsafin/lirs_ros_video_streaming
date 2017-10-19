#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <cv_bridge/cv_bridge.h>

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

  ROS_INFO("Stream provider: %s", DEFAULT_DEVICE_NAME);

  lirs::V4L2Capture cap(DEFAULT_DEVICE_NAME);

  std::string camera_name = "camera";

  std::string frame_id = "camera_optical_frame";

  std::string camera_info_url;

  if (!cap.isOpened()) {
    ROS_ERROR_STREAM("Couldn't open the video device");
    return -1;
  }

  ROS_INFO("Opened stream, starting to publish");

  void* rawFrame;
  sensor_msgs::ImagePtr msg;
  sensor_msgs::CameraInfo cam_info_msg;
  std_msgs::Header header;
  header.frame_id = frame_id;
  camera_info_manager::CameraInfoManager cam_info_manager(nh, camera_name, camera_info_url);
  cam_info_msg = cam_info_manager.getCameraInfo();

  ros::Rate r(DEFAULT_FPS);

  while (nh.ok()) {

    if (pub.getNumSubscribers() > 0) {

      auto status = cap.mainloop();

      if (status) {

        rawFrame = cap.getCurrentFrameData();

        cv::Mat frame = cv::Mat(cap.getHeight(), cap.getWidth(), CV_8UC1, rawFrame, cap.getStep());

        msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BAYER_GRBG8, frame).toImageMsg();

        if (cam_info_msg.distortion_model == "") {
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
