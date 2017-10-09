#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
#include <boost/assign/list_of.hpp>

#include "test/video_test.h"

sensor_msgs::CameraInfo getDefaultCAMInfo(sensor_msgs::ImagePtr img){
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = img->header.frame_id;
    // Fill image size
    cam_info_msg.height = img->height;
    cam_info_msg.width = img->width;
    ROS_INFO_STREAM("The image width is: " << img->width);
    ROS_INFO_STREAM("The image height is: " << img->height);
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
  ros::init(argc, argv, "publisher");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<sensor_msgs::Image>("image_raw", 64);

  lirs::V4L2Capture capture(DEFAULT_DEVICE_NAME);

  ros::Rate loop_rate(DEFAULT_FPS);

  while (ros::ok())
  {
    sensor_msgs::Image img;

    auto status = capture.mainloop();

    if (status) {

      auto frame = capture.currentFrame();

      std::vector<uint8_t> _data;

      _data.assign((uint8_t*)frame.data, (uint8_t*) frame.data + frame.length);

      img.data = _data;
      img.encoding = sensor_msgs::image_encodings::YUV422;
      img.height = capture.getHeight();
      img.width = capture.getWidth();
      img.is_bigendian = 0;
      img.step = capture.getStep();
      img.header.frame_id = "optical_frame_id";
      img.header.seq = capture.getSequence();
      img.header.stamp.sec = capture.getTimestamp().tv_sec;
      img.header.stamp.nsec = capture.getTimestamp().tv_usec;

      chatter_pub.publish(img);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
