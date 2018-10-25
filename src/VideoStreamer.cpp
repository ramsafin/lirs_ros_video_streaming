#include <linux/videodev2.h>

#include <string>
#include <sstream>
#include <optional>
#include <boost/assign/list_of.hpp>

#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include "lirs_ros_video_streaming/V4L2VideoCapture.hpp"

namespace lirs {
    namespace ros_utils {
        static sensor_msgs::CameraInfo defaultCameraInfoFrom(sensor_msgs::ImagePtr img) {
            sensor_msgs::CameraInfo cam_info_msg;
            cam_info_msg.header.frame_id = img->header.frame_id;
            cam_info_msg.width = img->width;
            cam_info_msg.height = img->height;
            cam_info_msg.distortion_model = "plumb_bob";
            cam_info_msg.D.resize(5, 0.0);
            cam_info_msg.K = boost::assign::list_of(1.0)(0.0)(img->width / 2.0)
                    (0.0)(1.0)(img->height / 2.0)
                    (0.0)(0.0)(1.0);
            cam_info_msg.R = boost::assign::list_of(1.0)(0.0)(0.0)
                    (0.0)(1.0)(0.0)
                    (0.0)(0.0)(1.0);
            cam_info_msg.P = boost::assign::list_of(1.0)(0.0)(img->width / 2.0)(0.0)
                    (0.0)(1.0)(img->height / 2.0)(0.0)
                    (0.0)(0.0)(1.0)(0.0);
            return cam_info_msg;
        }

        // NOTE: Add other image format correspondences if it is necessary.
        static std::optional<uint32_t> findCorrespondentV4l2PixFmt(std::string const &imageFormat) {
            if (imageFormat == sensor_msgs::image_encodings::YUV422)
                return std::optional{V4L2_PIX_FMT_YUYV};
            if (imageFormat == sensor_msgs::image_encodings::BAYER_GRBG8)
                return std::optional{V4L2_PIX_FMT_SGRBG8};
            if (imageFormat == sensor_msgs::image_encodings::BAYER_GBRG8)
                return std::optional{V4L2_PIX_FMT_SGBRG8};
            if (imageFormat == sensor_msgs::image_encodings::BAYER_BGGR8)
                return std::optional{V4L2_PIX_FMT_SBGGR8};
            if (imageFormat == sensor_msgs::image_encodings::BAYER_RGGB8)
                return std::optional{V4L2_PIX_FMT_SRGGB8};
            return std::nullopt;
        }

        static bool checkImageFormat(std::string const &imageFormat) {
            if (!(sensor_msgs::image_encodings::isMono(imageFormat)
                  || sensor_msgs::image_encodings::isBayer(imageFormat)
                  || sensor_msgs::image_encodings::isColor(imageFormat)
                  || imageFormat == sensor_msgs::image_encodings::YUV422)) {
                ROS_ERROR_STREAM("Given ROS image format: " << imageFormat << " does not supported!");
                return false;
            }
            return true;
        }

        static sensor_msgs::ImagePtr imageMessageFrom(std::string const &frameId, std::string const &imageFormat,
                                                      lirs::VideoCapture const &capture) {
            auto imageMsg = boost::make_shared<sensor_msgs::Image>();
            imageMsg->header.frame_id = frameId;
            imageMsg->width = capture.Get(lirs::CaptureParam::WIDTH);
            imageMsg->height = capture.Get(lirs::CaptureParam::HEIGHT);
            imageMsg->is_bigendian = 0;

            // YUV422 supports only for UYVY format,
            // thus images will be converted into greyscale (conversion into RGB/BGR is computation demanded)
            if (imageFormat == sensor_msgs::image_encodings::YUV422
                && capture.Get(lirs::CaptureParam::PIX_FMT) != V4L2_PIX_FMT_UYVY) {
                imageMsg->step = imageMsg->width;  // 1 byte pixel (depth)
                imageMsg->encoding = sensor_msgs::image_encodings::MONO8;
                imageMsg->data.reserve(imageMsg->step * imageMsg->height);
            } else {
                imageMsg->encoding = imageFormat;
                imageMsg->step = capture.imageStep();
                imageMsg->data.reserve(capture.imageSize());
            }

            return imageMsg;
        }

    }  // namespace ros_utils
}  // namespace lirs

int main(int argc, char **argv) {
    using std::string_literals::operator ""s;

    ros::init(argc, argv, "lirs_ros_video_streaming");

    ros::NodeHandle nodeHandle;
    ros::NodeHandle nodeHandle_{"~"};

    auto imageTransport = image_transport::ImageTransport{nodeHandle};
    auto publisher = imageTransport.advertiseCamera("image", 4);

    // get and validate capture parameters

    std::string deviceName;
    std::string cameraName;
    std::string frameId;
    std::string cameraInfoUrl;

    int width;
    int height;
    int frameRate;
    std::string imageFormat;

    nodeHandle_.param("device_name", deviceName, "/dev/video0"s);
    nodeHandle_.param("camera_name", cameraName, "camera"s);
    nodeHandle_.param("frame_id", frameId, "frame_id"s);
    nodeHandle_.param("camera_info_url", cameraInfoUrl, ""s);
    nodeHandle_.param("width", width, static_cast<int>(lirs::defaults::DEFAULT_WIDTH));
    nodeHandle_.param("height", height, static_cast<int> (lirs::defaults::DEFAULT_HEIGHT));
    nodeHandle_.param("fps", frameRate, static_cast<int>(lirs::defaults::DEFAULT_FRAME_RATE));
    nodeHandle_.param("image_format", imageFormat, sensor_msgs::image_encodings::YUV422);

    // checking image format

    if (!lirs::ros_utils::checkImageFormat(imageFormat)) {
        return -1;
    }

    auto pixFormat = lirs::ros_utils::findCorrespondentV4l2PixFmt(imageFormat);

    if (!pixFormat) {
        ROS_ERROR_STREAM("No corresponding v4l2 pixel format found for the given image format: " << imageFormat);
        return -1;
    }

    lirs::V4L2Capture capture(deviceName, *pixFormat, static_cast<uint32_t>(width), static_cast<uint32_t>(height),
                              static_cast<uint32_t>(frameRate));

    if (!capture.IsOpened()) {
        ROS_ERROR_STREAM("Couldn't open the video device: " << deviceName);
        return -1;
    }

    if (!capture.StartStreaming()) {
        ROS_ERROR_STREAM("Couldn't start streaming on: " << deviceName);
        return -1;
    }

    auto cameraInfoManager = camera_info_manager::CameraInfoManager(nodeHandle, cameraName, cameraInfoUrl);
    auto cameraInfoMsg = cameraInfoManager.getCameraInfo();

    ros::Rate rate(capture.Get(lirs::CaptureParam::FPS));

    // NOTE: Image message format may differ from the image format (see imageMessageFrom() method).
    auto imageMsg = lirs::ros_utils::imageMessageFrom(frameId, imageFormat, capture);

    while (ros::ok()) {
        if (publisher.getNumSubscribers() >= 0) {
            // no cameraInfoUrl is provided
            if (cameraInfoMsg.distortion_model.empty()) {
                cameraInfoMsg = lirs::ros_utils::defaultCameraInfoFrom(imageMsg);
                cameraInfoManager.setCameraInfo(cameraInfoMsg);
            }

            if (auto frame = capture.ReadFrame(); frame.has_value()) {
                if (imageFormat == sensor_msgs::image_encodings::YUV422) {
                    // convert into greyscale
                    cv::Mat rawImage(capture.Get(lirs::CaptureParam::HEIGHT),
                                     capture.Get(lirs::CaptureParam::WIDTH), CV_8UC2);

                    rawImage.data = frame->data().data();  // no copy

                    cv::Mat greyscale;

                    cv::cvtColor(rawImage, greyscale, cv::COLOR_YUV2GRAY_YUYV, 1);  // copy

                    imageMsg->data.assign(greyscale.data, greyscale.data + greyscale.rows * greyscale.cols);  // copy

                } else {
                    imageMsg->data = std::move(frame->data());
                }

                // TODO (Ramil Safin): Use frame's timestamp.
                publisher.publish(*imageMsg, cameraInfoMsg, ros::Time::now());
            }

            ros::spinOnce();
        }

        rate.sleep();
    }
}
