/*
 *  MIT License
 *
 *  Copyright (c) 2018 Laboratory of Intelligent Robotic Systems,
 *                     Higher Institute of Information Technology and Intelligent Systems,
 *                     Kazan Federal University
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *  Created by Ramil Safin <safin.ramil@it.kfu.ru> on 02.11.2018.
 */

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

using std::string_literals::operator ""s;

namespace lirs {
    namespace ros_utils {

        /* defaults */
        constexpr auto DEFAULT_DEVICE_NAME = "/dev/video0";
        constexpr auto DEFAULT_CAMERA_NAME = "camera";
        constexpr auto DEFAULT_CAMERA_INFO_URL = "";
        constexpr auto DEFAULT_FRAME_ID = "camera_frame_id";

        constexpr auto DEFAULT_FRAME_RATE = 30;
        constexpr auto DEFAULT_FRAME_WIDTH = 640;
        constexpr auto DEFAULT_FRAME_HEIGHT = 480;
        constexpr auto DEFAULT_IMAGE_FORMAT = "yuv422";

        static sensor_msgs::CameraInfo defaultCameraInfoFrom(sensor_msgs::ImagePtr const &img) {
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
                ROS_ERROR_STREAM("Given ROS image format: " << imageFormat << " is not supported!");
                return false;
            }
            return true;
        }

        static sensor_msgs::ImagePtr imageMessageFrom(std::string const &frameId, std::string const &imageFormat,
                                                      lirs::VideoCapture const &capture) {

            auto imageMsg = boost::make_shared<sensor_msgs::Image>();
            imageMsg->header.frame_id = frameId;
            imageMsg->width = static_cast<uint32_t >(capture.Get(lirs::CaptureParam::FRAME_WIDTH));
            imageMsg->height = static_cast<uint32_t >(capture.Get(lirs::CaptureParam::FRAME_HEIGHT));
            imageMsg->is_bigendian = 0;

            // YUV422 represents UYVY (not YUYV),
            // thus images will be converted into grayscale
            if (imageFormat == sensor_msgs::image_encodings::YUV422
                && capture.Get(lirs::CaptureParam::V4L2_PIX_FMT) != V4L2_PIX_FMT_UYVY) {
                imageMsg->step = imageMsg->width;  // 1 byte pixel (depth)
                imageMsg->encoding = sensor_msgs::image_encodings::MONO8;
                imageMsg->data.reserve(imageMsg->step * imageMsg->height);
            } else {
                imageMsg->encoding = imageFormat;
                imageMsg->step = static_cast<uint32_t >(capture.imageStep());
                imageMsg->data.reserve(static_cast<size_t >(capture.imageSize()));
            }

            return imageMsg;
        }

    }  // namespace ros_utils
}  // namespace lirs

int main(int argc, char **argv) {
    ros::init(argc, argv, "lirs_ros_video_streaming");

    ros::NodeHandle nodeHandle;
    ros::NodeHandle nodeHandle_{"~"};

    auto imageTransport = image_transport::ImageTransport{nodeHandle};
    auto publisher = imageTransport.advertiseCamera("image", 10);

    // get and validate capture parameters

    std::string deviceName;
    std::string cameraName;
    std::string frameId;
    std::string cameraInfoUrl;

    int width;
    int height;
    int frameRate;
    std::string imageFormat;

    nodeHandle_.param("device_name", deviceName, std::string{lirs::ros_utils::DEFAULT_DEVICE_NAME});
    nodeHandle_.param("camera_name", cameraName, std::string{lirs::ros_utils::DEFAULT_CAMERA_NAME});
    nodeHandle_.param("frame_id", frameId, std::string{lirs::ros_utils::DEFAULT_FRAME_ID});
    nodeHandle_.param("camera_info_url", cameraInfoUrl, std::string{lirs::ros_utils::DEFAULT_CAMERA_INFO_URL});
    nodeHandle_.param("width", width, lirs::ros_utils::DEFAULT_FRAME_WIDTH);
    nodeHandle_.param("height", height, lirs::ros_utils::DEFAULT_FRAME_HEIGHT);
    nodeHandle_.param("fps", frameRate, lirs::ros_utils::DEFAULT_FRAME_RATE);
    nodeHandle_.param("image_format", imageFormat, std::string{lirs::ros_utils::DEFAULT_IMAGE_FORMAT});

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
        ROS_ERROR_STREAM("Couldn't start streaming on: " << deviceName << ". Check streaming parameters.");
        return -1;
    }

    auto cameraInfoManager = camera_info_manager::CameraInfoManager(nodeHandle, cameraName, cameraInfoUrl);
    auto cameraInfoMsg = cameraInfoManager.getCameraInfo();

    ros::Rate rate(capture.Get(lirs::CaptureParam::FRAME_RATE));

    // NOTE: Image message format may differ from the image format (see imageMessageFrom() method).
    auto imageMsg = lirs::ros_utils::imageMessageFrom(frameId, imageFormat, capture);

    while (nodeHandle.ok()) {
        if (publisher.getNumSubscribers() > 0) {
            // if no cameraInfoUrl is provided
            if (cameraInfoMsg.distortion_model.empty()) {
                cameraInfoMsg = lirs::ros_utils::defaultCameraInfoFrom(imageMsg);
                cameraInfoManager.setCameraInfo(cameraInfoMsg);
            }

            if (auto frame = capture.ReadFrame(); frame.has_value()) {
                if (imageFormat == sensor_msgs::image_encodings::YUV422) {

                    cv::Mat rawImage(capture.Get(lirs::CaptureParam::FRAME_HEIGHT),
                                     capture.Get(lirs::CaptureParam::FRAME_WIDTH), CV_8UC2);

                    rawImage.data = frame->buffer().data();  // no copy

                    cv::Mat grayscale;

                    cv::cvtColor(rawImage, grayscale, cv::COLOR_YUV2GRAY_YUYV, 1);  // copy

                    imageMsg->data.assign(grayscale.data, grayscale.data + grayscale.rows * grayscale.cols);  // copy

                } else {
                    imageMsg->data = std::move(frame->buffer());
                }

                // TODO (Ramil Safin): Use frame's native timestamp, i.e. v4l2 buffer's timestamp.
                publisher.publish(*imageMsg, cameraInfoMsg, ros::Time::now());
            }

            ros::spinOnce();
        }

        rate.sleep();
    }
}
