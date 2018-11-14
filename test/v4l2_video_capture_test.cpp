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

#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <linux/videodev2.h>

#include "lirs_ros_video_streaming/V4L2VideoCapture.hpp"

using std::string_literals::operator ""s;

const auto TESTED_DEVICE = "/dev/video0"s;

TEST(VideoCaptureUtilsTestCase, OpenCloseHandleShouldPass) {
    auto handle = lirs::V4L2Utils::open_device(TESTED_DEVICE);

    EXPECT_TRUE(handle > 0);
    EXPECT_TRUE(lirs::V4L2Utils::close_device(handle));
}

TEST(VideoCaptureUtilsTestCase, CloseClosedHandleShouldPass) {
    EXPECT_FALSE(lirs::V4L2Utils::close_device(lirs::V4L2Constants::CLOSED_HANDLE));
}

TEST(VideoCaptureUtilsTestCase, GetFrameRateShouldPass) {
    auto handle = lirs::V4L2Utils::open_device(TESTED_DEVICE);

    ASSERT_TRUE(handle != lirs::V4L2Constants::CLOSED_HANDLE);

    auto frameRate = lirs::V4L2Utils::v4l2_get_current_frame_rate(handle);

    EXPECT_TRUE(frameRate.has_value());
    ASSERT_TRUE(lirs::V4L2Utils::close_device(handle));
}

TEST(VideoCaptureUtilsTestCase, SetFrameRateShouldPass) {
    auto handle = lirs::V4L2Utils::open_device(TESTED_DEVICE);

    ASSERT_TRUE(handle > 0);

    auto den = 30u;
    auto num = 1u;

    auto frameRate = lirs::V4L2Utils::v4l2_set_frame_rate(handle, num, den);

    ASSERT_TRUE(frameRate.has_value());
    EXPECT_EQ(frameRate->parm.capture.timeperframe.numerator, num);
    EXPECT_EQ(frameRate->parm.capture.timeperframe.denominator, den);

    ASSERT_TRUE(lirs::V4L2Utils::close_device(handle));
}

TEST(VideoCaptureUtilsTestCase, CaptureGetFormatShouldPass) {
    auto handle = lirs::V4L2Utils::open_device(TESTED_DEVICE);

    ASSERT_TRUE(handle > 0);

    auto format = lirs::V4L2Utils::v4l2_get_current_format(handle);

    EXPECT_TRUE(format);
    ASSERT_TRUE(lirs::V4L2Utils::close_device(handle));
}

TEST(VideoCaptureUtilsTestCase, SetAndTryFormatShouldPass) {
    auto handle = lirs::V4L2Utils::open_device(TESTED_DEVICE);

    ASSERT_TRUE(handle > 0);

    EXPECT_FALSE(lirs::V4L2Utils::v4l2_try_format(handle, V4L2_PIX_FMT_H264, 244, 333));

    auto width = 640u;
    auto height = 480u;
    auto tryFormat = lirs::V4L2Utils::v4l2_try_format(handle, V4L2_PIX_FMT_YUYV, width, height);

    ASSERT_TRUE(tryFormat.has_value());

    EXPECT_EQ(tryFormat->fmt.pix.pixelformat, V4L2_PIX_FMT_YUYV);
    EXPECT_EQ(tryFormat->fmt.pix.width, width);
    EXPECT_EQ(tryFormat->fmt.pix.height, height);

    EXPECT_TRUE(lirs::V4L2Utils::v4l2_set_format(handle, tryFormat->fmt.pix.pixelformat,
                                                 tryFormat->fmt.pix.width, tryFormat->fmt.pix.height));

    ASSERT_TRUE(lirs::V4L2Utils::close_device(handle));
}

TEST(VideoCaptureTestCase, GetPixelFormatsShouldNotReturnEmptySet) {
    auto handle = lirs::V4L2Utils::open_device(TESTED_DEVICE);

    ASSERT_TRUE(handle != lirs::V4L2Constants::CLOSED_HANDLE);

    auto pixelFormats = lirs::V4L2Utils::v4l2_query_pixel_formats(handle);

    EXPECT_FALSE(pixelFormats.empty());
    ASSERT_TRUE(lirs::V4L2Utils::close_device(handle));
}

TEST(VideoCaptureTestCase, V4L2CaptureConstructionShouldPass) {
    lirs::V4L2Capture capture(TESTED_DEVICE);

    EXPECT_TRUE(capture.IsOpened());
    EXPECT_FALSE(capture.IsStreaming());
    EXPECT_FALSE(capture.ReadFrame().has_value());
    EXPECT_TRUE(capture.StopStreaming());

    EXPECT_STREQ(TESTED_DEVICE.data(), capture.device().data());
}

TEST(VideoCaptureTestCase, NonStreamingCaptureShouldPass) {
    lirs::V4L2Capture capture(TESTED_DEVICE);

    EXPECT_TRUE(capture.IsOpened());
    EXPECT_FALSE(capture.IsStreaming());
    EXPECT_FALSE(capture.ReadFrame().has_value());
    EXPECT_TRUE(capture.StopStreaming());
}

TEST(VideoCaptureTestCase, DefaultParamsShouldBeSetCorrectly) {
    lirs::V4L2Capture capture(TESTED_DEVICE);

    EXPECT_EQ(capture.Get(lirs::CaptureParam::V4L2_BUFFERS_NUM), lirs::V4L2Defaults::DEFAULT_V4L2_BUFFERS_NUM);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::V4L2_PIX_FMT), lirs::V4L2Defaults::DEFAULT_V4L2_PIXEL_FORMAT);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::FRAME_RATE), lirs::V4L2Defaults::DEFAULT_FRAME_RATE);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::FRAME_HEIGHT), lirs::V4L2Defaults::DEFAULT_FRAME_HEIGHT);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::FRAME_WIDTH), lirs::V4L2Defaults::DEFAULT_FRAME_WIDTH);
}

TEST(VideoCaptureTestCase, CaptureConstructorParamsShouldBeSetCorrectly) {
    auto bufferSize = 8u;
    auto width = 320u;
    auto height = 240u;
    auto pixFmt = V4L2_PIX_FMT_YUYV;
    auto frameRate = 24u;

    lirs::V4L2Capture capture(TESTED_DEVICE, pixFmt, width, height, frameRate, bufferSize);

    EXPECT_EQ(capture.Get(lirs::CaptureParam::V4L2_BUFFERS_NUM), bufferSize);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::V4L2_PIX_FMT), pixFmt);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::FRAME_RATE), frameRate);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::FRAME_HEIGHT), height);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::FRAME_WIDTH), width);
}

TEST(VideoCaptureTestCase, SetParamsShouldChangeParameterValues) {
    lirs::V4L2Capture capture(TESTED_DEVICE);

    auto bufferSize = 16u;
    auto frameRate = 60u;

    EXPECT_TRUE(capture.Set(lirs::CaptureParam::V4L2_BUFFERS_NUM, bufferSize));
    EXPECT_TRUE(capture.Set(lirs::CaptureParam::FRAME_RATE, frameRate));

    EXPECT_EQ(capture.Get(lirs::CaptureParam::V4L2_BUFFERS_NUM), bufferSize);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::FRAME_RATE), frameRate);
}

TEST(VideoCaptureTestCase, NonExistingDeviceCaptureShouldPass) {
    auto deviceName = "/dev/videoXXX"s;
    lirs::V4L2Capture capture(deviceName);

    EXPECT_FALSE(capture.IsOpened());
    EXPECT_STREQ(capture.device().data(), deviceName.data());

    EXPECT_FALSE(capture.IsStreaming());
    EXPECT_FALSE(capture.StopStreaming());
    EXPECT_FALSE(capture.StartStreaming());
    EXPECT_FALSE(capture.ReadFrame().has_value());
}

TEST(VideoCaptureTestCase, StartStreamingOnOpenedCaptureShouldPass) {
    lirs::V4L2Capture capture(TESTED_DEVICE);

    ASSERT_TRUE(capture.IsOpened());
    EXPECT_FALSE(capture.IsStreaming());

    ASSERT_TRUE(capture.StartStreaming());
    EXPECT_TRUE(capture.IsStreaming());

    EXPECT_TRUE(capture.StartStreaming());
    EXPECT_TRUE(capture.IsStreaming());

    EXPECT_TRUE(capture.ReadFrame().has_value());
}

TEST(VideoCaptureTestCase, StopStreamingOnOpenedCaptureShouldPass) {
    lirs::V4L2Capture capture(TESTED_DEVICE);

    ASSERT_TRUE(capture.IsOpened());
    ASSERT_TRUE(capture.StartStreaming());
    EXPECT_TRUE(capture.IsStreaming());
    EXPECT_TRUE(capture.ReadFrame().has_value());

    EXPECT_TRUE(capture.StopStreaming());
    EXPECT_FALSE(capture.IsStreaming());

    EXPECT_TRUE(capture.StopStreaming());
    EXPECT_FALSE(capture.IsStreaming());

    EXPECT_NE(capture.Get(lirs::CaptureParam::V4L2_BUFFERS_NUM), 0);
}

TEST(VideoCaptureTestCase, AnotherCaptureOnOpenedDeviceShouldNotPass) {
    lirs::V4L2Capture capture(TESTED_DEVICE);

    ASSERT_TRUE(capture.IsOpened());

    lirs::V4L2Capture otherCapture(TESTED_DEVICE);

    EXPECT_TRUE(otherCapture.IsOpened());
    EXPECT_TRUE(capture.StartStreaming());
    EXPECT_FALSE(otherCapture.StartStreaming());
    ASSERT_TRUE(capture.StopStreaming());
    EXPECT_TRUE(otherCapture.StartStreaming());
}

TEST(VideoCaptureTestCase, ReadFrameOnOpenedCaptureShouldPass) {
    using std::chrono_literals::operator ""s;

    lirs::V4L2Capture capture(TESTED_DEVICE);

    ASSERT_TRUE(capture.IsOpened());
    ASSERT_TRUE(capture.StartStreaming());
    EXPECT_TRUE(capture.IsStreaming());

    auto start_time = std::chrono::system_clock::now();

    // for 5 seconds
    for (auto i = 0u; i < capture.Get(lirs::CaptureParam::FRAME_RATE) * 5; i++) {
        auto frame = capture.ReadFrame();

        EXPECT_TRUE(frame.has_value());
        EXPECT_FALSE(frame->buffer().empty());
    }

    auto stop_time = std::chrono::system_clock::now();
    auto delta = stop_time - start_time;

    EXPECT_TRUE(delta >= 5s && delta < 5.5s);
}

TEST(VideoCaptureTestCase, DeviceDestrcutroShouldCleansUpResources) {
    {
        lirs::V4L2Capture capture(TESTED_DEVICE);
        ASSERT_TRUE(capture.IsOpened());
        EXPECT_TRUE(capture.StartStreaming());
    }
    {
        lirs::V4L2Capture capture(TESTED_DEVICE);
        ASSERT_TRUE(capture.IsOpened());
        EXPECT_TRUE(capture.StartStreaming());
    }
}

TEST(VideoCaptureTestCase, ExceededCaptureBufferSizeShouldBeValidated) {
    lirs::V4L2Capture capture(TESTED_DEVICE);

    EXPECT_TRUE(capture.IsOpened());
    EXPECT_FALSE(capture.Set(lirs::CaptureParam::V4L2_BUFFERS_NUM, 50));  // incorrect buffer size
    EXPECT_EQ(capture.Get(lirs::CaptureParam::V4L2_BUFFERS_NUM), lirs::V4L2Defaults::DEFAULT_V4L2_BUFFERS_NUM);
}

TEST(VideoCaptureTestCase, FrameRateShouldBeChanged) {
    lirs::V4L2Capture capture(TESTED_DEVICE);

    ASSERT_TRUE(capture.IsOpened());

    capture.Set(lirs::CaptureParam::FRAME_RATE, 34);

    EXPECT_TRUE(capture.StartStreaming());
    EXPECT_EQ(capture.Get(lirs::CaptureParam::FRAME_RATE), 30);
}

TEST(VideoCaptureTestCase, CaptureFormatNegotiationShouldPass) {
    lirs::V4L2Capture capture(TESTED_DEVICE, V4L2_PIX_FMT_H264, 777, 666);

    ASSERT_TRUE(capture.IsOpened());
    EXPECT_FALSE(capture.StartStreaming());

    EXPECT_EQ(capture.Get(lirs::CaptureParam::V4L2_PIX_FMT), V4L2_PIX_FMT_H264);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::FRAME_WIDTH), 777);
    EXPECT_EQ(capture.Get(lirs::CaptureParam::FRAME_HEIGHT), 666);
}

TEST(VideoCaptureTestCase, CaptureImageStepAndSizeAreSetCorrectly) {
    lirs::V4L2Capture capture(TESTED_DEVICE);

    ASSERT_TRUE(capture.IsOpened());
    ASSERT_TRUE(capture.StartStreaming());

    EXPECT_EQ(capture.imageStep(), capture.Get(lirs::CaptureParam::FRAME_WIDTH) * 2);
    EXPECT_EQ(capture.imageSize(),
              capture.Get(lirs::CaptureParam::FRAME_WIDTH) * capture.Get(lirs::CaptureParam::FRAME_HEIGHT) * 2);
}

TEST(DISABLED_VideoCaptureTestCase, BuggyCameraShouldPass) {
    lirs::V4L2Capture capture("/dev/v4l/by-id/usb-Twiga_TWIGACam-video-index0");

    ASSERT_TRUE(capture.IsOpened());

    capture.Set(lirs::CaptureParam::FRAME_RATE, 50);
    capture.Set(lirs::CaptureParam::FRAME_WIDTH, 1280);
    capture.Set(lirs::CaptureParam::FRAME_HEIGHT, 720);
    capture.Set(lirs::CaptureParam::V4L2_PIX_FMT, V4L2_PIX_FMT_YUYV);
    capture.Set(lirs::CaptureParam::V4L2_BUFFERS_NUM, 4);

    ASSERT_TRUE(capture.StartStreaming());

    // corrupted v4l2 buffers
    EXPECT_FALSE(capture.ReadFrame());
    EXPECT_FALSE(capture.ReadFrame());
    EXPECT_FALSE(capture.ReadFrame());
    EXPECT_FALSE(capture.ReadFrame());

    auto buffer = capture.ReadFrame();
    ASSERT_TRUE(buffer);
}

TEST(FrameTestCase, EmptyFrameShouldPass) {
    lirs::Frame frame{nullptr, 0};

    EXPECT_TRUE(frame.buffer().empty());
}

TEST(FrameTestCase, FrameShouldBeCopied) {
    size_t bufferSize = 32;
    uint8_t buffer[bufferSize] = {0};

    lirs::Frame frame{buffer, bufferSize};

    EXPECT_EQ(frame.buffer().size(), bufferSize);

    auto copyFrame = frame;  // copy constructor

    EXPECT_EQ(copyFrame.buffer().size(), frame.buffer().size());

    lirs::Frame otherFrame {std::move(copyFrame)};  // move constructor

    EXPECT_EQ(otherFrame.buffer().size(), bufferSize);
    EXPECT_TRUE(copyFrame.buffer().empty());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
