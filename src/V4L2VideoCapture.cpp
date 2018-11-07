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
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdint>
#include <cstring>
#include <cerrno>

#include <iostream>

#include "lirs_ros_video_streaming/V4L2VideoCapture.hpp"

namespace lirs {
    V4L2Capture::V4L2Capture(std::string device, uint32_t v4l2PixFmt, uint32_t width, uint32_t height,
                             uint32_t frameRate, uint32_t bufferSize)
            : handle_(V4L2Constants::CLOSED_HANDLE),
              imageStep_(0), imageSize_(0),
              device_(std::move(device)),
              isStreaming_(false) {
        params_ = {
                {CaptureParam::WIDTH,       width},
                {CaptureParam::HEIGHT,      height},
                {CaptureParam::FPS,         frameRate},
                {CaptureParam::PIX_FMT,     v4l2PixFmt},
                {CaptureParam::BUFFER_SIZE, bufferSize}
        };

        handle_ = V4L2Utils::open_handle(device_);  // acquire resource
    }

    bool V4L2Capture::IsOpened() const {
        return handle_ != V4L2Constants::CLOSED_HANDLE;
    }

    bool V4L2Capture::IsStreaming() const {
        return isStreaming_;
    }

    bool V4L2Capture::StartStreaming() {
        if (!IsOpened()) return false; // CLOSED HANDLE ERROR

        if (IsStreaming()) return true; // ALREADY STREAMING

        if (!checkCapabilities()) return false; // UNSUPPORTED CAPABILITIES ERROR

        if (!negotiateFormat()) return false;  // FORMAT NEGOTIATION ERROR

        if (!negotiateFrameRate()) return false; // UNSUPPORTED FORMAT ERROR

        if (!allocateBuffers()) {
            cleanupBuffers();
            return false; // BUFFERS ALLOCATION ERROR
        }

        return enableStreaming();
    }

    bool V4L2Capture::StopStreaming() {
        if (!IsOpened()) {
            return false; // CLOSED HANDLE ERROR
        }

        if (!IsStreaming()) {
            return true; // NOT STREAMING
        }

        if (!disableSteaming()) {
            return false;  // CANNOT STOP STREAMING
        };

        cleanupBuffers();

        return true;
    }

    bool V4L2Capture::Set(CaptureParam param, uint32_t value) {
        if (IsStreaming()) return false;  // no change of params while streaming

        // TODO (Ramil Safin): Add parameters validation.
        switch (param) {
            case CaptureParam::BUFFER_SIZE:
                if (!V4L2Utils::is_in_range_inclusive(1u, V4L2Constants::V4L2_MAX_BUFFER_SIZE, value)) {
                    return false;
                }
                break;
            default:
                break;
        }
        params_[param] = value;

        return true;
    }

    uint32_t V4L2Capture::Get(CaptureParam param) const {
        return params_.at(param);
    }

    std::optional<Frame> V4L2Capture::ReadFrame() {
        return IsStreaming() && V4L2Utils::v4l2_is_readable(handle_) ? internalReadFrame() : std::nullopt;
    }

    bool V4L2Capture::allocateBuffers() {
        v4l2_requestbuffers requestBuffers{};
        requestBuffers.count = Get(CaptureParam::BUFFER_SIZE);
        requestBuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        requestBuffers.memory = V4L2_MEMORY_MMAP;

        // enqueue requested buffers to the driver's queue

        if (V4L2Utils::xioctl(handle_, VIDIOC_REQBUFS, &requestBuffers) == -1) {
            if (errno == EINVAL) {
                std::cerr << "ERROR: Device does not support memory mapping - " << strerror(errno) << '\n';
            } else {
                std::cerr << "ERROR: VIDIOC_REQBUFS - " << strerror(errno) << '\n';
            }
            return false;
        }

        if (requestBuffers.count != Get(CaptureParam::BUFFER_SIZE)) {
            params_[CaptureParam::BUFFER_SIZE] = requestBuffers.count;

            std::cerr << "WARNING: Buffer size on " << device_
                      << " has changed to " << requestBuffers.count << '\n';
        }

        internalBuffer_.reserve(requestBuffers.count);

        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;

        for (buffer.index = 0; buffer.index < requestBuffers.count; ++buffer.index) {
            if (V4L2Utils::xioctl(handle_, VIDIOC_QUERYBUF, &buffer) == -1) {
                std::cerr << "ERROR: VIDIOC_QUERYBUF - " << strerror(errno) << '\n';
                return false;
            }

            auto bufferLength = buffer.length;

            auto bufferData = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                   handle_, buffer.m.offset);

            if (bufferData == MAP_FAILED) {
                std::cerr << "ERROR: Memory Mapping has failed - " << strerror(errno) << '\n';
                return false;
            }

            internalBuffer_.emplace_back(bufferData, bufferLength);
        }

        return true;
    }

    void V4L2Capture::cleanupBuffers() {
        if (!internalBuffer_.empty()) {
            internalBuffer_.clear();
            internalBuffer_.shrink_to_fit();

            v4l2_requestbuffers requestBuffers{};
            requestBuffers.count = 0;
            requestBuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            requestBuffers.memory = V4L2_MEMORY_MMAP;

            if (V4L2Utils::xioctl(handle_, VIDIOC_REQBUFS, &requestBuffers) == -1) {
                std::cerr << "ERROR: Cannot cleanup allocated buffers - " << strerror(errno) << '\n';
                return;
            }
        }
    }

    bool V4L2Capture::enableStreaming() {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;

        for (buffer.index = 0; buffer.index < Get(CaptureParam::BUFFER_SIZE); ++buffer.index) {
            if (V4L2Utils::xioctl(handle_, VIDIOC_QBUF, &buffer) == -1) {
                std::cerr << "ERROR: VIDIOC_QBUF  - " << strerror(errno) << '\n';
                return false;
            }
        }

        if (V4L2Utils::xioctl(handle_, VIDIOC_STREAMON, &buffer.type) == -1) {
            std::cerr << "ERROR: Cannot enable streaming mode - " << strerror(errno) << '\n';
            return false;
        }

        isStreaming_ = true;

        return true;
    }

    bool V4L2Capture::disableSteaming() {
        if (auto bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                V4L2Utils::xioctl(handle_, VIDIOC_STREAMOFF, &bufType) == -1) {
            std::cerr << "ERROR: Unable to stop streaming - " << strerror(errno) << '\n';
            return false;
        }

        isStreaming_ = false;

        return true;
    }

    bool V4L2Capture::negotiateFormat() {
        if (V4L2Utils::v4l2_try_format(handle_, Get(CaptureParam::PIX_FMT), Get(CaptureParam::WIDTH),
                                       Get(CaptureParam::HEIGHT))) {

            if (auto format = V4L2Utils::v4l2_set_format(handle_,
                                                         Get(CaptureParam::PIX_FMT),
                                                         Get(CaptureParam::WIDTH),
                                                         Get(CaptureParam::HEIGHT)); format.has_value()) {
                imageStep_ = format->fmt.pix.bytesperline;
                imageSize_ = format->fmt.pix.sizeimage;
                return true;
            }

        }

        return false;
    }

    bool V4L2Capture::negotiateFrameRate() {
        if (auto frameRate = V4L2Utils::v4l2_set_frame_rate(handle_, 1, Get(CaptureParam::FPS)); frameRate) {
            params_[CaptureParam::FPS] = frameRate->parm.capture.timeperframe.denominator;
            return true;
        }

        return false;
    }

    bool V4L2Capture::checkCapabilities() {
        // TODO (Ramil Safin): Cache queried capabilities.
        auto requiredCapabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

        if (auto caps = V4L2Utils::v4l2_query_capabilities(handle_); caps.has_value()) {
            return V4L2Utils::v4l2_check_input_capabilities(handle_)
                   && V4L2Utils::v4l2_check_capabilities(caps.value(), requiredCapabilities);
        }

        return true;
    }

    std::optional<Frame> V4L2Capture::internalReadFrame() {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;

        int res = 0;  // dequerying status

        // dequery v4l2 buffers from the driver's outgoing queue

        while ((res = V4L2Utils::xioctl(handle_, VIDIOC_DQBUF, &buffer)) < 0 && (errno == EINTR));

        if (res < 0) {
            switch (errno) {
                case EAGAIN:
                    std::cerr << "WARNING: Device is not ready for reading" << strerror(errno) << '\n';
                    return std::nullopt;
                case EIO:
                    std::cerr << "ERROR: I/O error while reading - " << strerror(errno) << '\n';
                    return std::nullopt;
                default:
                    std::cerr << "ERROR: VIDIOC_DQBUF - " << strerror(errno) << '\n';
                    return std::nullopt;
            }
        }

        // skip corrupted v4l2 buffers

        if (buffer.flags & V4L2_BUF_FLAG_ERROR) {
            std::cerr << "WARNING: Dequeued v4l2 buffer with size " << buffer.bytesused << " (bytes) is corrupted\n";

            buffer.bytesused = 0;

            if (V4L2Utils::xioctl(handle_, VIDIOC_QBUF, &buffer) == -1) {
                std::cerr << "ERROR: VIDIOC_QBUF - " << strerror(errno) << '\n';
            }

            return std::nullopt;
        }

        // TODO (Ramil Safin): Convert frame timestamp into absolute time.

        // copy buffer before querying
        Frame frame{static_cast<uint8_t *>(internalBuffer_[buffer.index].data), buffer.bytesused};

        if (V4L2Utils::xioctl(handle_, VIDIOC_QBUF, &buffer) == -1) {
            std::cerr << "ERROR: VIDIOC_QBUF - " << strerror(errno) << '\n';
        }

        return std::optional{frame};
    }

}  // namespace lirs