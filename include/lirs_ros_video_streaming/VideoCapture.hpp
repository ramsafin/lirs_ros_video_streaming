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

#pragma once

#include <linux/videodev2.h>
#include <vector>
#include <chrono>
#include <optional>

namespace lirs {

    namespace v4l2_defaults {
        constexpr auto DEFAULT_FRAME_RATE = 30;
        constexpr auto DEFAULT_FRAME_WIDTH = 640;
        constexpr auto DEFAULT_FRAME_HEIGHT = 480;
        constexpr auto DEFAULT_V4L2_BUFFERS_NUM = 4;
        constexpr auto DEFAULT_V4L2_PIXEL_FORMAT = uint32_t{V4L2_PIX_FMT_YUYV};
    }

    /**
     * @brief Captured video data, i.e. images.
     *
     * @todo add frame's captured timestamp and sequence number.
     */
    class Frame final {
    public:
        Frame(uint8_t *data, size_t size)
                : buffer_{std::vector<uint8_t>(data, data + size)},
                  captured_(std::chrono::system_clock::now().time_since_epoch()) {}

        std::vector<uint8_t> &buffer() {
            return buffer_;
        }

        std::vector<uint8_t> const &buffer() const {
            return buffer_;
        }

        std::chrono::nanoseconds timestamp() const {
            return captured_;
        }

    private:
        std::vector<uint8_t> buffer_;
        std::chrono::nanoseconds captured_;
    };

    /**
     * @brief Capture parameters enum.
     *
     * Capture parameters used in order to get and/or modify parameters in VideoCapture.
     */
    enum class CaptureParam : uint8_t {
        FRAME_RATE,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        V4L2_PIX_FMT,
        V4L2_BUFFERS_NUM
    };

    /**
     * @brief Video capturing device representation.
     *
     * Video capture acquires the resource (e.g. /dev/video) and controls its lifecycle.
     * Capture is not acquired only if IsOpened() returns false. Resource is free after
     * destruction of the video capture instance.
     *
     * Non-copyable and non-movable.
     */
    class VideoCapture {
    public:
        VideoCapture() = default;

        virtual ~VideoCapture() = default;

        /**
         * Check whether capture is acquired or not,
         * i.e. whether the resource is controlled or not.
         *
         * @return true - if resource is open, false - otherwise.
         */
        virtual bool IsOpened() const = 0;

        /**
         * @brief Starts video streaming process.
         *
         * After successful result frames from the resource can be retrieved by ReadFrame().
         * Streaming can be stopped using StopStreaming().
         *
         * Capture parameters should not be changed during the streaming process.
         *
         * @return true - in case if streaming is started, false - otherwise.
         */
        virtual bool StartStreaming() = 0;

        /**
         * @brief Stops video streaming process.
         *
         * Capture parameters could be changed while streaming is stopped.
         * ReadFrame() could retrieve left frames from the buffer (e.g. queue), but not directly from the resource.
         *
         * @return true - streaming is stopped, false - otherwise.
         */
        virtual bool StopStreaming() = 0;

        /**
         * @brief Returns the streaming status.
         *
         * @return true - streaming is enabled, false - otherwise.
         */
        virtual bool IsStreaming() const = 0;

        /**
         * @brief Captures frame from the resource or retrieves one from the buffers.
         *
         * @return empty - in case if there is no frames to capture/retrieve, otherwise - frame with data.
         */
        virtual std::optional<Frame> ReadFrame() = 0;

        /**
         * @brief Sets capture parameter.
         *
         * In case if streaming is enabled (IsStreaming() = true)
         * it is advised not to return false (not to change params).
         *
         * @param param capture parameter to change.
         * @param value capture parameter's new value.
         * @return true - if parameter is changed, false - otherwise.
         */
        virtual bool Set(CaptureParam param, int value) = 0;

        /**
         * @brief Gets capture parameter's value.
         *
         * @param param capture parameter name.
         * @return capture parameter's value.
         */
        virtual int Get(CaptureParam param) const = 0;

        /**
         * @return capture resource, or device.
         */
        virtual std::string const &device() const = 0;

        /**
         * @return captured image line size in bytes.
         */
        virtual int imageStep() const = 0;

        /**
         * @return captured image size in bytes.
         */
        virtual int imageSize() const = 0;

        /* No copy */
        VideoCapture(VideoCapture const &) = delete;

        VideoCapture &operator=(VideoCapture const &) = delete;

    };
}  // namespace lirs
