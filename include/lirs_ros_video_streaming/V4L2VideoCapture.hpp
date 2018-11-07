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

#include <sys/mman.h>

#include <optional>
#include <atomic>
#include <vector>
#include <string>
#include <map>

#include "VideoCapture.hpp"
#include "V4L2Utils.hpp"

namespace lirs {


    class V4L2Capture final : public VideoCapture {
    public:
        /**
         * @brief Constructs video capture to the specified device using given parameters.
         *
         * If parameters are not specified the default ones will be used.
         * After successful video resource acquisition it will be opened, i.e. IsOpen() = true.
         *
         * @param device video resource.
         * @param v4l2PixFmt image format.
         * @param width captured frames width.
         * @param height captured frames height.
         * @param frameRate frames per second.
         * @param bufferSize preferred number of v4l2 buffers.
         */
        explicit V4L2Capture(std::string device,
                             uint32_t v4l2PixFmt = V4L2Defaults::DEFAULT_V4L2_PIXEL_FORMAT,
                             uint32_t width = V4L2Defaults::DEFAULT_WIDTH,
                             uint32_t height = V4L2Defaults::DEFAULT_HEIGHT,
                             uint32_t frameRate = V4L2Defaults::DEFAULT_FRAME_RATE,
                             uint32_t bufferSize = V4L2Defaults::DEFAULT_V4L2_BUFFER_SIZE);

        /**
         * @brief Cleans up allocated resources and frees video resource acquisition.
         */
        ~V4L2Capture() override {
            if (IsOpened() && IsStreaming()) {
                disableSteaming();
                cleanupBuffers();
                if (V4L2Utils::close_handle(handle_)) {
                    handle_ = V4L2Constants::CLOSED_HANDLE;  // IsOpened() = false
                }
            }
        }

        bool IsOpened() const override;

        bool IsStreaming() const override;

        bool StartStreaming() override;

        bool StopStreaming() override;

        /**
         * @brief Sets capture parameters if streaming mode is not enabled.
         *
         * @param param capture parameter name.
         * @param value capture parameter value.
         * @return true - if parameter is changed, false - otherwise.
         */
        bool Set(CaptureParam param, uint32_t value) override;

        uint32_t Get(CaptureParam param) const override;

        /**
         * @brief Captures a frame from the device.
         *
         * @return empty - no frames are captured, captured frame - otherwise.
         */
        std::optional<Frame> ReadFrame() override;

        std::string const &device() const override {
            return device_;
        };

        uint32_t imageStep() const override {
            return imageStep_;
        }

        uint32_t imageSize() const override {
            return imageSize_;
        }

        // prohibit copying and moving

        V4L2Capture(const V4L2Capture &) = delete;

        V4L2Capture &operator=(V4L2Capture const &) = delete;

        V4L2Capture(V4L2Capture &&) = delete;

        V4L2Capture &operator=(V4L2Capture &&) = delete;

    private:
        /* v4l2 buffer */
        struct Buffer final {
            void *data;
            size_t length;  // size in bytes

            /**
             * Cleans up mapped buffer.
             */
            ~Buffer() {
                if (munmap(data, length) == -1) std::cerr << "WARNING: Unable to unmap buffers\n";
            }

            Buffer(void *bufData, size_t bufLen) : data(bufData), length(bufLen) {}
        };

        /**
         * @brief Allocates v4l2 buffers by mapping them into application's memory.
         *
         * @return true - success, false - otherwise.
         */
        bool allocateBuffers();

        /**
         * @brief Cleans up allocated buffers by deleting Buffers (unmap) and requesting zero v4l2 buffers.
         */
        void cleanupBuffers();

        /**
         * @brief Getting into streaming mode (@see IsStreaming()).
         *
         * @return true - success, false - otherwise.
         */
        bool enableStreaming();

        /**
         * @brief Stops streaming mode (@see IsStreaming()).
         *
         * @return true - success, false - otherwise.
         */
        bool disableSteaming();

        /**
         * @brief Tries to set the specified by the capture parameters format.
         *
         * If given format does not supported by the device format negotiation stops.
         *
         * @return true - format is set, false - otherwise.
         */
        bool negotiateFormat();

        /**
         * @brief Tries to set the frame rate to the specified by capture parameters value.
         *
         * If given frame rate does not supported v4l2 driver handles it automatically and sets it to the supported one.
         *
         * @return true - frame rate is set, false - otherwise.
         */
        bool negotiateFrameRate();

        /**
         * @brief Checks v4l2 device capabilities.
         *
         * Capabilities include support for video streaming (memory mapping I/O) and capturing abilities.
         *
         * @return true - v4l2 device supports for required capabilities, false - otherwise.
         */
        bool checkCapabilities();

        /**
         * @brief Retrieves frame from the v4l2 driver's outgoing queue.
         *
         * @return captured frame - in case of success, empty - error occurred .
         */
        std::optional<Frame> internalReadFrame();

    private:
        /* Video device file descriptor */
        int handle_;

        /* Image step (bytes in one image line) */
        uint32_t imageStep_;

        /* Size of the images in bytes */
        uint32_t imageSize_;

        /* Video device url, e.g. '/dev/video0' */
        std::string const device_;

        /* Flag indicating if streaming process in on */
        std::atomic_bool isStreaming_;

        /* Frames buffer mapped to the video driver's internal buffer (Memory Mapping) */
        std::vector<Buffer> internalBuffer_;

        /* Video capturing parameters */
        std::map<CaptureParam, uint32_t> params_;
    };

}  // namespace lirs