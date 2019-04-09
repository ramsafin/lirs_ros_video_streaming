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

#include "VideoCapture.hpp"

#include <sys/mman.h>
#include <optional>
#include <atomic>
#include <vector>
#include <string>
#include <map>

#include "V4L2Utils.hpp"

namespace lirs {

    class V4L2Capture final : public VideoCapture {
    public:
        /**
         * @brief Constructs video capture to the specified device using given parameters.
         *
         * If parameters are not specified the default ones will be used.
         * After successful video resource acquisition it will be opened, i.e. IsOpen() = true.
         */
        explicit V4L2Capture(std::string device,
                             uint32_t v4l2PixFmt = v4l2_defaults::DEFAULT_V4L2_PIXEL_FORMAT,
                             int width = v4l2_defaults::DEFAULT_FRAME_WIDTH,
                             int height = v4l2_defaults::DEFAULT_FRAME_HEIGHT,
                             int frameRate = v4l2_defaults::DEFAULT_FRAME_RATE,
                             int bufferSize = v4l2_defaults::DEFAULT_V4L2_BUFFERS_NUM);

        ~V4L2Capture() override {
            if (IsOpened()) {
                if (IsStreaming()) {
                    disableSteaming();
                    cleanupInternalBuffers();
                }

                if (V4L2Utils::close_device(handle_)) {
                    handle_ = v4l2_constants::CLOSED_HANDLE;
                }
            }
        }

        bool IsOpened() const override;

        bool IsStreaming() const override;

        bool StartStreaming() override;

        bool StopStreaming() override;

        /**
         * @brief Sets capture parameters if streaming mode is not enabled.
         */
        bool Set(CaptureParam param, int value) override;

        int Get(CaptureParam param) const override;

        std::optional<Frame> ReadFrame() override;

        std::string const &device() const override {
            return device_;
        };

        int imageStep() const override {
            return imageStep_;
        }

        int imageSize() const override {
            return imageSize_;
        }

        // prohibit copying and moving

        V4L2Capture(const V4L2Capture &) = delete;

        V4L2Capture &operator=(V4L2Capture const &) = delete;

        V4L2Capture(V4L2Capture &&) = delete;

        V4L2Capture &operator=(V4L2Capture &&) = delete;

    private:
        struct MappedBuffer final {
            void *rawDataPtr = nullptr;
            int lengthBytes = -1;

            ~MappedBuffer() {
                if (munmap(rawDataPtr, static_cast<size_t>(lengthBytes)) == V4L2Utils::ERROR_CODE) {
                    std::cerr << "WARNING: Unable to unmap buffers\n";
                }
            }

            MappedBuffer(void *bufData, int bufLen) : rawDataPtr(bufData), lengthBytes(bufLen) {}
        };

        bool allocateInternalBuffers();

        void cleanupInternalBuffers();

        bool enableStreaming();

        bool disableSteaming();

        bool negotiateFormat();

        bool negotiateFrameRate();

        bool checkSupportedCapabilities();

        std::optional<Frame> internalReadFrame();

    private:
        int handle_;

        int imageStep_;

        int imageSize_;

        std::string const device_;

        /* Flag indicating if streaming process is on */
        std::atomic_bool isStreaming_;

        std::vector<MappedBuffer> internalBuffers_;

        std::map<CaptureParam, int> params_;
    };

}  // namespace lirs
