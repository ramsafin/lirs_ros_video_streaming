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
 *  Created by Ramil Safin <safin.ramil@it.kfu.ru> on 07.11.2018.
 */

#pragma once

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>

#include <optional>
#include <iostream>
#include <string>
#include <set>

namespace lirs {

    namespace v4l2_constants {
        constexpr auto CLOSED_HANDLE = -1;
        constexpr auto V4L2_MAX_BUFFER_SIZE = 32;
    }

    struct V4L2Utils {
        static constexpr auto ERROR_CODE = -1;

        static constexpr auto SELECT_TIMEOUT_SECONDS = 1;
        static constexpr auto SELECT_TIMEOUT_MICROSECONDS = 0;
        static constexpr auto SELECT_NUM_OF_READY_DEVICES = 1;

        static inline bool v4l2_is_readable(int handle,
                                            timeval timeout = {SELECT_TIMEOUT_SECONDS, SELECT_TIMEOUT_MICROSECONDS}) {
            fd_set fds{};
            FD_ZERO(&fds);
            FD_SET(handle, &fds);

            return select(handle + 1, &fds, nullptr, nullptr, &timeout) == SELECT_NUM_OF_READY_DEVICES;
        }

        static inline int xioctl(int handle, size_t request, void *arg) {
            int status{0};
            do {
                status = ioctl(handle, request, arg);
            } while (status == ERROR_CODE && errno == EINTR);

            return status;
        }

        static bool v4l2_is_character(std::string const &device) {
            if (struct stat status{}; stat(device.c_str(), &status) != ERROR_CODE) {
                if (!S_ISCHR(status.st_mode)) {
                    std::cerr << "ERROR: " << device << " is not a character v4l2 device - "
                              << strerror(errno) << '\n';
                    return false;
                }
                return true;
            }

            std::cerr << "ERROR: Cannot identify device - " << device << " - " << strerror(errno) << '\n';
            return false;
        }

        static inline int open_device(std::string const &device) {
            if (!v4l2_is_character(device)) return v4l2_constants::CLOSED_HANDLE;

            if (auto handle = open(device.c_str(), O_RDWR | O_NONBLOCK); handle != v4l2_constants::CLOSED_HANDLE) {
                return handle;
            }

            std::cerr << "ERROR: Cannot open v4l2 device " << device << " - " << strerror(errno) << '\n';
            return v4l2_constants::CLOSED_HANDLE;
        }

        static bool close_device(int handle) {
            if (handle == lirs::v4l2_constants::CLOSED_HANDLE) return false;

            if (close(handle) == ERROR_CODE) {
                std::cerr << "ERROR: Cannot close v4l2 device - " << strerror(errno) << '\n';
                return false;
            }
            return true;
        }

        static std::set<uint32_t> v4l2_query_pixel_formats(int fd) {
            std::set<uint32_t> pixelFormats;

            v4l2_fmtdesc desc{};
            desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            while (V4L2Utils::xioctl(fd, VIDIOC_ENUM_FMT, &desc) != ERROR_CODE) {
                ++desc.index;
                pixelFormats.insert(desc.pixelformat);
            }

            return pixelFormats;
        }

        static bool v4l2_check_input_capabilities(int handle) {
            v4l2_input v4l2Input{};

            if (V4L2Utils::xioctl(handle, VIDIOC_G_INPUT, &v4l2Input.index) == ERROR_CODE) {
                std::cerr << "ERROR: VIDIOC_G_INPUT - " << strerror(errno) << '\n';
                return false;
            }
            if (V4L2Utils::xioctl(handle, VIDIOC_ENUMINPUT, &v4l2Input) == ERROR_CODE) {
                std::cerr << "ERROR: VIDIOC_G_ENUMINPUT - " << strerror(errno) << '\n';
                return false;
            }
            if (v4l2Input.type != V4L2_INPUT_TYPE_CAMERA) {
                std::cerr << "ERROR: Not a video device\n";
                return false;
            }
            if (v4l2Input.status == V4L2_IN_ST_NO_POWER || v4l2Input.status == V4L2_IN_ST_NO_SIGNAL) {
                std::cerr << "ERROR: Device is off or there is no signal\n";
                return false;
            }
            return true;
        }

        static std::optional<v4l2_capability> v4l2_query_capabilities(int handle) {
            v4l2_capability capability{};

            if (V4L2Utils::xioctl(handle, VIDIOC_QUERYCAP, &capability) == ERROR_CODE) {
                std::cerr << "ERROR: Cannot execute VIDIOC_QUERYCAP - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            return {capability};
        }

        static bool v4l2_check_capabilities(v4l2_capability const &caps, uint32_t requiredCaps) {
            if (!(caps.capabilities & requiredCaps)) {
                std::cerr << "ERROR: Device does not support required capabilities\n";
                return false;
            }

            if (caps.capabilities & V4L2_CAP_TIMEPERFRAME) {
                std::cout << "NOTICE: Device supports for TIMEPERFRAME capability\n";
            }

            return true;
        }

        // Checks if given format is supported on v4l2 device w/o interrupting video capturing
        static std::optional<v4l2_format> v4l2_try_format(int handle, uint32_t pixFmt,
                                                          int width, int height) {
            v4l2_format format{};
            format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            format.fmt.pix.field = V4L2_FIELD_ANY;
            format.fmt.pix.pixelformat = pixFmt;
            format.fmt.pix.width = static_cast<uint32_t >(width);
            format.fmt.pix.height = static_cast<uint32_t >(height);

            if (V4L2Utils::xioctl(handle, VIDIOC_TRY_FMT, &format) == ERROR_CODE) {
                std::cerr << "ERROR: VIDIOC_TRY_FMT - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            if (format.fmt.pix.pixelformat != pixFmt
                || format.fmt.pix.width != static_cast<uint32_t >(width)
                || format.fmt.pix.height != static_cast<uint32_t >(height)) {
                return std::nullopt;
            }

            return {format};
        }

        // Sets given format to the v4l2 device (device should not be in streaming mode)
        static std::optional<v4l2_format> v4l2_set_format(int handle, uint32_t pixFmt,
                                                          int width, int height) {
            v4l2_format format{};
            format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            format.fmt.pix.field = V4L2_FIELD_ANY;
            format.fmt.pix.pixelformat = pixFmt;
            format.fmt.pix.width = static_cast<uint32_t >(width);
            format.fmt.pix.height = static_cast<uint32_t >(height);

            if (V4L2Utils::xioctl(handle, VIDIOC_S_FMT, &format) == ERROR_CODE) {
                std::cerr << "ERROR: VIDIOC_S_FMT - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            if (format.fmt.pix.pixelformat != pixFmt
                || format.fmt.pix.width != static_cast<uint32_t >(width)
                || format.fmt.pix.height != static_cast<uint32_t >(height)) {
                return std::nullopt;
            }

            return {format};
        }

        static std::optional<v4l2_format> v4l2_get_current_format(int handle) {
            v4l2_format format{};
            format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (V4L2Utils::xioctl(handle, VIDIOC_G_FMT, &format) == ERROR_CODE) {
                std::cerr << "ERROR: VIDIOC_G_FMT - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            return {format};
        }

        static std::optional<v4l2_streamparm> v4l2_get_current_frame_rate(int handle) {
            v4l2_streamparm streamParam{};
            streamParam.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (V4L2Utils::xioctl(handle, VIDIOC_G_PARM, &streamParam) == ERROR_CODE) {
                std::cerr << "ERROR: VIDIOC_G_PARM - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            return {streamParam};
        }

        // Sets frame rate on v4l2 device (device should not be in streaming mode)
        static std::optional<v4l2_streamparm> v4l2_set_frame_rate(int handle, int num, int den) {
            v4l2_streamparm streamParam{};
            streamParam.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            streamParam.parm.capture.timeperframe.numerator = static_cast<uint32_t >(num);
            streamParam.parm.capture.timeperframe.denominator = static_cast<uint32_t >(den);

            if (V4L2Utils::xioctl(handle, VIDIOC_S_PARM, &streamParam) == ERROR_CODE) {
                std::cerr << "ERROR: VIDIOC_S_PARM - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            return {streamParam};
        }

        template<typename T, typename std::enable_if<std::is_arithmetic<T>::value, T>::type * = nullptr>
        static bool is_in_range_inclusive(T low, T high, T value) {
            return value >= low && value <= high;
        }
    };

}  // namespace lirs

