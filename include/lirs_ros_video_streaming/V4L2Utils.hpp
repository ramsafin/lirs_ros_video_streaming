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
    struct V4L2Constants {
        static constexpr auto CLOSED_HANDLE = -1;  /* closed file descriptor */
        static constexpr auto V4L2_MAX_BUFFER_SIZE = 32u;  /* maximum number of v4l2 buffers */
    };

    /**
     * @brief V4L2 utility functions.
     */
    struct V4L2Utils {
        /* ioctl wrapper */
        static inline int xioctl(int handle, unsigned long int request, void *arg) {
            int status = 0;
            do {
                // see https://stackoverflow.com/questions/41474299/checking-if-errno-eintr-what-does-it-mean
                status = ioctl(handle, request, arg);
            } while (status == -1 && errno == EINTR);

            return status;
        }

        // Checks v4l2 device status, i.e. if it is a character device
        static bool v4l2_check_device_status(std::string const &device) {
            if (struct stat status{}; stat(device.c_str(), &status) != -1) {
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

        /**
         * @brief Opens v4l2 device.
         *
         * @param device resource url.
         * @return on success - positive file descriptor, otherwise - negative one.
         */
        static inline int open_handle(std::string const &device) {
            if (!V4L2Utils::v4l2_check_device_status(device)) return V4L2Constants::CLOSED_HANDLE;

            if (auto handle = open(device.c_str(), O_RDWR | O_NONBLOCK); handle != V4L2Constants::CLOSED_HANDLE) {
                return handle;
            }

            std::cerr << "ERROR: Cannot open v4l2 device " << device << " - " << strerror(errno) << '\n';

            return V4L2Constants::CLOSED_HANDLE;
        }

        /**
         * @brief Closes v4l2 device represented by file descriptor.
         *
         * @param handle v4l2 device file decriptor.
         * @return true - on success, otherwise - false.
         */
        static bool close_handle(int handle) {
            if (handle == lirs::V4L2Constants::CLOSED_HANDLE) return false;

            if (close(handle) == -1) {
                std::cerr << "ERROR: Cannot close v4l2 device - " << strerror(errno) << '\n';
                return false;
            }
            return true;
        }

        static std::set<uint32_t> v4l2_query_pixel_formats(int fd) {
            std::set<uint32_t> pixelFormats;

            v4l2_fmtdesc desc{};
            desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            while (V4L2Utils::xioctl(fd, VIDIOC_ENUM_FMT, &desc) != -1) {
                ++desc.index;
                pixelFormats.insert(desc.pixelformat);
            }

            return pixelFormats;
        }

        static bool v4l2_check_input_capabilities(int handle) {
            v4l2_input v4l2Input{};

            if (V4L2Utils::xioctl(handle, VIDIOC_G_INPUT, &v4l2Input.index) == -1) {
                std::cerr << "ERROR: VIDIOC_G_INPUT - " << strerror(errno) << '\n';
                return false;
            }
            if (V4L2Utils::xioctl(handle, VIDIOC_ENUMINPUT, &v4l2Input) == -1) {
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

            if (V4L2Utils::xioctl(handle, VIDIOC_QUERYCAP, &capability) == -1) {
                std::cerr << "ERROR: Cannot execute VIDIOC_QUERYCAP - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            return std::optional{capability};
        }

        static bool v4l2_check_capabilities(v4l2_capability const &caps, long requiredCaps) {
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
                                                          uint32_t width, uint32_t height) {
            v4l2_format format{};
            format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            format.fmt.pix.field = V4L2_FIELD_ANY;
            format.fmt.pix.pixelformat = pixFmt;
            format.fmt.pix.width = width;
            format.fmt.pix.height = height;

            if (V4L2Utils::xioctl(handle, VIDIOC_TRY_FMT, &format) == -1) {
                std::cerr << "ERROR: VIDIOC_TRY_FMT - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            if (format.fmt.pix.pixelformat != pixFmt || format.fmt.pix.width != width
                || format.fmt.pix.height != height) {
                return std::nullopt;
            }

            return std::optional{format};
        }

        // Sets given format to the v4l2 device (device should not be in streaming mode)
        static std::optional<v4l2_format> v4l2_set_format(int handle, uint32_t pixFmt,
                                                          uint32_t width, uint32_t height) {
            v4l2_format format{};
            format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            format.fmt.pix.field = V4L2_FIELD_ANY;
            format.fmt.pix.pixelformat = pixFmt;
            format.fmt.pix.width = width;
            format.fmt.pix.height = height;

            if (V4L2Utils::xioctl(handle, VIDIOC_S_FMT, &format) == -1) {
                std::cerr << "ERROR: VIDIOC_S_FMT - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            if (format.fmt.pix.pixelformat != pixFmt || format.fmt.pix.width != width
                || format.fmt.pix.height != height) {
                return std::nullopt;
            }

            return std::optional{format};
        }

        // Gets default format of the v4l2 device (e.g. set by v4l-ctl)
        static std::optional<v4l2_format> v4l2_get_format(int handle) {
            v4l2_format format{};
            format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (V4L2Utils::xioctl(handle, VIDIOC_G_FMT, &format) == -1) {
                std::cerr << "ERROR: VIDIOC_G_FMT - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            return std::optional{format};
        }

        // Gets default frame rate of the v4l2 device (e.g. set by v4l-ctl)
        static std::optional<v4l2_streamparm> v4l2_get_frame_rate(int handle) {
            v4l2_streamparm streamParam{};
            streamParam.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (V4L2Utils::xioctl(handle, VIDIOC_G_PARM, &streamParam) == -1) {
                std::cerr << "ERROR: VIDIOC_G_PARM - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            return std::optional{streamParam};
        }

        // Sets frame rate on v4l2 device (device should not be in streaming mode)
        static std::optional<v4l2_streamparm> v4l2_set_frame_rate(int handle, uint32_t num, uint32_t den) {
            v4l2_streamparm streamParam{};
            streamParam.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            streamParam.parm.capture.timeperframe.numerator = num;
            streamParam.parm.capture.timeperframe.denominator = den;

            if (V4L2Utils::xioctl(handle, VIDIOC_S_PARM, &streamParam) == -1) {
                std::cerr << "ERROR: VIDIOC_S_PARM - " << strerror(errno) << '\n';
                return std::nullopt;
            }

            return std::optional{streamParam};
        }

        // Checks if v4l2 device is ready to read operation
        static inline bool v4l2_is_readable(int handle, timeval timeout = {1, 0}) {
            fd_set fds{};
            FD_ZERO(&fds);
            FD_SET(handle, &fds);

            return select(handle + 1, &fds, nullptr, nullptr, &timeout) == 1;
        }

        // Checks if given numeric value is in the specified range
        template<typename T, typename std::enable_if<std::is_arithmetic<T>::value, T>::type * = nullptr>
        static bool is_in_range_inclusive(T low, T high, T value) {
            return value >= low && value <= high;
        }
    };

}  // namespace lirs

