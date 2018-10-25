#pragma once

#include "VideoCapture.hpp"

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdint>
#include <cstring>
#include <cerrno>

#include <type_traits>
#include <optional>
#include <atomic>
#include <vector>
#include <set>
#include <map>

namespace lirs {
    namespace constants {
        static constexpr auto CLOSED_HANDLE = -1;  /* closed file descriptor */
        static constexpr auto V4L2_MAX_BUFFER_SIZE = 32u;  /* maximum number of v4l2 buffers */
    }  // namespace constants

    namespace utils {
        struct V4L2Utils {
            /* ioctl wrapper */
            static inline int xioctl(int handle, unsigned long int request, void *arg) {
                int status{};
                do {
                    // see https://stackoverflow.com/questions/41474299/checking-if-errno-eintr-what-does-it-mean
                    status = ioctl(handle, request, arg);
                } while (status == -1 && errno == EINTR);

                return status;
            }

            static bool v4l2_check_device_status(std::string const &device) {
                if (struct stat status{}; stat(device.c_str(), &status) != -1) {
                    if (!S_ISCHR(status.st_mode)) {
                        std::cerr << "ERROR: " << device << " is not a character v4l2 device - " << strerror(errno)
                                  << '\n';
                        return false;
                    }
                    return true;
                }

                std::cerr << "ERROR: Cannot identify device - " << device << " - " << strerror(errno) << '\n';

                return false;
            }

            static inline int open_handle(std::string const &device) {
                if (!utils::V4L2Utils::v4l2_check_device_status(device)) return lirs::constants::CLOSED_HANDLE;

                if (auto handle = open(device.c_str(), O_RDWR | O_NONBLOCK); handle != lirs::constants::CLOSED_HANDLE) {
                    return handle;
                }

                std::cerr << "ERROR: Cannot open v4l2 device " << device << " - " << strerror(errno) << '\n';

                return lirs::constants::CLOSED_HANDLE;
            }

            static bool close_handle(int handle) {
                if (handle == lirs::constants::CLOSED_HANDLE) return false;

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

                while (utils::V4L2Utils::xioctl(fd, VIDIOC_ENUM_FMT, &desc) != -1) {
                    ++desc.index;
                    pixelFormats.insert(desc.pixelformat);
                }

                return pixelFormats;
            }

            static bool v4l2_check_input_capabilities(int handle) {
                v4l2_input v4l2Input{};

                if (utils::V4L2Utils::xioctl(handle, VIDIOC_G_INPUT, &v4l2Input.index) == -1) {
                    std::cerr << "ERROR: VIDIOC_G_INPUT - " << strerror(errno) << '\n';
                    return false;
                }
                if (utils::V4L2Utils::xioctl(handle, VIDIOC_ENUMINPUT, &v4l2Input) == -1) {
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

                if (utils::V4L2Utils::xioctl(handle, VIDIOC_QUERYCAP, &capability) == -1) {
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

            static std::optional<v4l2_format> v4l2_try_format(int handle, uint32_t pixFmt,
                                                              uint32_t width, uint32_t height) {
                v4l2_format format{};
                format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                format.fmt.pix.field = V4L2_FIELD_ANY;
                format.fmt.pix.pixelformat = pixFmt;
                format.fmt.pix.width = width;
                format.fmt.pix.height = height;

                if (utils::V4L2Utils::xioctl(handle, VIDIOC_TRY_FMT, &format) == -1) {
                    std::cerr << "ERROR: VIDIOC_TRY_FMT - " << strerror(errno) << '\n';
                    return std::nullopt;
                }

                if (format.fmt.pix.pixelformat != pixFmt || format.fmt.pix.width != width
                    || format.fmt.pix.height != height) {
                    return std::nullopt;
                }

                return std::optional{format};
            }

            static std::optional<v4l2_format> v4l2_set_format(int handle, uint32_t pixFmt,
                                                              uint32_t width, uint32_t height) {
                v4l2_format format{};
                format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                format.fmt.pix.field = V4L2_FIELD_ANY;
                format.fmt.pix.pixelformat = pixFmt;
                format.fmt.pix.width = width;
                format.fmt.pix.height = height;

                if (utils::V4L2Utils::xioctl(handle, VIDIOC_S_FMT, &format) == -1) {
                    std::cerr << "ERROR: VIDIOC_S_FMT - " << strerror(errno) << '\n';
                    return std::nullopt;
                }

                if (format.fmt.pix.pixelformat != pixFmt || format.fmt.pix.width != width
                    || format.fmt.pix.height != height) {
                    return std::nullopt;
                }

                return std::optional{format};
            }

            static std::optional<v4l2_format> v4l2_get_format(int handle) {
                v4l2_format format{};
                format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                if (utils::V4L2Utils::xioctl(handle, VIDIOC_G_FMT, &format) == -1) {
                    std::cerr << "ERROR: VIDIOC_G_FMT - " << strerror(errno) << '\n';
                    return std::nullopt;
                }

                return std::optional{format};
            }

            static std::optional<v4l2_streamparm> v4l2_get_frame_rate(int handle) {
                v4l2_streamparm streamParam{};
                streamParam.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                if (utils::V4L2Utils::xioctl(handle, VIDIOC_G_PARM, &streamParam) == -1) {
                    std::cerr << "ERROR: VIDIOC_G_PARM - " << strerror(errno) << '\n';
                    return std::nullopt;
                }

                return std::optional{streamParam};
            }

            static std::optional<v4l2_streamparm> v4l2_set_frame_rate(int handle, uint32_t num, uint32_t den) {
                v4l2_streamparm streamParam{};
                streamParam.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                streamParam.parm.capture.timeperframe.numerator = num;
                streamParam.parm.capture.timeperframe.denominator = den;

                if (utils::V4L2Utils::xioctl(handle, VIDIOC_S_PARM, &streamParam) == -1) {
                    std::cerr << "ERROR: VIDIOC_S_PARM - " << strerror(errno) << '\n';
                    return std::nullopt;
                }

                return std::optional{streamParam};
            }

            static inline bool v4l2_is_readable(int handle, timeval timeout = {1, 0}) {
                fd_set fds{};
                FD_ZERO(&fds);
                FD_SET(handle, &fds);

                return select(handle + 1, &fds, nullptr, nullptr, &timeout) == 1;
            }

            template<typename T, typename std::enable_if<std::is_arithmetic<T>::value, T>::type * = nullptr>
            static bool is_in_range_inclusive(T low, T high, T value) {
                return value >= low && value <= high;
            }
        };
    }  // namespace utils

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
                             uint32_t v4l2PixFmt = defaults::DEFAULT_V4L2_PIXEL_FORMAT,
                             uint32_t width = defaults::DEFAULT_WIDTH,
                             uint32_t height = defaults::DEFAULT_HEIGHT,
                             uint32_t frameRate = defaults::DEFAULT_FRAME_RATE,
                             uint32_t bufferSize = defaults::DEFAULT_V4L2_BUFFER_SIZE);

        /**
         * @brief Cleans up allocated resources and frees video resource acquisition.
         */
        ~V4L2Capture() override {
            if (IsOpened() && IsStreaming()) {
                disableSteaming();
                cleanupBuffers();
                if (utils::V4L2Utils::close_handle(handle_)) {
                    handle_ = constants::CLOSED_HANDLE;  // IsOpened() = false
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