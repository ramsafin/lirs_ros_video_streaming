#ifndef V4L2VIDEO_STREAMER_V4L2CAPTURE_H
#define V4L2VIDEO_STREAMER_V4L2CAPTURE_H

#include <iostream>
#include <memory>
#include <vector>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#define DEFAULT_DEVICE_NAME "/dev/video1"
#define DEFAULT_BUFFER_SIZE 2
#define DEFAULT_FRAME_WIDTH 744
#define DEFAULT_FRAME_HEIGHT 480
#define DEFAULT_FPS 30
#define DEFAULT_PALETTE V4L2_PIX_FMT_SGRBG8
//V4L2_PIX_FMT_SGRBG8
//V4L2_PIX_FMT_YUYV

namespace lirs {

    /* ioctl wrapper */
    static int v4l2_ioctl(int deviceHandle, unsigned long int request, void *arg) {
        int status = 0;
        do {
            status = ioctl(deviceHandle, request, arg);
            // see https://stackoverflow.com/questions/41474299/checking-if-errno-eintr-what-does-it-mean
        } while (status == -1 && errno == EINTR);

        return status;
    }

    /* v4l2 mmap buffer structure */
    typedef struct {
        void *data;
        size_t length;
    } Buffer;

    /* v4l2 device capture */
    class V4L2Capture {
    public:

        /******* Constructors *******/

        V4L2Capture(const std::string &);

        /******* Prohibited copy constructor *******/
        V4L2Capture(const V4L2Capture &) = delete;

        /******* Prohibited copy assignment operator *******/
        V4L2Capture &operator=(const V4L2Capture &) = delete;

        /******* Destructor *******/
        ~V4L2Capture();

        /******* Public methods *******/

        bool openDevice(const std::string &);

        bool closeDevice();

        bool queryCap();

        bool tryPalette();

        bool tryFps();

        bool initBuffers();

        void terminateBuffers();

        bool enableStreaming();

        bool disableSteaming();

        int isStreamReadable();

        bool isOpened() const;

        bool initCapture(const std::string&);

        int readFrame();

        bool mainloop();

        /******* Getters *******/

        void* getCurrentFrameData();
        size_t getCurrentFrameSize() const;
        uint32_t getHeight() const;
        uint32_t getWidth() const;
        timeval getTimestamp() const;
        size_t getSequence() const;
        uint32_t getStep() const;

        /******* Setters *******/

    private:

        int _deviceHandle;

        std::string _deviceName;

        bool _isCapturing;

        uint32_t _width;
        uint32_t _height;

        uint32_t _bufferSize;
        uint32_t _palette;
        uint32_t _fps;

        std::vector<Buffer> _buffers;

        v4l2_format _format;
        v4l2_capability _capability;

        void* _currentFrameData;
        size_t _currentFrameSize;
        timeval _timestamp;
        size_t _sequence;
    };

    V4L2Capture::V4L2Capture(const std::string &deviceName) {
        printf(">>> Constructor\n");

        _width       = DEFAULT_FRAME_WIDTH;
        _height      = DEFAULT_FRAME_HEIGHT;
        _palette     = DEFAULT_PALETTE;
        _fps         = DEFAULT_FPS;
        _bufferSize  = DEFAULT_BUFFER_SIZE;
        _isCapturing      = false;
        _currentFrameSize = 0;
        _currentFrameData = nullptr;

        initCapture(deviceName);

        printf("Buffers: %d, fps: %d, width: %d, height: %d, step: %d, imgSize: %d\n",
               _bufferSize, _fps, _width, _height, _format.fmt.pix.bytesperline, _format.fmt.pix.sizeimage);
    }

    V4L2Capture::~V4L2Capture() {
        printf(">>> Destructor\n");
        disableSteaming();
        terminateBuffers();
        closeDevice();
    }


    bool V4L2Capture::openDevice(const std::string &deviceName) {

        printf("Open device: '%s'\n", deviceName.c_str());

        struct stat st;

        if (stat(deviceName.c_str(), &st) == -1) {
            fprintf(stderr, "Cannot identify device '%s': %s\n", _deviceName.c_str(), strerror(errno));
            return false;
        }

        if (!S_ISCHR(st.st_mode)) {
            fprintf(stderr, "Is not a char device '%s': %s\n", _deviceName.c_str(), strerror(errno));
            return false;
        }

        int handle = open(deviceName.c_str(), O_RDWR | O_NONBLOCK);

        if (handle == -1) {
            fprintf(stderr, "Cannot open device '%s': %s\n", _deviceName.c_str(), strerror(errno));
            return false;
        }

        // set device handle and name appropriately
        _deviceHandle = handle;
        _deviceName = deviceName;

        return true;
    }

    bool V4L2Capture::closeDevice() {

        printf("Close device: '%s'\n", _deviceName.c_str());

        if (close(_deviceHandle) == -1) {
            fprintf(stderr, "Cannot close device '%s': %s\n", _deviceName.c_str(), strerror(errno));
            return false;
        }

        // reset device handle and name appropriately
        _deviceHandle = -1;
        _deviceName.clear();

        return true;
    }

    bool V4L2Capture::queryCap() {

        printf("Query cap\n");

        auto cap = v4l2_capability();

        if (v4l2_ioctl(_deviceHandle, VIDIOC_QUERYCAP, &cap) == -1) {
            fprintf(stderr, "queryCap VIDIOC_QUERYCAP '%s': %s\n", _deviceName.c_str(), strerror(errno));
            return false;
        }

        _capability = cap;

        return true;
    }

    bool V4L2Capture::tryPalette() {

        printf("Try palette: %d\n", _palette);

        auto format = v4l2_format();

        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.field = V4L2_FIELD_INTERLACED;
        format.fmt.pix.pixelformat = _palette;
        format.fmt.pix.height = _height;
        format.fmt.pix.width = _width;

        if (v4l2_ioctl(_deviceHandle, VIDIOC_S_FMT, &format) == -1) {
            fprintf(stderr, "tryPalette %d VIDOC_S_FMT '%s'\n", _palette, strerror(errno));
            return false;
        }

        if (_palette != format.fmt.pix.pixelformat) {
          fprintf(stderr, "tryPalette %d is not supported by device", _palette);
          return false;
        }

        _format = format;

        _width = format.fmt.pix.width;
        _height = format.fmt.pix.height;

        return true;
    }

    bool V4L2Capture::tryFps() {

        auto fps = v4l2_streamparm();

        fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fps.parm.capture.timeperframe.numerator = 1;
        fps.parm.capture.timeperframe.denominator = _fps;

        if (v4l2_ioctl(_deviceHandle, VIDIOC_S_PARM, &fps) == -1) {
            fprintf(stderr, "tryFps %d VIDIOC_S_PARM %s\n", _fps, strerror(errno));
            return false;
        }

        _fps = fps.parm.capture.timeperframe.denominator;

        return true;
    }

    bool V4L2Capture::initBuffers() {

        printf("Init buffers\n");

        _buffers.reserve(_bufferSize);

        for (size_t idx = 0; idx < _bufferSize; ++idx) {
            _buffers.push_back(Buffer());
        }

        auto requestBuffers = v4l2_requestbuffers();

        requestBuffers.count = _bufferSize;
        requestBuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        requestBuffers.memory = V4L2_MEMORY_MMAP;

        if (v4l2_ioctl(_deviceHandle, VIDIOC_REQBUFS, &requestBuffers) == -1) {
            if (errno == EINVAL) {
                fprintf(stderr, "Device doesn't support memory mapping: '%s', %s\n", _deviceName.c_str(),
                        strerror(errno));
            } else {
                fprintf(stderr, "VIDIOC_REQBUFS\n");
            }
            return false;
        }

        if (requestBuffers.count < _bufferSize) {
            if (requestBuffers.count == 1) {
                fprintf(stderr, "Insufficient buffer memory on '%s'\n", _deviceName.c_str());
                // todo clean up
                return false;
            }
        }

        printf("Actual buffers number: %d\n", requestBuffers.count);

        for (uint32_t idx = 0; idx < requestBuffers.count; ++idx) {

            auto buf = v4l2_buffer();

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = idx;

            if (v4l2_ioctl(_deviceHandle, VIDIOC_QUERYBUF, &buf) == -1) {
                perror("VIDIOC_QUERYBUF");
                // todo clean up
                return false;
            }

            _buffers[idx].length = buf.length;

            _buffers[idx].data = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, _deviceHandle,
                                      buf.m.offset);

            if (_buffers[idx].data == MAP_FAILED) {
                // todo clean up
                perror("MMAP");
                return false;
            }
        }

        /* initialize current buffer */
        auto bufLength = _buffers[0].length;

        _currentFrameSize = bufLength;
        // todo may be error use malloc
//        _currentFrameData = ::operator new (bufLength);
        _currentFrameData = malloc(bufLength);

        return true;
    }

    void V4L2Capture::terminateBuffers() {

        printf("Terminate buffers\n");

        for (auto &buf: _buffers) {
            if (munmap(buf.data, buf.length) == -1) {
                perror("MUNMAP");
            }
        }

        if (_currentFrameData) {
            // note deleting *void
//            delete (uint8_t*) _currentFrameData;
            free(_currentFrameData);
        }
    }

    bool V4L2Capture::enableStreaming() {

        if (!_isCapturing) {

            printf("Enable streaming\n");

            auto buf = v4l2_buffer();

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            /*
            * NOTE: some devices will refuse to get into streaming mode
            * if there aren't already buffers queued
            */
            for (uint32_t idx = 0; idx < _bufferSize; ++idx) {

                buf.index = idx;

                if (v4l2_ioctl(_deviceHandle, VIDIOC_QBUF, &buf) == -1) {
                    fprintf(stderr, "enable stream VIDIOC_QBUF: %s\n", strerror(errno));
                    return false;
                }
            }

            // enable streaming
            if (v4l2_ioctl(_deviceHandle, VIDIOC_STREAMON, &buf.type) == -1) {
                perror("VIDIOC_STREAMON");
                return false;
            }

            _isCapturing = true;

            return true;
        }

        return false;
    }

    bool V4L2Capture::disableSteaming() {

        if (_isCapturing) {

            printf("Disable streaming\n");

            auto bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (v4l2_ioctl(_deviceHandle, VIDIOC_STREAMOFF, &bufType) == -1) {
                perror("Unable to stop the stream");
                return false;
            }

            _isCapturing = false;

            return true;
        }

        return false;
    }

    int V4L2Capture::isStreamReadable() {

        fd_set fds;

        FD_ZERO(&fds);
        FD_SET(_deviceHandle, &fds);

        struct timeval time;
        time.tv_sec  = 10;
        time.tv_usec = 0;

        /*
         * check whether specified device with file descriptor is ready for
         * reading/writing operation or has an error condition
         */
        return select(_deviceHandle + 1, &fds, nullptr, nullptr, &time);
    }

    bool V4L2Capture::initCapture(const std::string& deviceName) {

        if (!openDevice(deviceName)) {
            // todo clean up
            return false;
        }

        if (!queryCap()) {
            // todo clean up
            return false;
        }

        if (!(_capability.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            fprintf(stderr, "queryCap Not video capture device: '%s'\n", _deviceName.c_str());
            return false;
        }

        if (!(_capability.capabilities & V4L2_CAP_STREAMING)) {
            fprintf(stderr, "queryCap Device doesn't support streaming I/O: '%s'\n", _deviceName.c_str());
            return false;
        }

        if (!tryPalette()) {
            // todo clean up
            // todo autosetup
            return false;
        }

        tryFps();

        if (!initBuffers()) {
            // todo clean up
            return false;
        }

        enableStreaming();

        // create frame
        // firstCapture flag

        return true;
    }

    int V4L2Capture::readFrame() {

        auto buf = v4l2_buffer();

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // get frame from driver's outgoing queue
        if (v4l2_ioctl(_deviceHandle, VIDIOC_DQBUF, &buf) == -1) {
            switch (errno) {
                case EAGAIN:
                    return 0;
                case EIO:
                    if (!buf.flags & (V4L2_BUF_FLAG_QUEUED | V4L2_BUF_FLAG_DONE)) {
                        if (v4l2_ioctl(_deviceHandle, VIDIOC_QBUF, &buf)) {
                            return 0;
                        }
                    }
                    return 0;
                default:
                    // todo  set flag to false, returnFrame
                    perror("VIDIOC_DQBUF");
                    return -1;
            }
        }

        auto currentFrame = _buffers[buf.index];

        /* CPU consuming code */
        memcpy(_currentFrameData, currentFrame.data, currentFrame.length);
        _currentFrameSize = currentFrame.length;

        _timestamp = buf.timestamp;
        _sequence  = buf.sequence;

        if (v4l2_ioctl(_deviceHandle, VIDIOC_QBUF, &buf) == -1) {
            perror("VIDIOC_QBUF");
        }

        return 1;
    }

    bool V4L2Capture::mainloop() {

        for (;;) {

            int retCode = isStreamReadable();

            if (retCode == -1) {
                if (errno == EINTR) continue;
                perror("Select");
            }

            if (retCode == 0) {
                perror("select timeout");
                break;
            }

            auto readStatusCode = readFrame();

            if (readStatusCode == -1) {
                return false;
            }

            if (readStatusCode == 1) {
                break; // success
            }
        }

        return true;
    }

    uint32_t V4L2Capture::getHeight() const {
      return _height;
    }

    uint32_t V4L2Capture::getWidth() const {
      return _width;
    }

    size_t V4L2Capture::getSequence() const {
      return _sequence;
    }

    timeval V4L2Capture::getTimestamp() const {
      return _timestamp;
    }

    uint32_t V4L2Capture::getStep() const {
      return _format.fmt.pix.bytesperline;
    }

    size_t V4L2Capture::getCurrentFrameSize() const {
      return _currentFrameSize;
    }

    void* V4L2Capture::getCurrentFrameData() {
        return _currentFrameData;
    }

    bool V4L2Capture::isOpened() const {
      return _deviceHandle != -1 && !_deviceName.empty();
    }
}

#endif //V4L2VIDEO_STREAMER_V4L2CAPTURE_H
