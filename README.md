# Video Streaming ROS package

Video streaming ROS package based on Video4Linux API (ver. 2).

It allows to concurrently capture video frames from multiple cameras of the [Servosila Engineer crawler-type mobile robot](https://www.servosila.com/en/index.shtml).

It makes use of Video4Linux2 API for low-level interaction with cameras.

Memory mapping (_mmap_) I/O approach is used in order to eliminate copies from the camera's driver into the application's memory.

## Paper
[R. Safin, and R. Lavrenov "Implementation of ROS package for simultaneous video streaming from several different cameras"](https://www.researchgate.net/publication/325903109_Implementation_of_ROS_package_for_simultaneous_video_streaming_from_several_different_cameras?origin=mail&uploadChannel=re390&reqAcc=Jenny_Midwinter&useStoredCopy=0)

The 2018 International Conference on Artificial Life and Robotics (ICAROB2018)At: B-Con Plaza, Beppu, Oita, Japan.
