ENGUERRAND
==========

Dependencies
------------

* Ceres
* Eigen3
* OpenCV
* openscenegraph
* Qt5
* realsense2
* Sophus
* Threading Building Blocks

Added value
-----------

* Implementation of various SLAM methods:
    * EKF SLAM
    * Graph SLAM
    * Fast SLAM
* Custom algorithm to track ping-pong balls in video stream.
* User interface to inspect pose and map provided by SLAM algorithms.

TODO
----

* update Engine in order to support AsynchronousVideoSource.
* write particle filter odometry code.
* write 3d visualization.
* write GPU implementation of landmark detection.
* improve performances of CPU implementation of landmark detection.
* use { bool first\_seen; size\_t id; } or { bool first\_seen; size\_t last\_index; } and track on multiple frames.
* check histogram with neural network instead of using distance to reference histogram.
* test everything.

