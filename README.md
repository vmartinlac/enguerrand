
ENGUERRAND
==========

Dependencies
------------

* Ceres
* Eigen3
* libsvm (included as git submodule)
* OpenCV
* openscenegraph
* Qt5
* realsense2
* Sophus
* Threading Building Blocks (included as git submodule)

Added value
-----------

* Custom algorithm to track ping-pong balls in video stream.
* Implementation of various SLAM methods:
    * EKF SLAM
    * Graph SLAM
    * Fast SLAM
* User interface to inspect pose and map provided by SLAM algorithms.

Computer vision algorithms are multi-threaded according to task-based parallelism.

TODO
----

* Validate Kalman filter odometry.
* Write particle filter odometry.
* Check histogram with one-class SVM instead of using distance to some reference histogram. Store SVM model in Qt ressource file.
* Write GPU implementation of edge detection.
* (nice to have) Optimize performances of CPU implementation of landmark detection.
* (nice to have) Multi-frame landmark tracking.

