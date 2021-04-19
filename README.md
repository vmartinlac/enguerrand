
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
