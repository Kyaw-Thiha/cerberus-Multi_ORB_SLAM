# multicam_slam_ros2_wrapper

A minimal `ament_cmake` ROS2 package that wraps the pre-built Multi_ORB_SLAM
core library (`lib/libORB_SLAM2.so`, one directory up -- this fork is based
on ORB-SLAM2, extended for a synchronized dual-RGBD-camera setup) in a
single `rclcpp` node. It does not rebuild the core library; it links
against the already-compiled shared library plus its DBoW2/g2o Thirdparty
libs.

RGB-D only (dual camera), matching the original ROS1 `Examples/ROS/ORB_SLAM2`
entry point (`ros_rgbd.cc`) this wrapper replaces for ROS2. That ROS1 code is
left in place, untouched, alongside this package.

## Node: `multicam_slam_node`

### Parameters (required, no defaults)

| Parameter           | Type   | Description                                          |
|----------------------|--------|-------------------------------------------------------|
| `vocabulary_file`    | string | Path to the ORB vocabulary file (e.g. `ORBvoc.txt`)    |
| `settings_file`      | string | Path to the camera/settings YAML (e.g. `OtherFiles/multi.yaml`) -- shared intrinsics for both cameras |
| `calibration_file`   | string | Path to the plain-text 4x3 extrinsic calibration file between the two cameras (e.g. `OtherFiles/calibration.txt`) |

All three must be supplied at launch; the node fails fast at startup if any
is empty. Example:

```bash
ros2 run multicam_slam_ros2_wrapper multicam_slam_node \
  --ros-args \
  -p vocabulary_file:=/path/to/ORBvoc.txt \
  -p settings_file:=/path/to/multi.yaml \
  -p calibration_file:=/path/to/calibration.txt
```

### Parameters (optional, topic overrides)

| Parameter              | Default                        |
|--------------------------|-------------------------------|
| `camera1_rgb_topic`      | `/camera_01/rgb/image_raw`    |
| `camera1_depth_topic`    | `/camera_01/depth/image_raw`  |
| `camera2_rgb_topic`      | `/camera_02/rgb/image_raw`    |
| `camera2_depth_topic`    | `/camera_02/depth/image_raw`  |

These default to the exact same topic names the original ROS1 `ros_rgbd.cc`
subscribed to.

The `ORB_SLAM2::System` is constructed once at startup in
`ORB_SLAM2::System::RGBD` mode with the viewer disabled (`bUseViewer=false`)
-- this fork's core library is built without Pangolin (see the top-level
`CMakeLists.txt` Pangolin-optional change), so the viewer is always off
regardless.

### Subscribed topics

Four `sensor_msgs/msg/Image` topics (the two camera pairs above), synced
with `message_filters::sync_policies::ApproximateTime` at queue depth 10 --
same synchronizer policy and queue depth as the original ROS1 `ros_rgbd.cc`.
Each image is converted via `cv_bridge::toCvShare()` before being passed to
`ORB_SLAM2::System::TrackRGBD()`. Depth images are expected in the same
16-bit millimeter convention the core library's `Tracking::GrabImageRGBD`
already assumes (see `DepthMapFactor` in the settings YAML).

### Published topic

- `/multicam_slam/odom` (`nav_msgs/msg/Odometry`)
  - `header.frame_id`: `"map"`
  - `child_frame_id`: `"camera"`
  - `header.stamp`: copied from camera 1's RGB image stamp
  - `pose.pose.position`: camera translation in the world frame
  - `pose.pose.orientation`: camera rotation in the world frame, as a
    quaternion
  - Covariance fields are left at zero (not populated).

**This publisher is new functionality, not a port.** The original ROS1
`ros_rgbd.cc` never published live pose to any topic -- `TrackRGBD()`'s
return value (`Tcw`) was discarded every frame and only ever written to a
TUM-format trajectory file (`SLAM.SaveTrajectoryTUM(...)`) on shutdown.

`TrackRGBD()` returns `Tcw` (world-to-camera, `CV_32F`, empty when tracking
is lost/uninitialized); the node inverts it to `Twc` (camera pose in the
world frame) before publishing, and skips publishing entirely on an empty
result rather than emitting a bogus pose.

### Shutdown

`ORB_SLAM2::System::Shutdown()` is called from the node's destructor, which
runs when the `rclcpp::Node` shared_ptr goes out of scope after
`rclcpp::spin()` returns (i.e. on a clean `rclcpp::shutdown()`, e.g.
Ctrl-C).

## Building

This package depends on the already-built core library at
`../lib/libORB_SLAM2.so` and `../Thirdparty/{DBoW2,g2o}/lib/*.so` -- build
those first (see the top-level `CMakeLists.txt` / `build.sh` in the parent
directory) before building this package.

It also depends on ROS2 (`rclcpp`, `sensor_msgs`, `nav_msgs`,
`message_filters`) and `cv_bridge`. Build with `colcon`, e.g. as a
standalone one-package workspace:

```bash
colcon build --symlink-install --base-paths ros2_wrapper \
  --build-base /tmp/multicam_ws/build --install-base /tmp/multicam_ws/install
```

### Known environment quirks on this build host

These are specific to the dev machine this was verified on, not to the
package itself (the same quirks as `orb_slam3/ros2_wrapper`'s README,
since both wrappers were verified on the same pixi environment):

- `cv_bridge` was not a declared dependency of the surrounding pixi
  environment at the time of writing; it was resolved from a locally
  cached conda package instead (added to `CMAKE_PREFIX_PATH` for the
  build). If your ROS2 install already provides `cv_bridge`, this is a
  non-issue.
- `rclcpp` links against `liblttng-ust`; if your linker reports
  `cannot find -llttng-ust`, make sure that library is on
  `LIBRARY_PATH`/`LD_LIBRARY_PATH`.
- Using the cached (not properly environment-installed) `cv_bridge`
  package above pulls in a *second*, differently-versioned OpenCV via its
  exported `ament_cmake` dependencies (its `cv_bridge-extras.cmake` bakes
  in an `OpenCV_CONFIG_PATH` from the machine that built the cached
  package, which does not exist here), which conflicted at link time with
  the system OpenCV this package's own `find_package(OpenCV 4 REQUIRED)`
  correctly resolves (`undefined reference to cv::error(...)` from a
  mismatched `libopencv_core`). Fix: do not list `OpenCV` in this
  package's own `ament_target_dependencies()` call -- `cv_bridge` already
  transitively exports it, and letting `cv_bridge`'s (broken, in this
  environment) copy win at CMake's `find_package` resolution time is what
  caused the mismatch. This package links OpenCV only via its own
  directly-resolved `${OpenCV_LIBS}` in `target_link_libraries()`.
- The system OpenCV package on this host pulls in `libopencv_viz` ->
  VTK, and one VTK module (`libvtkIOExport`) has an unresolved `libfmt`
  symbol of its own that only the core library (not this wrapper) touches
  via `${OpenCV_LIBS}`. Since neither the core library's tracking code
  (viewer disabled) nor this node use viz/VTK functionality, the
  executable is linked with `-Wl,--allow-shlib-undefined` to skip
  resolving symbols in that unused, indirectly-linked library.
