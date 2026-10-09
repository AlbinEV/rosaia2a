# RGB-D ArUco shape tracker

This package converts synchronized RealSense colour/depth frames into the
six-dimensional visual state used by identification and control:

```text
x = [x1, y1, x2, y2, x3, y3]
```

The launch file runs the C++ implementation (`aruco_shape_tracker_cpp`). The
original Python geometry and Kalman modules remain as executable reference
specifications and unit tests. Building requires the RealSense development
headers in addition to the runtime already needed by the camera:

```bash
sudo apt install librealsense2-dev
```

The RGB image supplies sub-pixel ArUco corners. Depth is aligned to colour and
estimated as a robust median inside the central 60% of each marker, avoiding
mixed pixels at its edges. Marker centres are deprojected into metric 3-D
camera coordinates. The two distant base-marker centres define a common,
stable robot x axis; y is camera-down orthogonalized against it and points
along the arms. Each arm uses its own base centre as origin, while sharing
these axes. This avoids the ambiguous and noisy planar PnP orientation of one
small square. The z coordinate is retained for planarity diagnostics. The
recorder and learning pipeline continue to consume x and y, so their six-state
API is unchanged.

Markers 0 and 1 are rigid ground references. At startup they must both remain
visible for `reference_lock_frames` valid RGB-D frames. Their median 3-D
origins and shared rotation are then frozen; subsequent state publication no
longer requires either ground marker to remain visible. The three moving
markers of the relevant arm are still required in every published sample.

A constant-velocity 3-D Kalman filter stabilizes each measured marker. The
tracker never publishes prediction-only moving-marker samples: a training
sample is emitted only when all three moving markers have valid RGB-D
measurements in the current frameset and the ground frame has already locked.
Its initial noise values come from the stationary test and must be refined
during calibration.

| Topic | Type | Meaning |
|---|---|---|
| `/arm_1/aruco_state` | `sensor_msgs/PointCloud` | filtered metric points 2, 3, 4 in base-0 frame |
| `/arm_2/aruco_state` | `sensor_msgs/PointCloud` | filtered metric points 5, 6, 7 in base-1 frame |
| `/arm_1/aruco_points_3d_raw` | `sensor_msgs/PointCloud` | raw camera-frame points 2, 3, 4 |
| `/arm_2/aruco_points_3d_raw` | `sensor_msgs/PointCloud` | raw camera-frame points 5, 6, 7 |
| `/aruco/status` | `std_msgs/String` | JSON completeness plus per-stage timing |
| `/aruco/annotated/compressed` | `sensor_msgs/CompressedImage` | annotated RGB JPEG with depth labels |

The current D435 profile and serial are in `config/two_arms.yaml`. The physical
marker size is 30 mm, although centre deprojection does not depend on that size.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch aruco_shape_tracker aruco_shape_tracker.launch.py
```

On the development workstation, the C++ node sustains both arm states and the
annotated JPEG at 30 Hz for the 1280x720 RGB-D profile. The status timing fields
`acquire_align_ms`, `detect_ms`, `measure_publish_ms`, and `processing_ms`
separate camera alignment, ArUco detection, and the remaining reconstruction
work. They should be checked after camera or resolution changes.

The local z values currently measure optical depth relative to the respective
base centre. Because the base markers are physically closer to the camera than
the arm markers, z has a non-zero static offset. The upcoming plane calibration
will estimate that offset and the arm-plane normal; until then z is diagnostic
and is not part of the learned six-state vector.
