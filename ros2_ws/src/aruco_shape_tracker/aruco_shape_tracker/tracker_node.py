"""ROS 2 RGB-D ArUco tracker for the two deformable arms."""

from __future__ import annotations

import json

import cv2
from geometry_msgs.msg import Point32
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import ChannelFloat32, CompressedImage, PointCloud
from std_msgs.msg import String

from .geometry import quadrilateral_area
from .kalman import PositionKalman3D
from .rgbd import (
    blend_rotation,
    frozen_reference_frame,
    project_to_base_frame,
    robot_rotation_from_bases,
    robust_marker_depth,
)


class RealSenseSource:
    """Acquire synchronized colour/depth frames aligned in colour pixels."""

    def __init__(self, serial: str, width: int, height: int, fps: int) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as error:
            raise RuntimeError(
                'pyrealsense2 is required for camera_backend=realsense_rgbd'
            ) from error
        self._rs = rs
        self._pipeline = rs.pipeline()
        configuration = rs.config()
        if serial:
            configuration.enable_device(serial)
        configuration.enable_stream(
            rs.stream.color, width, height, rs.format.bgr8, fps
        )
        configuration.enable_stream(
            rs.stream.depth, width, height, rs.format.z16, fps
        )
        profile = self._pipeline.start(configuration)
        device = profile.get_device()
        self.serial = device.get_info(rs.camera_info.serial_number)
        self._align = rs.align(rs.stream.color)
        self._depth_scale = device.first_depth_sensor().get_depth_scale()

    def read(self):
        frames = self._pipeline.poll_for_frames()
        if not frames:
            return None
        aligned = self._align.process(frames)
        colour_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not colour_frame or not depth_frame:
            return None
        colour = np.asanyarray(colour_frame.get_data())
        depth_m = (
            np.asanyarray(depth_frame.get_data()).astype(np.float32)
            * self._depth_scale
        )
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        return colour, depth_m, intrinsics

    def deproject(self, intrinsics, pixel, depth_m: float) -> np.ndarray:
        point = self._rs.rs2_deproject_pixel_to_point(
            intrinsics, [float(pixel[0]), float(pixel[1])], float(depth_m)
        )
        return np.asarray(point, dtype=np.float64)

    def close(self) -> None:
        self._pipeline.stop()


class ArucoShapeTracker(Node):
    """Publish metric marker states in each arm's base-marker frame."""

    def __init__(self) -> None:
        super().__init__('aruco_shape_tracker')
        self.declare_parameter('camera_backend', 'realsense_rgbd')
        self.declare_parameter('camera_serial', '')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('minimum_reference_area_px2', 80.0)
        self.declare_parameter('minimum_base_baseline_m', 0.10)
        self.declare_parameter('depth_inner_fraction', 0.60)
        self.declare_parameter('minimum_depth_samples', 10)
        self.declare_parameter('minimum_depth_m', 0.10)
        self.declare_parameter('maximum_depth_m', 5.0)
        self.declare_parameter('enable_kalman', True)
        self.declare_parameter('kalman_lateral_std_m', 0.0006)
        self.declare_parameter('kalman_depth_std_m', 0.0015)
        self.declare_parameter('kalman_acceleration_std_m_s2', 0.50)
        self.declare_parameter('kalman_reset_gap_s', 0.50)
        self.declare_parameter('base_rotation_filter_alpha', 0.15)
        self.declare_parameter('freeze_reference_markers', True)
        self.declare_parameter('reference_lock_frames', 30)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('arm_names', ['arm_1'])
        self.declare_parameter('reference_marker_ids', [0])
        self.declare_parameter('moving_marker_ids', [2, 3, 4])
        self.declare_parameter('output_topics', ['/arm_1/aruco_state'])
        self.declare_parameter(
            'raw_3d_topics', ['/arm_1/aruco_points_3d_raw']
        )

        backend = str(self.get_parameter('camera_backend').value)
        if backend != 'realsense_rgbd':
            raise ValueError('only camera_backend=realsense_rgbd is supported')
        self._serial = str(self.get_parameter('camera_serial').value)
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._fps = int(self.get_parameter('fps').value)
        self._minimum_reference_area = float(
            self.get_parameter('minimum_reference_area_px2').value
        )
        self._minimum_baseline = float(
            self.get_parameter('minimum_base_baseline_m').value
        )
        self._depth_inner_fraction = float(
            self.get_parameter('depth_inner_fraction').value
        )
        self._minimum_depth_samples = int(
            self.get_parameter('minimum_depth_samples').value
        )
        self._minimum_depth = float(
            self.get_parameter('minimum_depth_m').value
        )
        self._maximum_depth = float(
            self.get_parameter('maximum_depth_m').value
        )
        self._enable_kalman = bool(
            self.get_parameter('enable_kalman').value
        )
        self._rotation_alpha = float(
            self.get_parameter('base_rotation_filter_alpha').value
        )
        self._freeze_references = bool(
            self.get_parameter('freeze_reference_markers').value
        )
        self._reference_lock_frames = int(
            self.get_parameter('reference_lock_frames').value
        )
        self._publish_image = bool(
            self.get_parameter('publish_annotated_image').value
        )
        if min(self._width, self._height, self._fps) <= 0:
            raise ValueError('camera dimensions and fps must be positive')
        if not 0.0 < self._rotation_alpha <= 1.0:
            raise ValueError('base_rotation_filter_alpha must be in (0, 1]')
        if self._reference_lock_frames <= 0:
            raise ValueError('reference_lock_frames must be positive')

        arm_names = self._strings('arm_names')
        reference_ids = self._integers('reference_marker_ids')
        moving_flat = self._integers('moving_marker_ids')
        output_topics = self._strings('output_topics')
        raw_topics = self._strings('raw_3d_topics')
        if not arm_names or not (
            len(reference_ids)
            == len(arm_names)
            == len(output_topics)
            == len(raw_topics)
            and len(moving_flat) == 3 * len(arm_names)
        ):
            raise ValueError('arm configuration lengths are inconsistent')

        self._arms = []
        for index, name in enumerate(arm_names):
            moving = tuple(moving_flat[3 * index:3 * index + 3])
            if reference_ids[index] in moving:
                raise ValueError(f'{name}: reference ID also appears as moving ID')
            state_publisher = self.create_publisher(
                PointCloud, output_topics[index], qos_profile_sensor_data
            )
            raw_publisher = self.create_publisher(
                PointCloud, raw_topics[index], qos_profile_sensor_data
            )
            self._arms.append({
                'name': name,
                'reference_id': reference_ids[index],
                'moving_ids': moving,
                'state_publisher': state_publisher,
                'raw_publisher': raw_publisher,
            })

        dictionary_name = str(self.get_parameter('dictionary').value)
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f'unsupported ArUco dictionary: {dictionary_name}')
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(dictionary_id), parameters
        )

        filter_arguments = (
            float(self.get_parameter('kalman_lateral_std_m').value),
            float(self.get_parameter('kalman_depth_std_m').value),
            float(self.get_parameter('kalman_acceleration_std_m_s2').value),
            float(self.get_parameter('kalman_reset_gap_s').value),
        )
        all_ids = set(reference_ids) | set(moving_flat)
        self._filters = {
            marker_id: PositionKalman3D(*filter_arguments)
            for marker_id in all_ids
        }
        self._source = RealSenseSource(
            self._serial, self._width, self._height, self._fps
        )
        self._serial = self._source.serial
        self._image_publisher = self.create_publisher(
            CompressedImage, 'aruco/annotated/compressed',
            qos_profile_sensor_data
        )
        self._status_publisher = self.create_publisher(
            String, 'aruco/status', 10
        )
        self._frame_index = 0
        self._last_status_frame = -1000
        self._empty_reads = 0
        self._base_rotation = None
        self._reference_origins = {}
        self._reference_samples = {
            marker_id: [] for marker_id in reference_ids
        }
        self._timer = self.create_timer(1.0 / self._fps, self._process_frame)
        self.get_logger().info(
            f'RGB-D ArUco tracker active on RealSense {self._serial} at '
            f'{self._width}x{self._height}@{self._fps}'
        )

    def _strings(self, parameter: str) -> list[str]:
        return [str(value) for value in self.get_parameter(parameter).value]

    def _integers(self, parameter: str) -> list[int]:
        return [int(value) for value in self.get_parameter(parameter).value]

    @staticmethod
    def _largest_detection_by_id(corners, ids) -> dict[int, np.ndarray]:
        selected: dict[int, np.ndarray] = {}
        if ids is None:
            return selected
        for marker_corners, marker_id_value in zip(corners, ids.flatten()):
            marker_id = int(marker_id_value)
            candidate = np.asarray(
                marker_corners, dtype=np.float32
            ).reshape(4, 2)
            previous = selected.get(marker_id)
            if previous is None or quadrilateral_area(
                candidate
            ) > quadrilateral_area(previous):
                selected[marker_id] = candidate
        return selected

    def _measure_points(self, detections, depth_m, intrinsics, time_s):
        raw_points = {}
        filtered_points = {}
        sample_counts = {}
        for marker_id, corners in detections.items():
            try:
                depth, samples = robust_marker_depth(
                    depth_m,
                    corners,
                    inner_fraction=self._depth_inner_fraction,
                    minimum_samples=self._minimum_depth_samples,
                    minimum_depth_m=self._minimum_depth,
                    maximum_depth_m=self._maximum_depth,
                )
                center = corners.mean(axis=0)
                point = self._source.deproject(intrinsics, center, depth)
            except ValueError:
                continue
            raw_points[marker_id] = point
            sample_counts[marker_id] = samples
            filtered_points[marker_id] = (
                self._filters[marker_id].update(point, time_s)
                if self._enable_kalman and marker_id in self._filters
                else point
            )
        return raw_points, filtered_points, sample_counts

    @staticmethod
    def _point_cloud(stamp, frame_id, points, marker_ids, channels=()):
        message = PointCloud()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.points = [
            Point32(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            for point in points
        ]
        id_channel = ChannelFloat32()
        id_channel.name = 'marker_id'
        id_channel.values = [float(value) for value in marker_ids]
        message.channels = [id_channel]
        for name, values in channels:
            channel = ChannelFloat32()
            channel.name = name
            channel.values = [float(value) for value in values]
            message.channels.append(channel)
        return message

    def _publish_arm(
        self, arm, raw_points, filtered_points, sample_counts,
        base_origin, base_rotation, stamp
    ) -> bool:
        moving_ids = arm['moving_ids']
        if any(marker_id not in filtered_points for marker_id in moving_ids):
            return False
        try:
            local_points = project_to_base_frame(
                [filtered_points[value] for value in moving_ids],
                base_origin,
                base_rotation,
            )
        except (ValueError, np.linalg.LinAlgError):
            return False

        arm['state_publisher'].publish(self._point_cloud(
            stamp,
            f"aruco_base_{arm['reference_id']}",
            local_points,
            moving_ids,
            channels=(
                ('depth_samples', [sample_counts[value] for value in moving_ids]),
                ('raw_camera_z_m', [raw_points[value][2] for value in moving_ids]),
            ),
        ))
        arm['raw_publisher'].publish(self._point_cloud(
            stamp,
            f'camera_{self._serial}_optical_frame',
            [raw_points[value] for value in moving_ids],
            moving_ids,
            channels=(
                ('depth_samples', [sample_counts[value] for value in moving_ids]),
            ),
        ))
        return True

    def _publish_annotated(
        self, frame, corners, ids, raw_points, complete, stamp
    ) -> None:
        if (
            not self._publish_image
            or self._image_publisher.get_subscription_count() == 0
        ):
            return
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for marker_id, point in raw_points.items():
            detection_index = None
            if ids is not None:
                matches = np.flatnonzero(ids.flatten() == marker_id)
                if matches.size:
                    detection_index = int(matches[0])
            if detection_index is not None:
                position = tuple(np.rint(
                    corners[detection_index].reshape(4, 2).mean(axis=0)
                ).astype(int))
                cv2.putText(
                    frame, f'{point[2]:.3f}m', position,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1,
                    cv2.LINE_AA,
                )
        cv2.putText(
            frame,
            f'{self._serial} | frame {self._frame_index} | {",".join(complete)}',
            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            cv2.LINE_AA,
        )
        success, encoded = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
        )
        if success:
            message = CompressedImage()
            message.header.stamp = stamp
            message.header.frame_id = f'camera_{self._serial}_optical_frame'
            message.format = 'jpeg'
            message.data = encoded.tobytes()
            self._image_publisher.publish(message)

    def _process_frame(self) -> None:
        acquired = self._source.read()
        if acquired is None:
            self._empty_reads += 1
            return
        frame, depth_m, intrinsics = acquired
        self._frame_index += 1
        now = self.get_clock().now()
        stamp = now.to_msg()
        corners, ids, _ = self._detector.detectMarkers(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        )
        detections = self._largest_detection_by_id(corners, ids)
        raw_points, filtered_points, sample_counts = self._measure_points(
            detections, depth_m, intrinsics, now.nanoseconds * 1e-9
        )
        complete = []
        reference_ids = [arm['reference_id'] for arm in self._arms]
        reference_valid = {
            marker_id: (
                marker_id in filtered_points
                and marker_id in detections
                and quadrilateral_area(detections[marker_id])
                >= self._minimum_reference_area
            )
            for marker_id in reference_ids
        }
        references_valid = all(reference_valid.values())
        if references_valid and len(reference_ids) >= 2:
            if self._freeze_references and self._base_rotation is None:
                for marker_id in reference_ids:
                    self._reference_samples[marker_id].append(
                        filtered_points[marker_id].copy()
                    )
                sample_count = min(
                    len(values) for values in self._reference_samples.values()
                )
                if sample_count >= self._reference_lock_frames:
                    try:
                        origins, rotation = frozen_reference_frame(
                            self._reference_samples,
                            reference_ids,
                            minimum_baseline_m=self._minimum_baseline,
                        )
                        self._reference_origins = origins
                        self._base_rotation = rotation
                        self.get_logger().info(
                            'reference frame locked from '
                            f'{sample_count} RGB-D samples'
                        )
                    except ValueError:
                        for values in self._reference_samples.values():
                            values.clear()
            elif not self._freeze_references:
                try:
                    measured_rotation = robot_rotation_from_bases(
                        filtered_points[reference_ids[0]],
                        filtered_points[reference_ids[1]],
                        minimum_baseline_m=self._minimum_baseline,
                    )
                    self._base_rotation = blend_rotation(
                        self._base_rotation,
                        measured_rotation,
                        self._rotation_alpha,
                    )
                    self._reference_origins = {
                        value: filtered_points[value].copy()
                        for value in reference_ids
                    }
                except ValueError:
                    pass
        elif self._freeze_references and self._base_rotation is None:
            for values in self._reference_samples.values():
                values.clear()
        if self._base_rotation is not None:
            for arm in self._arms:
                reference_id = arm['reference_id']
                base_origin = self._reference_origins.get(reference_id)
                if base_origin is None:
                    continue
                if self._publish_arm(
                    arm, raw_points, filtered_points, sample_counts,
                    base_origin, self._base_rotation, stamp
                ):
                    complete.append(arm['name'])
        self._publish_annotated(
            frame, corners, ids, raw_points, complete, stamp
        )

        if self._frame_index - self._last_status_frame >= self._fps:
            status = String()
            status.data = json.dumps({
                'frame': self._frame_index,
                'camera_serial': self._serial,
                'detected_ids': sorted(detections),
                'depth_valid_ids': sorted(raw_points),
                'valid_reference_ids': sorted(
                    marker_id
                    for marker_id, valid in reference_valid.items()
                    if valid
                ),
                'base_rotation_ready': self._base_rotation is not None,
                'reference_frame_locked': (
                    self._freeze_references and self._base_rotation is not None
                ),
                'cached_reference_ids': sorted(self._reference_origins),
                'reference_lock_samples': min(
                    (len(values) for values in self._reference_samples.values()),
                    default=0,
                ),
                'complete_arms': complete,
                'coordinate_frame': 'metric_robot_base_3d',
                'kalman_enabled': self._enable_kalman,
                'empty_poll_count': self._empty_reads,
            }, separators=(',', ':'))
            self._status_publisher.publish(status)
            self._last_status_frame = self._frame_index

    def destroy_node(self):
        if hasattr(self, '_source'):
            self._source.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoShapeTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
