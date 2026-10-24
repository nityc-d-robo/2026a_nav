"""ROS adapter. USB CDC decoding belongs in a separate input adapter."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from .kinematics import OmniOdometry


def covariance(values):
    result = [0.0] * 36
    for index, value in zip((0, 7, 14, 21, 28, 35), values):
        result[index] = value
    return result


class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')
        self.declare_parameter('wheel_names', Parameter.Type.STRING_ARRAY)
        for name in ('wheel_x', 'wheel_y', 'wheel_drive_angles', 'wheel_radii', 'wheel_signs'):
            self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('input_topic', '/wheel_states')
        self.declare_parameter('input_mode', 'velocity')
        self.declare_parameter('max_sample_interval', 0.2)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        # Illustrative variances only: calibrate with measured errors before fusion.
        self.declare_parameter('pose_covariance_diagonal', [0.01, 0.01, 1e6, 1e6, 1e6, 0.02])
        self.declare_parameter('twist_covariance_diagonal', [0.01, 0.01, 1e6, 1e6, 1e6, 0.02])

        def get(name):
            return self.get_parameter(name).value

        self.names = get('wheel_names')
        if not self.names or len(set(self.names)) != len(self.names) or any(not n for n in self.names):
            raise ValueError('Provide nonempty, unique wheel_names and all wheel geometry parameters')
        geometry = [get(n) for n in ('wheel_x', 'wheel_y', 'wheel_drive_angles', 'wheel_radii', 'wheel_signs')]
        if any(a is None or len(a) != len(self.names) for a in geometry):
            raise ValueError('All wheel geometry arrays must have one value per wheel_names entry')
        self.core = OmniOdometry(*geometry)
        self.input_mode = get('input_mode')
        self.max_interval = get('max_sample_interval')
        if self.input_mode not in ('velocity', 'position'):
            raise ValueError('input_mode must be velocity or position')
        if not math.isfinite(self.max_interval) or self.max_interval <= 0:
            raise ValueError('max_sample_interval must be positive')
        self.odom_frame, self.base_frame = get('odom_frame'), get('base_frame')
        if (not self.odom_frame or not self.base_frame or self.odom_frame == self.base_frame
                or self.odom_frame.startswith('/') or self.base_frame.startswith('/')):
            raise ValueError('Use distinct nonempty frame names without leading slashes')
        self.pose_cov = self._covariance(get('pose_covariance_diagonal'))
        self.twist_cov = self._covariance(get('twist_covariance_diagonal'))
        self.publisher = self.create_publisher(Odometry, get('odom_topic'), 10)
        self.broadcaster = TransformBroadcaster(self) if get('publish_tf') else None
        self.subscription = self.create_subscription(
            JointState, get('input_topic'), self.on_sample, qos_profile_sensor_data)
        self.get_logger().info('Encoder odometry ready; output follows measurement timestamps/rate')

    @staticmethod
    def _covariance(values):JointState.velocity
        if len(values) != 6 or any(not math.isfinite(v) or v < 0 for v in values):
            raise ValueError('Covariance diagonal must contain 6 finite nonnegative variances')
        return covariance(values)

    def on_sample(self, msg):
        try:
            values = msg.velocity if self.input_mode == 'velocity' else msg.position
            if len(msg.name) != len(values) or len(set(msg.name)) != len(msg.name):
                raise ValueError('JointState needs unique names and matching input values')
            wheel_values = dict(zip(msg.name, values))
            ordered = [wheel_values[name] for name in self.names]
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if self.input_mode == 'velocity':
                previous = self.core.velocity_stamp
                if previous is not None and (stamp_ns - previous) * 1e-9 > self.max_interval:
                    self.get_logger().warning(
                        'Encoder data gap: skipping displacement over the missing interval',
                        throttle_duration_sec=2.0)
                pose, velocity = self.core.update_velocity(ordered, stamp_ns, self.max_interval)
            else:
                pose, velocity = self.core.update(ordered, stamp_ns)
        except (ValueError, KeyError) as error:
            self.get_logger().warning(f'Ignoring encoder sample: {error}', throttle_duration_sec=2.0)
            return
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(pose[0])
        odom.pose.pose.position.y = float(pose[1])
        odom.pose.pose.orientation.z = math.sin(pose[2] / 2)
        odom.pose.pose.orientation.w = math.cos(pose[2] / 2)
        odom.pose.covariance = self.pose_cov
        odom.twist.twist.linear.x = float(velocity[0])
        odom.twist.twist.linear.y = float(velocity[1])
        odom.twist.twist.angular.z = float(velocity[2])
        odom.twist.covariance = self.twist_cov
        self.publisher.publish(odom)
        if self.broadcaster:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EncoderOdometry()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
