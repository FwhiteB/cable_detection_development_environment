import math
import random
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float32

from mission_logic.models import MoveResult, ReceiverReading, RobotPose


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Robot:
    def __init__(
        self,
        node,
        goal_pose_publisher,
        speed_publisher,
        goal_frame='map',
        arrival_tolerance=0.3,
        speed=1.0,
    ):
        self._node = node
        self._goal_pose_publisher = goal_pose_publisher
        self._speed_publisher = speed_publisher
        self._goal_frame = goal_frame
        self._arrival_tolerance = arrival_tolerance
        self._speed = speed

        self.pose: Optional[RobotPose] = None
        self.reading: Optional[ReceiverReading] = None
        self.active_goal: Optional[RobotPose] = None

    def update_pose(self, odometry_msg):
        orientation = odometry_msg.pose.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        position = odometry_msg.pose.pose.position
        self.pose = RobotPose(
            x=position.x,
            y=position.y,
            z=position.z,
            yaw=yaw,
        )

    def update_reading(self, magnetic_field_msg):
        field = magnetic_field_msg.magnetic_field
        stamp = magnetic_field_msg.header.stamp
        self.reading = ReceiverReading(
            magnetic_x=field.x,
            magnetic_y=field.y,
            magnetic_z=field.z,
            stamp_sec=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            frame_id=magnetic_field_msg.header.frame_id,
        )

    def read(self):
        return self.pose, self.reading

    def move_to(self, x, y, yaw):
        if self.pose is None:
            return None

        self.active_goal = RobotPose(
            x=x,
            y=y,
            z=self.pose.z,
            yaw=yaw,
        )
        self._publish_speed()
        self.publish_active_goal()
        return MoveResult(target=self.active_goal, reading=self.reading)

    def publish_active_goal(self):
        if self.active_goal is None:
            return

        orientation_x, orientation_y, orientation_z, orientation_w = yaw_to_quaternion(
            self.active_goal.yaw
        )

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose_msg.header.frame_id = self._goal_frame
        goal_pose_msg.pose.position.x = self.active_goal.x
        goal_pose_msg.pose.position.y = self.active_goal.y
        goal_pose_msg.pose.position.z = self.active_goal.z
        goal_pose_msg.pose.orientation.x = orientation_x
        goal_pose_msg.pose.orientation.y = orientation_y
        goal_pose_msg.pose.orientation.z = orientation_z
        goal_pose_msg.pose.orientation.w = orientation_w

        self._goal_pose_publisher.publish(goal_pose_msg)

    def has_arrived(self):
        if self.pose is None or self.active_goal is None:
            return False

        dx = self.pose.x - self.active_goal.x
        dy = self.pose.y - self.active_goal.y
        return math.hypot(dx, dy) <= self._arrival_tolerance

    def _publish_speed(self):
        speed_msg = Float32()
        speed_msg.data = float(self._speed)
        self._speed_publisher.publish(speed_msg)


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('step_x', 1.0)
        self.declare_parameter('step_y', 1.0)
        self.declare_parameter('arrival_tolerance', 0.3)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('goal_republish_period', 1.0)
        self.declare_parameter('max_steps', 100)

        self.step_x = self.get_parameter('step_x').value
        self.step_y = self.get_parameter('step_y').value
        self.max_steps = self.get_parameter('max_steps').value
        goal_republish_period = self.get_parameter('goal_republish_period').value

        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.speed_publisher = self.create_publisher(Float32, '/speed', 10)

        self.robot = Robot(
            node=self,
            goal_pose_publisher=self.goal_pose_publisher,
            speed_publisher=self.speed_publisher,
            goal_frame=self.get_parameter('goal_frame').value,
            arrival_tolerance=self.get_parameter('arrival_tolerance').value,
            speed=self.get_parameter('speed').value,
        )
        self.step_count = 0

        self.state_subscription = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.state_estimation_callback,
            10,
        )
        self.magnetic_field_subscription = self.create_subscription(
            MagneticField,
            '/magnetic_field',
            self.magnetic_field_callback,
            10,
        )
        self.goal_timer = self.create_timer(
            goal_republish_period,
            self.robot.publish_active_goal,
        )

    def state_estimation_callback(self, odometry_msg):
        self.robot.update_pose(odometry_msg)

        if self.robot.active_goal is None:
            self._issue_next_move()
            return

        if self.robot.has_arrived():
            pose, reading = self.robot.read()
            self.get_logger().info(
                'Reached step %d at x=%.2f y=%.2f. Reading available: %s'
                % (self.step_count, pose.x, pose.y, reading is not None)
            )
            self._issue_next_move()

    def magnetic_field_callback(self, magnetic_field_msg):
        self.robot.update_reading(magnetic_field_msg)

    def _issue_next_move(self):
        if self.robot.pose is None:
            return

        if self.step_count >= self.max_steps:
            self.get_logger().info('Mission reached max_steps=%d.' % self.max_steps)
            return

        base_pose = self.robot.pose
        next_x = base_pose.x + self.step_x
        next_y = base_pose.y + self.step_y
        random_yaw = random.uniform(-math.pi, math.pi)

        move_result = self.robot.move_to(next_x, next_y, random_yaw)
        self.step_count += 1

        self.get_logger().info(
            'Step %d target: x=%.2f y=%.2f yaw=%.2f rad'
            % (
                self.step_count,
                move_result.target.x,
                move_result.target.y,
                move_result.target.yaw,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
