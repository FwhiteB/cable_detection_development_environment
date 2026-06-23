import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from mission_logic_msgs.msg import SensorMsg
from std_msgs.msg import Float32
from mission_logica import Point3D

from mission_logic.models import MissionLogEntry, MissionState, MoveResult, ReceiverReading, RobotPose


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
        arrival_tolerance_distance = 0.2,
        arrival_tolerance_yaw_rad = math.radians(10.0),
        speed=1.0,
    ):
        self._node = node
        self._goal_pose_publisher = goal_pose_publisher
        self._speed_publisher = speed_publisher
        self._goal_frame = goal_frame
        self._arrival_tolerance_distance = arrival_tolerance_distance
        self._arrival_tolerance_yaw_rad = arrival_tolerance_yaw_rad
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

        stamp = magnetic_field_msg.header.stamp
        self.reading = ReceiverReading(
            signal_strength = magnetic_field_msg.signal_strength,
            depth = magnetic_field_msg.depth_meters,
            current = magnetic_field_msg.current_milliamps,
            pipeline_heading_degrees = magnetic_field_msg.pipeline_heading_degrees,
            signal_strength_percent = magnetic_field_msg.signal_strength_percent,
            left_arrow = magnetic_field_msg.left_arrow,  # notice: 指的是出现在左边的，指示机器向右的箭头，这表明机器本身在管线左边
            right_arrow = magnetic_field_msg.right_arrow,
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
        dyaw = self.pose.yaw - self.active_goal.yaw
        yaw_error = abs(math.atan2(math.sin(dyaw), math.cos(dyaw)))
        return (
            math.hypot(dx, dy) <= self._arrival_tolerance_distance
            and yaw_error <= self._arrival_tolerance_yaw_rad
        )

    def _publish_speed(self):
        speed_msg = Float32()
        speed_msg.data = float(self._speed)
        self._speed_publisher.publish(speed_msg)


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('step_x', 1.0)
        self.declare_parameter('step_y', 0.0)
        self.declare_parameter('magnetic_y_gain', 0.5)
        self.declare_parameter('max_lateral_step', 1.0)
        self.declare_parameter('workspace_min_x', -20.0)
        self.declare_parameter('workspace_max_x', 20.0)
        self.declare_parameter('workspace_min_y', -20.0)
        self.declare_parameter('workspace_max_y', 20.0)
        self.declare_parameter('detect_threshold', 0.2)
        self.declare_parameter('loss_threshold', 0.08)
        self.declare_parameter('center_magnetic_z_threshold', 0.05)
        self.declare_parameter('search_probe_distance', 0.75)
        self.declare_parameter('centering_step', 0.25)
        self.declare_parameter('forward_step', 1.0)
        self.declare_parameter('reacquire_probe_offset', 0.4)
        self.declare_parameter('follow_heading_degrees', 0.0)
        self.declare_parameter('orientation_check_interval', 3)
        self.declare_parameter('arrival_tolerance_distance', 0.2)
        self.declare_parameter('arrival_tolerance_yaw_degree', 10.0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('goal_republish_period', 1.0)
        self.declare_parameter('max_steps', 100)

        self.step_x = self.get_parameter('step_x').value
        self.step_y = self.get_parameter('step_y').value
        self.magnetic_y_gain = self.get_parameter('magnetic_y_gain').value
        self.max_lateral_step = self.get_parameter('max_lateral_step').value
        self.workspace_min_x = self.get_parameter('workspace_min_x').value
        self.workspace_max_x = self.get_parameter('workspace_max_x').value
        self.workspace_min_y = self.get_parameter('workspace_min_y').value
        self.workspace_max_y = self.get_parameter('workspace_max_y').value
        self.detect_threshold = self.get_parameter('detect_threshold').value
        self.loss_threshold = self.get_parameter('loss_threshold').value
        self.center_magnetic_z_threshold = self.get_parameter('center_magnetic_z_threshold').value  # notice: useless
        self.search_probe_distance = self.get_parameter('search_probe_distance').value
        self.centering_step = self.get_parameter('centering_step').value
        self.forward_step = self.get_parameter('forward_step').value
        self.reacquire_probe_offset = self.get_parameter('reacquire_probe_offset').value
        self.follow_heading_degrees = self.get_parameter('follow_heading_degrees').value  # useless
        self.orientation_check_interval = int(self.get_parameter('orientation_check_interval').value)
        self.max_steps = self.get_parameter('max_steps').value
        goal_republish_period = self.get_parameter('goal_republish_period').value

        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.speed_publisher = self.create_publisher(Float32, '/speed', 10)

        self.robot = Robot(
            node=self,
            goal_pose_publisher=self.goal_pose_publisher,
            speed_publisher=self.speed_publisher,
            goal_frame=self.get_parameter('goal_frame').value,
            arrival_tolerance_distance=self.get_parameter('arrival_tolerance_distance').value,
            arrival_tolerance_yaw_rad=math.radians(
                self.get_parameter('arrival_tolerance_yaw_degree').value
                ),
            speed=self.get_parameter('speed').value,
        )
        self.step_count = 0
        self.state = MissionState.SEARCH_PEAK
        self.follow_moves_since_center = 0
        self.line_confirmed = False
        self.log: list[MissionLogEntry] = []
        self.done = False
        self.phase = "normal"
        self.rotation_count = 0
        self.max_yaw = 0
        self.max_signal = float('-inf')

        self.state_subscription = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.state_estimation_callback,
            10,
        )
        self.magnetic_field_subscription = self.create_subscription(
            SensorMsg,
            '/magnetic_field',
            self.magnetic_field_callback,
            10,
        )
        self.goal_timer = self.create_timer(
            goal_republish_period,
            self.robot.publish_active_goal,
        )
        self.control_timer = self.create_timer(0.2, self._advance_state_machine)

    def state_estimation_callback(self, odometry_msg):
        self.robot.update_pose(odometry_msg)
        self._advance_state_machine()

    def magnetic_field_callback(self, magnetic_field_msg):
        self.robot.update_reading(magnetic_field_msg)
        self._advance_state_machine()

    def _advance_state_machine(self):
        if self.done or self.robot.pose is None or self.robot.reading is None:
            return

        # rotation_trial module
        if self.phase == "rotation_trial":
            if self.robot.active_goal is not None and not self.robot.has_arrived():
                return
            if self.robot.has_arrived():
                self.last_search_target_yaw = self.robot.active_goal.yaw
                self.robot.active_goal = None
                self.wait_reading_after_stamp = self.robot.reading.stamp_sec
                self.phase = "rotation_trial_wait_reading"
                return

        if self.phase == "rotation_trial_wait_reading":
            if self.robot.reading.stamp_sec <= self.wait_reading_after_stamp:
                return
            else:
                self.phase = "rotation_trial"
                self.rotation_count += 1
                if self.robot.reading.left_arrow or self.robot.reading.right_arrow:
                    self.phase = "normal"
                    self.rotation_count = 0
                    self.max_yaw = 0
                    self.max_signal = float('-inf')
                    self._transition(MissionState.CENTER_ON_LINE, 'detected magnetic signal during rotation trial')
                    return
                if self.max_signal < self.robot.reading.signal_strength:
                    self.max_yaw = self.last_search_target_yaw # 无噪声
                    self.max_signal = self.robot.reading.signal_strength
                if self.rotation_count < 12:
                    next_yaw = (self.search_base_yaw + self.rotation_count * math.radians(30)) % (2 * math.pi)
                    self.robot.move_to(self.robot.pose.x, self.robot.pose.y, next_yaw)
                    return
                else:
                    self.phase = "normal"
                    dx = math.cos(self.max_yaw) * self.search_probe_distance
                    dy = math.sin(self.max_yaw) * self.search_probe_distance
                    self._issue_move_by(dx, dy, self.max_yaw, 'search peak')
                    self.rotation_count = 0
                    self.max_yaw = 0
                    self.max_signal = float('-inf')
                    return
                return

        if self.robot.active_goal is not None:
            if not self.robot.has_arrived():
                return
            self._record_log('arrived at active goal')
            self.robot.active_goal = None

        if self.step_count >= self.max_steps:
            self._complete(MissionState.FAILED, 'Mission reached max_steps=%d.' % self.max_steps)
            return

        for _ in range(8):
            if self.done or self.robot.active_goal is not None:
                return
            if not self._tick_state_without_active_goal(): # quickly skip state switching
                return

    def _tick_state_without_active_goal(self):
        reading = self.robot.reading
        signal = reading.signal_strength
        

        if self.state == MissionState.SEARCH_PEAK:
            if reading.left_arrow or reading.right_arrow: # notice: should be converted to l-r arrows
                self._transition(MissionState.CENTER_ON_LINE, 'detected magnetic signal')
                return True
            self._issue_searching_move('search peak')
            return False

        if self.state == MissionState.CENTER_ON_LINE:
            if reading.left_arrow and reading.right_arrow: # notice: check
                self.line_confirmed = True # notice: whether to trust the arrows
                self.follow_moves_since_center = 0 # notice: seems not necessary
                self._transition(MissionState.MEASURE_ON_LINE, 'centered on magnetic line')
                return True
            elif (not reading.left_arrow) and (not reading.right_arrow):
                self._transition(MissionState.REACQUIRE, 'lost magnetic line')
                return True
            self._issue_lateral_move(self.centering_step, 'center on line')
            return False

        if self.state == MissionState.MEASURE_ON_LINE:
            self._record_log('measurement on line')
            self._transition(MissionState.FOLLOW_LINE, 'measurement complete')
            return True

        if self.state == MissionState.FOLLOW_LINE:
            if signal < self.loss_threshold: # notice: should be converted to l-r arrows
                self._transition(MissionState.REACQUIRE, 'magnetic signal lost')
                return True
            if self.follow_moves_since_center >= self.orientation_check_interval:
                self.follow_moves_since_center = 0
                self._transition(MissionState.CENTER_ON_LINE, 'periodic centering check')
                return True
            self._issue_forward_move('follow line')
            self.follow_moves_since_center += 1
            return False

        if self.state == MissionState.REACQUIRE:
            if signal >= self.detect_threshold: # notice: should be converted to l-r arrows
                self._transition(MissionState.CENTER_ON_LINE, 'reacquired magnetic signal')
                return True
            self._issue_lateral_move(self.reacquire_probe_offset, 'reacquire line')
            return False

        return False

    def _issue_searching_move(self, reason): # notice: todo
        if self.robot.reading.left_arrow or self.robot.reading.right_arrow:
            return
        self.phase = "rotation_trial"
        self.search_base_yaw = self.robot.pose.yaw
        self.rotation_count = 0
        self.max_yaw = self.robot.pose.yaw
        self.max_signal = float('-inf')
        next_yaw = (self.search_base_yaw + self.rotation_count * math.radians(30)) % (2 * math.pi)
        self.robot.move_to(self.robot.pose.x, self.robot.pose.y, next_yaw)
        return
        
        
        

    def _issue_forward_move(self, reason):
        heading_rad = math.radians(self.robot.reading.pipeline_heading_degrees) + self.robot.pose.yaw # notice: radian or degree
        heading_rad = heading_rad % (2 * math.pi)  
        dx = math.cos(heading_rad) * self.forward_step
        dy = math.sin(heading_rad) * self.forward_step
        self._issue_move_by(dx, dy, heading_rad, reason)

    def _issue_lateral_move(self, step_size, reason):
        reading = self.robot.reading
        if reading.left_arrow and reading.right_arrow:
            return # notice: 是否会有潜在的问题
        if not reading.left_arrow and not reading.right_arrow:
            return
        
        side = -1.0 if reading.left_arrow else 1.0
        yaw = self.robot.pose.yaw
        dx = -math.sin(yaw) * side * step_size
        dy = math.cos(yaw) * side * step_size

        self._issue_move_by(dx, dy, yaw, reason)

    def _issue_move_by(self, dx, dy, yaw, reason):
        pose = self.robot.pose
        target_x = self._clamp(pose.x + dx, self.workspace_min_x, self.workspace_max_x)
        target_y = self._clamp(pose.y + dy, self.workspace_min_y, self.workspace_max_y)

        if math.hypot(target_x - pose.x, target_y - pose.y) <= 1e-9:
            if self.line_confirmed:
                self._complete(MissionState.COMPLETE, 'workspace boundary reached')
            else:
                self._transition(MissionState.REACQUIRE, 'no available motion in workspace')
            return

        move_result = self.robot.move_to(target_x, target_y, yaw)
        self.step_count += 1
        self.get_logger().info(
            'Step %d state=%s target: x=%.2f y=%.2f yaw=%.2f reason=%s signal=%.3f'
            % (
                self.step_count,
                self.state.value,
                move_result.target.x,
                move_result.target.y,
                move_result.target.yaw,
                reason,
                self.robot.reading.signal_strength,
            )
        )

    def _transition(self, next_state, reason):
        if self.state == next_state:
            return
        self.get_logger().info('%s -> %s: %s' % (self.state.value, next_state.value, reason))
        self.state = next_state

    def _record_log(self, note):
        if self.robot.pose is None or self.robot.reading is None:
            return
        self.log.append(
            MissionLogEntry(
                step=self.step_count,
                state=self.state,
                pose=self.robot.pose,
                reading=self.robot.reading,
                note=note,
            )
        )
        self.get_logger().info(
            'Log state=%s pose=(%.2f, %.2f) signal=%.3f note=%s'
            % (
                self.state.value,
                self.robot.pose.x,
                self.robot.pose.y,
                self.robot.reading.signal_strength,
                note,
            )
        )

    def _complete(self, final_state, reason):
        self.done = True
        self.state = final_state
        self._record_log(reason)
        self.get_logger().info('Mission finished with state=%s: %s' % (final_state.value, reason))

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))


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
