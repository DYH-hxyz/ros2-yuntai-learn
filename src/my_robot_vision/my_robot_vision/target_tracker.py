import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes


class TargetTracker(Node):

    def __init__(self):
        super().__init__('target_tracker')

        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('horizontal_fov', 1.047)
        self.declare_parameter('pan_sign', -1.0)
        self.declare_parameter('tilt_sign', -1.0)
        self.declare_parameter('gain', 1.0)
        self.declare_parameter('deadband', 15.0)
        self.declare_parameter('max_step', 0.35)
        self.declare_parameter('min_interval', 0.3)
        self.declare_parameter('retry_cooldown', 2.0)
        self.declare_parameter('move_threshold', 0.05)
        self.declare_parameter('lost_timeout', 1.5)
        self.declare_parameter('goal_timeout', 10.0)
        self.declare_parameter('joint1_lower', -3.14)
        self.declare_parameter('joint1_upper', 3.14)
        self.declare_parameter('joint2_lower', 0.0)
        self.declare_parameter('joint2_upper', 3.14)

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fx = (self.image_width / 2.0) / math.tan(self.get_parameter('horizontal_fov').value / 2.0)
        self.fy = self.fx
        self.pan_sign = self.get_parameter('pan_sign').value
        self.tilt_sign = self.get_parameter('tilt_sign').value
        self.gain = self.get_parameter('gain').value
        self.deadband = self.get_parameter('deadband').value
        self.max_step = self.get_parameter('max_step').value
        self.min_interval = self.get_parameter('min_interval').value
        self.retry_cooldown = self.get_parameter('retry_cooldown').value
        self.move_threshold = self.get_parameter('move_threshold').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.joint1_lower = self.get_parameter('joint1_lower').value
        self.joint1_upper = self.get_parameter('joint1_upper').value
        self.joint2_lower = self.get_parameter('joint2_lower').value
        self.joint2_upper = self.get_parameter('joint2_upper').value

        self.planning_group = 'yuntai_planning_groups'
        self.joint_names = ['yuntai_joint1', 'yuntai_joint2']
        self.joint_positions = None
        self.joint_velocities = None
        self.busy = False
        self.cooldown = 0.0
        self.last_send_time = 0.0
        self.last_seen = time.monotonic()

        self.scan_poses = []
        for tilt in (1.5708, 1.1, 2.0):
            for offset in (0.6, -0.6, 1.2, -1.2, 1.8, -1.8, 2.4, -2.4, 3.0, -3.0):
                self.scan_poses.append((offset, tilt))
        self.scan_index = 0
        self.scan_base = None

        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Detection2DArray, '/target/detections', self.detection_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('waiting for /move_action ...')
        self.move_group.wait_for_server()
        self.get_logger().info('target tracker ready')

    def joint_state_callback(self, msg):
        positions = {}
        for name, position in zip(msg.name, msg.position):
            positions[name] = position
        if all(name in positions for name in self.joint_names):
            self.joint_positions = [positions[self.joint_names[0]], positions[self.joint_names[1]]]
        velocities = {}
        for name, velocity in zip(msg.name, msg.velocity):
            velocities[name] = velocity
        if all(name in velocities for name in self.joint_names):
            self.joint_velocities = [velocities[self.joint_names[0]], velocities[self.joint_names[1]]]

    def is_moving(self):
        if not self.joint_velocities:
            return False
        return max(abs(v) for v in self.joint_velocities) > self.move_threshold

    def not_ready(self):
        if self.busy or not self.joint_positions or self.is_moving():
            return True
        return time.monotonic() - self.last_send_time < self.cooldown

    def nearest_angle(self, value, current, lower, upper):
        candidates = [value + turns * 2.0 * math.pi for turns in (0, 1, -1)]
        candidates = [c for c in candidates if lower <= c <= upper]
        if not candidates:
            return max(lower, min(upper, value))
        return min(candidates, key=lambda c: abs(c - current))

    def detection_callback(self, msg):
        if not msg.detections:
            return
        self.last_seen = time.monotonic()
        if self.not_ready():
            return

        bbox = msg.detections[0].bbox
        error_x = bbox.center.position.x - self.image_width / 2.0
        error_y = bbox.center.position.y - self.image_height / 2.0
        if abs(error_x) < self.deadband and abs(error_y) < self.deadband:
            return

        step1 = self.pan_sign * self.gain * math.atan(error_x / self.fx)
        step2 = self.tilt_sign * self.gain * math.atan(error_y / self.fy)
        step1 = max(-self.max_step, min(self.max_step, step1))
        step2 = max(-self.max_step, min(self.max_step, step2))

        target1 = self.nearest_angle(self.joint_positions[0] + step1, self.joint_positions[0], self.joint1_lower, self.joint1_upper)
        target2 = self.nearest_angle(self.joint_positions[1] + step2, self.joint_positions[1], self.joint2_lower, self.joint2_upper)

        if abs(target1 - self.joint_positions[0]) < 0.01 and abs(target2 - self.joint_positions[1]) < 0.01:
            return

        self.move_to(target1, target2, f'track x={error_x:.0f} y={error_y:.0f}')

    def timer_callback(self):
        if self.busy:
            if time.monotonic() - self.last_send_time > self.goal_timeout:
                self.get_logger().warn('move_action timeout, resume')
                self.busy = False
            else:
                return
        if not self.joint_positions:
            return
        if time.monotonic() - self.last_seen < self.lost_timeout:
            self.scan_base = None
            self.scan_index = 0
            return
        if self.not_ready():
            return
        if self.scan_base is None:
            self.scan_base = self.joint_positions[0]
            self.scan_index = 0

        offset, tilt = self.scan_poses[self.scan_index % len(self.scan_poses)]
        self.scan_index += 1
        pan = self.nearest_angle(self.scan_base + offset, self.joint_positions[0], self.joint1_lower, self.joint1_upper)
        self.move_to(pan, tilt, f'search step {self.scan_index}')

    def move_to(self, target1, target2, tag):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 1.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        constraints = Constraints()
        for name, position in zip(self.joint_names, (target1, target2)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = 0.02
            joint_constraint.tolerance_below = 0.02
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        goal.request.goal_constraints.append(constraints)

        self.last_send_time = time.monotonic()
        self.cooldown = self.min_interval
        self.busy = True
        self.get_logger().info(f'{tag} -> joint1={target1:.2f} joint2={target2:.2f}')
        self.move_group.send_goal_async(goal).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.last_send_time = time.monotonic()
            self.cooldown = self.retry_cooldown
            self.busy = False
            return
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        error_code = future.result().result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f'move failed ({error_code}), wait before retry')
            self.last_send_time = time.monotonic()
            self.cooldown = self.retry_cooldown
        self.busy = False


def main():
    rclpy.init()
    node = TargetTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
