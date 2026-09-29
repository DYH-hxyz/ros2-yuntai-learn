#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes

class YuntaiMoveitNode(Node):
    def __init__(self):
        super().__init__("yuntai_moveit_ctrl")
        # ========= 这里改你的参数 =========
        self.PLANNING_GROUP = "yuntai_planning_groups"  # 你在MoveIt Setup里定义的规划组名字
        self.JOINT_NAMES = ["yuntai_joint1", "yuntai_joint2"] # 和URDF、SRDF完全一致
        # =================================
        # demo.launch.py 启动的 move_group 对外提供 /move_action 动作接口
        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        self.get_logger().info("等待 move_group 的 /move_action ...")
        self.move_group.wait_for_server()
        self.get_logger().info("MoveGroup 已就绪，可以发送目标关节角度")

    def go_to_joint_target(self, j1_rad: float, j2_rad: float):
        # 组装目标：给要动的关节各加一个位置约束
        goal = MoveGroup.Goal()
        goal.request.group_name = self.PLANNING_GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1

        constraints = Constraints()
        for joint_name, position in zip(self.JOINT_NAMES, (j1_rad, j2_rad)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        goal.request.goal_constraints.append(constraints)

        # 只想看规划、不让云台动时，把下面这行改成 True
        goal.planning_options.plan_only = False

        # 规划 + 执行
        send_future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("目标被 move_group 拒绝！")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        error_code = result_future.result().result.error_code
        if error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"执行成功，目标 joint1={j1_rad}, joint2={j2_rad}")
        else:
            self.get_logger().error(f"运动规划/执行失败！错误码 {error_code.val}：{error_code.message}")

def main():
    rclpy.init()
    node = YuntaiMoveitNode()
    try:
        # ========= 在这里修改目标角度(rad) =========
        # 关节限位：joint1 ∈ [-1.57, 1.57]，joint2 ∈ [0, 3.14]
        # 注意：角度必须写成小数，3.0 不能写成 3（msg 的 float64 字段收到 int 会直接 abort）
        node.go_to_joint_target(-1.57, 3.14)
        # 你可以多次调用，比如：
        # node.go_to_joint_target(0.0, 0.0)
        # =========================================
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
