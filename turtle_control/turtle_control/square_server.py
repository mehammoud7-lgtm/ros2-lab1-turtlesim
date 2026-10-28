import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from turtle_interfaces.action import DrawSquare, GoToPose
from turtle_interfaces.msg import TargetPose

CX = CY = 5.544445       # start position (window centre)
DEFAULT_SIDE = 3.0
MAX_SIDE = 5.0           # keeps CX + side < 11


class SquareServer(Node):
    def __init__(self):
        super().__init__('square_server')
        self.cbg = ReentrantCallbackGroup()
        self.reset_cli = self.create_client(Trigger, 'reset_turtle', callback_group=self.cbg)
        self.goto_cli = ActionClient(self, GoToPose, 'go_to_pose', callback_group=self.cbg)
        self.active = False
        self.server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cbg)
        self.get_logger().info('square server ready')

    def _goal_cb(self, goal):
        s = goal.side_length
        if s > MAX_SIDE or self.active:
            self.get_logger().warn('Square goal rejected (too big or busy)')
            return GoalResponse.REJECT
        self.active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _wait(fut):
        while rclpy.ok() and not fut.done():
            time.sleep(0.02)
        return fut.result()

    def _goto(self, x, y, yaw, goal_handle):
        """One blocking goToPose call. Returns True on success."""
        goal = GoToPose.Goal()
        goal.target = TargetPose(x=float(x), y=float(y), yaw=float(yaw))
        gh = self._wait(self.goto_cli.send_goal_async(goal))
        if not gh.accepted:
            return False
        res_fut = gh.get_result_async()
        while rclpy.ok() and not res_fut.done():
            if goal_handle.is_cancel_requested:
                gh.cancel_goal_async()
            time.sleep(0.02)
        return res_fut.result().result.success

    def _execute(self, goal_handle):
        side = goal_handle.request.side_length
        side = DEFAULT_SIDE if side <= 0.0 else side
        result, fb = DrawSquare.Result(), DrawSquare.Feedback()
        done = 0
        try:
            if not self.reset_cli.wait_for_service(timeout_sec=5.0) or \
               not self.goto_cli.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('reset service or goToPose server missing')
                goal_handle.abort()
                result.success = False
                return result
            self._wait(self.reset_cli.call_async(Trigger.Request()))   # clean start
            # Start at (CX,CY) facing 0. Corners visited counter-clockwise.
            corners = [
                (CX + side, CY,        math.pi / 2),
                (CX + side, CY + side, math.pi),
                (CX,        CY + side, -math.pi / 2),
                (CX,        CY,        0.0),
            ]
            for i, (x, y, yaw) in enumerate(corners, start=1):
                fb.current_side = i
                goal_handle.publish_feedback(fb)
                self.get_logger().info(f'Side {i}/4 -> ({x:.2f}, {y:.2f})')
                if goal_handle.is_cancel_requested or not self._goto(x, y, yaw, goal_handle):
                    break
                done += 1
        finally:
            self.active = False
        result.sides_completed = done
        result.success = done == 4
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = SquareServer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
