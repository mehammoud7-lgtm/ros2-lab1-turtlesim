import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose

from turtle_interfaces.action import GoToPose
from turtle_interfaces.srv import SetSpeedLimits
from turtle_control.controller import Phase, PoseController

LIN = 'max_linear_speed'
ANG = 'max_angular_speed'
TIMEOUT_S = 60.0


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('goto_pose_server')
        self.cbg = ReentrantCallbackGroup()
        self.declare_parameter(LIN, 2.0)
        self.declare_parameter(ANG, 3.0)
        self.add_on_set_parameters_callback(self._on_params)

        self.pose = None
        self.active = False
        self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10,
                                 callback_group=self.cbg)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_service(SetSpeedLimits, 'set_speed_limits', self._speed_srv,
                            callback_group=self.cbg)
        self.server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cbg)
        self.get_logger().info('goToPose server ready')

    # ---- parameters ----
    def _on_params(self, params):
        for p in params:
            if p.name in (LIN, ANG):
                if p.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER) or p.value <= 0.0:
                    return SetParametersResult(successful=False,
                                               reason=f'{p.name} must be a number > 0')
        return SetParametersResult(successful=True)

    def _speed_srv(self, req, res):
        if req.max_linear_speed <= 0.0 or req.max_angular_speed <= 0.0:
            res.success = False
            res.message = 'Both speeds must be > 0; nothing changed.'
            return res
        results = self.set_parameters([
            Parameter(LIN, Parameter.Type.DOUBLE, float(req.max_linear_speed)),
            Parameter(ANG, Parameter.Type.DOUBLE, float(req.max_angular_speed)),
        ])
        res.success = all(r.successful for r in results)
        res.message = ('Speed limits updated.' if res.success else
                       'Rejected: ' + '; '.join(r.reason for r in results if not r.successful))
        return res

    # ---- action ----
    def _pose_cb(self, msg):
        self.pose = msg

    def _goal_cb(self, goal):
        t = goal.target
        if not (0.0 <= t.x <= 11.0 and 0.0 <= t.y <= 11.0):
            self.get_logger().warn('Goal rejected: outside window (0..11)')
            return GoalResponse.REJECT
        if self.active:
            self.get_logger().warn('Goal rejected: another goal is running')
            return GoalResponse.REJECT
        self.active = True
        return GoalResponse.ACCEPT

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _execute(self, goal_handle):
        t = goal_handle.request.target
        self.get_logger().info(f'New goal: x={t.x:.2f} y={t.y:.2f} yaw={t.yaw:.2f}')
        ctrl = PoseController(t.x, t.y, t.yaw)
        fb, result = GoToPose.Feedback(), GoToPose.Result()
        start, i = time.time(), 0
        outcome = 'succeed'
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    outcome = 'canceled'
                    break
                if time.time() - start > TIMEOUT_S:
                    outcome = 'abort'
                    break
                p = self.pose
                if p is None:
                    time.sleep(0.02)
                    continue
                # read live every cycle so parameter changes take effect immediately
                max_lin = self.get_parameter(LIN).value
                max_ang = self.get_parameter(ANG).value
                lin, ang = ctrl.compute(p.x, p.y, p.theta, max_lin, max_ang)
                if ctrl.phase is Phase.DONE:
                    break
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = float(lin), float(ang)
                self.cmd_pub.publish(cmd)
                if i % 5 == 0:
                    fb.distance_remaining = float(ctrl.dist)
                    fb.yaw_error = float(ctrl.yaw_err)
                    fb.phase = ctrl.phase.value
                    goal_handle.publish_feedback(fb)
                i += 1
                time.sleep(0.02)
        finally:
            self._stop()
            self.active = False
        result.success = outcome == 'succeed'
        result.final_distance_error = float(ctrl.dist)
        result.final_yaw_error = float(ctrl.yaw_err)
        getattr(goal_handle, outcome)()
        self.get_logger().info(f'Goal {outcome}: dist_err={ctrl.dist:.3f} yaw_err={ctrl.yaw_err:.3f}')
        return result


def main():
    rclpy.init()
    node = GoToPoseServer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
