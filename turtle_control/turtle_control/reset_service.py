import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute

START_X = START_Y = 5.544445
START_THETA = 0.0


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.cbg = ReentrantCallbackGroup()
        self.clear_cli = self.create_client(Empty, '/clear', callback_group=self.cbg)
        self.tele_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute',
                                           callback_group=self.cbg)
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen', callback_group=self.cbg)
        self.create_service(Trigger, 'reset_turtle', self._reset, callback_group=self.cbg)
        self.get_logger().info('reset_turtle service ready')

    def _call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {client.srv_name} not available')
        fut = client.call_async(req)
        end = time.time() + timeout
        while not fut.done():
            if time.time() > end:
                raise RuntimeError(f'timeout calling {client.srv_name}')
            time.sleep(0.01)
        return fut.result()

    def _pen(self, off):
        return SetPen.Request(r=179, g=184, b=255, width=3, off=off)  # turtlesim defaults

    def _reset(self, req, res):
        try:
            self._call(self.pen_cli, self._pen(1))                      # pen up
            self._call(self.tele_cli, TeleportAbsolute.Request(
                x=START_X, y=START_Y, theta=START_THETA))
            self._call(self.clear_cli, Empty.Request())
            self._call(self.pen_cli, self._pen(0))                      # pen down
            res.success, res.message = True, 'Turtle reset to start pose, screen cleared.'
        except Exception as e:  # noqa
            res.success, res.message = False, str(e)
        return res


def main():
    rclpy.init()
    node = ResetService()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
