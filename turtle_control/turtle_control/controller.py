"""Pure-Python go-to-pose controller (no ROS), implemented as a small state machine."""
import math
from enum import Enum


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clip(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


class Phase(Enum):
    ROTATE_TO_GOAL = 'rotate_to_goal'
    DRIVE = 'drive'
    ALIGN_YAW = 'align_yaw'
    DONE = 'done'


class PoseController:
    def __init__(self, gx, gy, gyaw, pos_tol=0.05, yaw_tol=0.03,
                 k_lin=1.5, k_ang=4.0, turn_threshold=0.5):
        self.gx, self.gy, self.gyaw = gx, gy, normalize_angle(gyaw)
        self.pos_tol, self.yaw_tol = pos_tol, yaw_tol
        self.k_lin, self.k_ang, self.turn_threshold = k_lin, k_ang, turn_threshold
        self.phase = Phase.ROTATE_TO_GOAL
        self.dist = 0.0
        self.yaw_err = 0.0

    def compute(self, x, y, theta, max_lin, max_ang):
        """Return (linear, angular) command; updates self.phase / dist / yaw_err."""
        dx, dy = self.gx - x, self.gy - y
        self.dist = math.hypot(dx, dy)
        self.yaw_err = normalize_angle(self.gyaw - theta)
        lin = ang = 0.0

        if self.phase in (Phase.ROTATE_TO_GOAL, Phase.DRIVE) and self.dist < self.pos_tol:
            self.phase = Phase.ALIGN_YAW

        if self.phase is Phase.ROTATE_TO_GOAL:
            err = normalize_angle(math.atan2(dy, dx) - theta)
            if abs(err) < 0.05:
                self.phase = Phase.DRIVE
            else:
                ang = clip(self.k_ang * err, max_ang)
        if self.phase is Phase.DRIVE:
            err = normalize_angle(math.atan2(dy, dx) - theta)
            if abs(err) > self.turn_threshold:
                self.phase = Phase.ROTATE_TO_GOAL
            else:
                lin = min(max(self.k_lin * self.dist, min(0.1, max_lin)), max_lin)
                ang = clip(self.k_ang * err, max_ang)
        elif self.phase is Phase.ALIGN_YAW:
            if abs(self.yaw_err) < self.yaw_tol:
                self.phase = Phase.DONE
            else:
                ang = clip(self.k_ang * self.yaw_err, max_ang)
                if abs(ang) < 0.1:
                    ang = math.copysign(min(0.1, max_ang), self.yaw_err)
        return lin, ang
