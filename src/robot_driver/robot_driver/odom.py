"""
Simple differential-drive odometry helper.

This module converts absolute encoder counts (signed 32-bit) into a
pose and velocity estimate for a two-wheel differential-drive robot.

Notes for debugging:
- The class expects absolute encoder counts (monotonically increasing or
    wrapping in 32-bit). If your controller provides relative counts, adapt
    accordingly.
- `encoder_cpr` should match the motor encoder resolution (counts per
    revolution). Wrong CPR leads to wrong distances.
- The first `update()` call initializes previous counters and returns a
    zero-motion state; subsequent calls compute deltas.
"""

import math
from dataclasses import dataclass


@dataclass
class OdomState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    omega: float = 0.0
    left_count: int = 0
    right_count: int = 0
    left_distance: float = 0.0
    right_distance: float = 0.0


class Odometry:
    """
    Nhận encoder count tuyệt đối của bánh trái/phải và tính odometry cho xe 2 bánh vi sai.
    """

    def __init__(self, wheel_radius: float, wheel_base: float, encoder_cpr: int):
        self.R = wheel_radius
        self.L = wheel_base
        self.CPR = encoder_cpr

        self.state = OdomState()
        self.prev_left_count = None
        self.prev_right_count = None

    def reset(self) -> None:
        self.state = OdomState()
        self.prev_left_count = None
        self.prev_right_count = None

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def counts_to_distance(self, delta_count: int) -> float:
        revolutions = delta_count / self.CPR
        return revolutions * 2.0 * math.pi * self.R

    def update(self, left_count: int, right_count: int, dt: float) -> OdomState:
        self.state.left_count = left_count
        self.state.right_count = right_count

        if self.prev_left_count is None or self.prev_right_count is None:
            self.prev_left_count = left_count
            self.prev_right_count = right_count
            return self.state

        delta_left = left_count - self.prev_left_count
        delta_right = right_count - self.prev_right_count

        self.prev_left_count = left_count
        self.prev_right_count = right_count

        dL = self.counts_to_distance(delta_left)
        dR = self.counts_to_distance(delta_right)

        self.state.left_distance += dL
        self.state.right_distance += dR

        dS = (dL + dR) / 2.0
        dTheta = (dR - dL) / self.L

        theta_mid = self.state.theta + 0.5 * dTheta

        self.state.x += dS * math.cos(theta_mid)
        self.state.y += dS * math.sin(theta_mid)
        self.state.theta = self.normalize_angle(self.state.theta + dTheta)

        if dt > 0:
            self.state.vx = dS / dt
            self.state.omega = dTheta / dt
        else:
            self.state.vx = 0.0
            self.state.omega = 0.0

        return self.state

    def get_state(self) -> OdomState:
        return self.state