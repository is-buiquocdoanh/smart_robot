import math
from dataclasses import dataclass


@dataclass
class OdomState:

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    omega: float = 0.0


class Odometry:

    def __init__(self, wheel_radius, wheel_base, encoder_cpr):

        self.R = wheel_radius
        self.L = wheel_base
        self.CPR = encoder_cpr

        self.x = 0
        self.y = 0
        self.theta = 0

        self.prev_left = None
        self.prev_right = None

    def counts_to_distance(self, delta):

        rev = delta / self.CPR

        return rev * 2 * math.pi * self.R

    def update(self, left, right, dt):

        if self.prev_left is None:

            self.prev_left = left
            self.prev_right = right

            return OdomState()

        dL = left - self.prev_left
        dR = right - self.prev_right

        self.prev_left = left
        self.prev_right = right

        dL = self.counts_to_distance(dL)
        dR = self.counts_to_distance(dR)

        dS = (dL + dR) / 2
        dTheta = (dR - dL) / self.L

        self.x += dS * math.cos(self.theta + dTheta/2)
        self.y += dS * math.sin(self.theta + dTheta/2)
        self.theta += dTheta

        vx = dS / dt
        omega = dTheta / dt

        return OdomState(self.x, self.y, self.theta, vx, omega)