import numpy as np
import math

class OdomNoiseModel:
    def __init__(self, alpha1=0.05, alpha2=0.01, alpha3=0.02, alpha4=0.01):
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.alpha3 = alpha3
        self.alpha4 = alpha4
        self._last_stddevs = (0.0, 0.0, 0.0)

    def compute_noisy_velocities(self, prev: dict, curr: dict):
        dt = curr["t"] - prev["t"]
        if dt <= 0.0:
            self._last_stddevs = (0.0, 0.0, 0.0)
            return 0.0, 0.0, 0.0, 0.0

        x1, y1, theta1 = prev["x"], prev["y"], prev["yaw"]
        x2, y2, theta2 = curr["x"], curr["y"], curr["yaw"]

        dx = x2 - x1
        dy = y2 - y1
        trasp = math.sqrt(dx**2 + dy**2)
        rot1 = math.atan2(dy, dx) - theta1
        rot2 = theta2 - theta1 - rot1

        sd_rot1 = self.alpha1 * abs(rot1) + self.alpha2 * trasp
        sd_rot2 = self.alpha1 * abs(rot2) + self.alpha2 * trasp
        sd_trasp = self.alpha3 * trasp + self.alpha4 * (abs(rot1) + abs(rot2))
        self._last_stddevs = (sd_trasp, sd_rot1, sd_rot2)

        noisy_trasp = trasp + np.random.normal(0, sd_trasp**2)
        noisy_rot1 = rot1 + np.random.normal(0, sd_rot1**2)
        noisy_rot2 = rot2 + np.random.normal(0, sd_rot2**2)

        vx = noisy_trasp * math.cos(theta1 + noisy_rot1) / dt
        vy = noisy_trasp * math.sin(theta1 + noisy_rot1) / dt
        vz = 0.0
        wz = (noisy_rot1 + noisy_rot2) / dt

        return vx, vy, vz, wz

    def get_last_stddevs(self):
        return self._last_stddevs
