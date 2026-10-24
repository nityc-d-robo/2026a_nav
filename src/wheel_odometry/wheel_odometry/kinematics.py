"""Planar kinematics for conventional omni wheels (not mecanum rollers)."""

import math

import numpy as np


class OmniOdometry:
    def __init__(self, x, y, drive_angles, radii, signs):
        n = len(x)
        arrays = [np.asarray(a, dtype=float) for a in (x, y, drive_angles, radii, signs)]
        if n < 3 or any(a.shape != (n,) or not np.all(np.isfinite(a)) for a in arrays):
            raise ValueError('Specify equal-length finite wheel arrays with at least 3 wheels')
        x, y, angles, self.radii, self.signs = arrays
        if np.any(self.radii <= 0) or np.any(np.abs(self.signs) != 1):
            raise ValueError('wheel_radii must be positive; wheel_signs must be +1 or -1')
        c, s = np.cos(angles), np.sin(angles)
        # Rolling distance = c*dx + s*dy + (-y*c + x*s)*dyaw.
        self.matrix = np.column_stack((c, s, -y * c + x * s))
        if np.linalg.matrix_rank(self.matrix) != 3:
            raise ValueError('Wheel geometry cannot observe x, y and yaw independently')
        self.inverse = np.linalg.pinv(self.matrix)
        self.pose = np.zeros(3)
        self.previous = None
        self.previous_stamp = None
        self.previous_velocity = None
        self.velocity_stamp = None
        self.integrated_angles = np.zeros(n)

    def update_velocity(self, angular_velocity, stamp_ns, max_interval=0.2):
        """Integrate wheel rad/s by trapezoidal rule. Rebaseline after a long gap.

        Use one input mode per instance. Never integrate a stale speed indefinitely.
        """
        velocity = np.asarray(angular_velocity, dtype=float)
        if velocity.shape != self.radii.shape or not np.all(np.isfinite(velocity)):
            raise ValueError('Wheel velocities have incorrect length or non-finite values')
        if stamp_ns <= 0 or (self.velocity_stamp is not None and stamp_ns <= self.velocity_stamp):
            raise ValueError('Measurement timestamps must be positive and strictly increasing')
        if self.velocity_stamp is not None:
            dt = (stamp_ns - self.velocity_stamp) * 1e-9
            if dt <= max_interval:
                self.integrated_angles += (self.previous_velocity + velocity) * 0.5 * dt
        pose, _ = self.update(self.integrated_angles, stamp_ns)
        self.previous_velocity = velocity.copy()
        self.velocity_stamp = stamp_ns
        # Twist is the instantaneous measured body velocity, not the interval average.
        body_velocity = self.inverse @ (velocity * self.radii * self.signs)
        return pose, body_velocity

    def update(self, angles, stamp_ns):
        """Accept simultaneous, unwrapped cumulative wheel angles in radians.

        The first sample establishes the encoder baseline and returns zero twist.
        Invalid/non-increasing samples leave the state unchanged.
        """
        angles = np.asarray(angles, dtype=float)
        if angles.shape != self.radii.shape or not np.all(np.isfinite(angles)):
            raise ValueError('Wheel angles have incorrect length or non-finite values')
        if stamp_ns <= 0 or (self.previous_stamp is not None and stamp_ns <= self.previous_stamp):
            raise ValueError('Measurement timestamps must be positive and strictly increasing')
        velocity = np.zeros(3)
        if self.previous is not None:
            delta = self.inverse @ ((angles - self.previous) * self.radii * self.signs)
            dx, dy, theta = delta
            # SE(2) exponential: integrate body translation during rotation.
            a = math.sin(theta) / theta if abs(theta) > 1e-8 else 1 - theta * theta / 6
            b = (1 - math.cos(theta)) / theta if abs(theta) > 1e-8 else theta / 2
            bx, by = a * dx - b * dy, b * dx + a * dy
            yaw = self.pose[2]
            self.pose[0] += math.cos(yaw) * bx - math.sin(yaw) * by
            self.pose[1] += math.sin(yaw) * bx + math.cos(yaw) * by
            self.pose[2] = math.atan2(math.sin(yaw + theta), math.cos(yaw + theta))
            velocity = delta / ((stamp_ns - self.previous_stamp) * 1e-9)
        self.previous = angles.copy()
        self.previous_stamp = stamp_ns
        return self.pose.copy(), velocity
