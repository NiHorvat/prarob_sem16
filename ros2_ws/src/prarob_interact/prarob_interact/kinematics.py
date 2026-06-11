#!/usr/bin/env python3

from math import floor, pi

import numpy as np


class Kinematics:
    """Direct and inverse kinematics for the 3-DOF RRR drawing robot."""

    D1 = 0.0579
    D2 = 0.0209
    L1 = 0.224
    L2 = 0.125
    JOINT_LIMIT = pi / 2.0

    def get_dk(self, q):
        """Return end-effector position [x, y, z] for joint values [q1, q2, q3]."""
        if len(q) < 3:
            raise ValueError("get_dk expects [q1, q2, q3]")

        q1, q2, q3 = float(q[0]), float(q[1]), float(q[2])

        r = self.D2 + self.L1 * np.cos(q2) + self.L2 * np.cos(q2 + q3)
        z = self.D1 + self.L1 * np.sin(q2) + self.L2 * np.sin(q2 + q3)
        x = r * np.cos(q1)
        y = r * np.sin(q1)

        return [float(x), float(y), float(z)]

    def get_ik(self, w, q0=None):
        """Return all valid IK solutions [[q1, q2, q3], ...] for target [x, y, z]."""
        if len(w) < 3:
            raise ValueError("get_ik expects [x, y, z]")

        x, y, z = float(w[0]), float(w[1]), float(w[2])
        q1 = float(np.arctan2(y, x))

        r = float(np.sqrt(x**2 + y**2) - self.D2)
        zz = z - self.D1
        dist = float(np.sqrt(r**2 + zz**2))

        if dist > self.L1 + self.L2:
            return []
        if dist < abs(self.L1 - self.L2):
            return []

        cos_q3 = np.clip(
            (dist**2 - self.L1**2 - self.L2**2) / (2.0 * self.L1 * self.L2),
            -1.0,
            1.0,
        )

        solutions = []
        for q3 in (float(np.arccos(cos_q3)), float(-np.arccos(cos_q3))):
            beta = np.arctan2(
                self.L2 * np.sin(q3),
                self.L1 + self.L2 * np.cos(q3),
            )
            alpha = np.arctan2(zz, r)
            q2 = float(alpha - beta)
            candidate = [q1, q2, q3]

            if all(abs(angle) <= self.JOINT_LIMIT for angle in candidate):
                solutions.append([float(angle) for angle in candidate])

        return solutions

    def get_closest_ik(self, q_all, q0):
        """Return the IK solution closest to q0, or None if q_all is empty."""
        if not q_all:
            return None

        q0_array = np.array(q0 if q0 is not None else [0.0, 0.0, 0.0], dtype=float)
        return min(
            q_all,
            key=lambda q: np.linalg.norm(np.array(q, dtype=float) - q0_array),
        )

    def wrap2PI(self, x):
        return x - 2 * pi * floor(x / (2 * pi) + 0.5)
