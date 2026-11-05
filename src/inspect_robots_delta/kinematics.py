"""Inverse and forward kinematics of a 3-RRR rotary delta robot (Clavel type).

Frame: origin at the centre of the base plate, z up, so the effector works at
negative z. Arm ``i`` lies in the vertical plane at azimuth ``120 deg * i``.
Joint angle ``theta = 0`` is the upper arm horizontal; positive is downward.
Units are metres and radians.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

AZIMUTHS = np.deg2rad([0.0, 120.0, 240.0])


@dataclass(frozen=True)
class DeltaGeometry:
        """Link lengths of the delta. Defaults are a mid-size picker."""

        base_radius: float = 0.200  # shoulder axis distance from base centre
        effector_radius: float = 0.050  # forearm attachment distance from TCP
        upper_arm: float = 0.300
        forearm: float = 0.700
        theta_min: float = -0.70  # rad, joint limit (upward)
        theta_max: float = 1.40  # rad, joint limit (downward)


class Unreachable(ValueError):
        """The target is outside the kinematic workspace or the joint limits."""


def inverse(geo: DeltaGeometry, p: np.ndarray) -> np.ndarray:
        """Return the three joint angles that place the TCP at ``p``."""
        x, y, z = (float(v) for v in p)
        L, l = geo.upper_arm, geo.forearm
        out = np.empty(3)
        for i, phi in enumerate(AZIMUTHS):
                # Rotate the target into arm i's plane; the arm points along +x'.
                xr = np.cos(phi) * x + np.sin(phi) * y
                yr = -np.sin(phi) * x + np.cos(phi) * y
                a = geo.base_radius - geo.effector_radius - xr
                # (a + L cos t)^2 + yr^2 + (z + L sin t)^2 = l^2  ->  A cos t + B sin t = K
                A, B = 2 * a * L, 2 * z * L
                K = l * l - a * a - L * L - yr * yr - z * z
                r = np.hypot(A, B)
                if r < 1e-12 or abs(K) > r:
                        raise Unreachable(f"arm {i} cannot reach {p.tolist()}")
                base = np.arctan2(B, A)
                d = np.arccos(K / r)
                # Elbow-out branch: the root with the larger cos(theta).
                t = max((base + d, base - d), key=np.cos)
                t = (t + np.pi) % (2 * np.pi) - np.pi
                if not geo.theta_min <= t <= geo.theta_max:
                        raise Unreachable(f"arm {i} needs {t:.3f} rad, outside joint limits")
                out[i] = t
        return out


def elbows(geo: DeltaGeometry, theta: np.ndarray) -> np.ndarray:
        """Elbow positions (3x3) in the base frame."""
        pts = np.empty((3, 3))
        for i, (phi, t) in enumerate(zip(AZIMUTHS, theta)):
                rad = geo.base_radius + geo.upper_arm * np.cos(t)
                pts[i] = (rad * np.cos(phi), rad * np.sin(phi), -geo.upper_arm * np.sin(t))
        return pts


def forward(geo: DeltaGeometry, theta: np.ndarray) -> np.ndarray:
        """Return the TCP position for joint angles ``theta`` (lower assembly)."""
        # Shift each elbow inward by the effector radius: the TCP is then the
        # intersection of three spheres of radius ``forearm``.
        c = elbows(geo, theta)
        for i, phi in enumerate(AZIMUTHS):
                c[i, :2] -= geo.effector_radius * np.array([np.cos(phi), np.sin(phi)])
        p1, p2, p3 = c
        ex = p2 - p1
        d = np.linalg.norm(ex)
        ex /= d
        i_ = float(ex @ (p3 - p1))
        ey = p3 - p1 - i_ * ex
        j = np.linalg.norm(ey)
        ey /= j
        ez = np.cross(ex, ey)
        r = geo.forearm
        x = d / 2
        y = (i_ * i_ + j * j) / (2 * j) - i_ * x / j
        z2 = r * r - x * x - y * y
        if z2 < 0:
                raise Unreachable("forearms cannot close for these joint angles")
        base = p1 + x * ex + y * ey
        cand = (base + np.sqrt(z2) * ez, base - np.sqrt(z2) * ez)
        return min(cand, key=lambda q: q[2])
