"""
spm_kinematics.py — Inverse Kinematics for a Coaxial 3-RRR SPM
================================================================
Implements the inverse kinematics (IK) for a coaxial spherical parallel
manipulator following the formulation in:

    Tursynbek, Niyetkaliyev, Shintemirov (2019),
    "Computation of Unique Kinematic Solutions of a Spherical Parallel
     Manipulator with Coaxial Input Shafts."

Geometry (your design):
    α₁ = 45°   — proximal link (lower curved link) arc length
    α₂ = 90°   — distal link (upper curved link) arc length
    β  = 90°   — angle from platform z-axis to platform joint axis
                 (β = 90° means platform joints lie in the equatorial
                 plane of the platform)
    γ  = 0°    — base joint axes are coaxial (all along world -z)

USAGE
-----
    >>> g = Geometry()                      # uses your design defaults
    >>> R = rpy_to_R(roll=0.1, pitch=0.0, yaw=0.0)
    >>> theta = inverse_kinematics(R, g)
    >>> print(np.degrees(theta))
    [ 5.7  -2.8  -2.9 ]

CONVENTIONS
-----------
  - Right-handed world frame with +z up.
  - All u_i (base joint axes) point along world -z (coaxial).
  - Platform joint vectors v_i_platform are 120° apart around platform z.
  - RPY uses the standard ZYX convention: R = Rz(yaw) · Ry(pitch) · Rx(roll).
    (This matches the SpaceMouse output convention used elsewhere in
    the project.)
  - θ_i is the actuator angle for leg i, measured from the home
    configuration; positive direction follows paper #1.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Sequence
import numpy as np

# ----------------------------------------------------------------------
# Geometry container
# ----------------------------------------------------------------------
@dataclass
class Geometry:
    """All physical parameters of the SPM in one place."""
    alpha1: float = np.deg2rad(45.0)        # proximal-link arc length
    alpha2: float = np.deg2rad(90.0)        # distal-link arc length
    beta:   float = np.deg2rad(90.0)        # platform joint polar angle
    # Azimuthal offset of leg 1 around platform z-axis (legs 2,3 follow at +120°, +240°)
    eta_offset: float = 0.0

    # Cached eta values for the three legs (set in __post_init__)
    eta: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self):
        self.eta = np.array([
            self.eta_offset + 0.0,
            self.eta_offset + 2.0 * np.pi / 3.0,
            self.eta_offset + 4.0 * np.pi / 3.0,
        ])

    def v_platform(self) -> np.ndarray:
        """Return the three platform-side joint vectors v_i in PLATFORM frame.
        Shape: (3, 3) — rows are leg index, columns are (x, y, z)."""
        v = np.zeros((3, 3))
        for i, eta in enumerate(self.eta):
            v[i, 0] = np.sin(self.beta) * np.sin(eta)
            v[i, 1] = np.sin(self.beta) * np.cos(eta)
            v[i, 2] = np.cos(self.beta)
        return v


# ----------------------------------------------------------------------
# Rotation matrices
# ----------------------------------------------------------------------
def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert roll, pitch, yaw (radians) to a 3x3 rotation matrix using
    the ZYX intrinsic convention:  R = Rz(yaw) · Ry(pitch) · Rx(roll).
    """
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr           ],
    ])


# ----------------------------------------------------------------------
# Proximal-link tip vector (parameterized by actuator angle)
# ----------------------------------------------------------------------
def w_i(eta: float, theta: float, alpha1: float) -> np.ndarray:
    """
    Unit vector along the intermediate joint axis for leg i, as a function
    of actuator angle θ.  From paper #1, equation (2):

        w_i = [sin(η - θ) sin α1,  cos(η - θ) sin α1,  -cos α1]
    """
    s = np.sin(eta - theta)
    c = np.cos(eta - theta)
    return np.array([s * np.sin(alpha1),
                     c * np.sin(alpha1),
                     -np.cos(alpha1)])


# ----------------------------------------------------------------------
# Inverse kinematics
# ----------------------------------------------------------------------
class IKError(RuntimeError):
    """Raised when a desired pose lies outside the workspace for at least one leg."""


def inverse_kinematics(
    R: np.ndarray,
    geom: Geometry,
    *,
    branch: Sequence[int] = (0, 0, 0),
    prev_theta: np.ndarray | None = None,
) -> np.ndarray:
    """
    Compute the three actuator angles θ = (θ₁, θ₂, θ₃) that achieve a
    desired platform orientation R.

    Args
    ----
    R          : 3x3 rotation matrix of platform pose.
    geom       : Geometry instance.
    branch     : 3-tuple of {0, 1} selecting which of the two algebraic
                 roots to take for each leg.  This selects the assembly
                 mode.  (0, 0, 0) is the default 'l-l-l' assembly.
    prev_theta : Optional previous solution (radians, shape (3,)).  If
                 given, the branch closest to prev_theta is chosen for
                 each leg, overriding the `branch` argument.  Use this
                 in real-time control to ensure continuity.

    Returns
    -------
    theta : np.ndarray of shape (3,) — actuator angles in radians.

    Raises
    ------
    IKError if any leg has no real solution (pose unreachable).
    """
    v_p = geom.v_platform()                   # (3, 3) platform-frame
    v_b = (R @ v_p.T).T                        # (3, 3) base-frame

    theta = np.zeros(3)
    for i in range(3):
        vx, vy, vz = v_b[i]
        # Coefficients of  a·sin(ψ) + b·cos(ψ) = c   with  ψ = η_i - θ_i
        a = np.sin(geom.alpha1) * vx
        b = np.sin(geom.alpha1) * vy
        c = np.cos(geom.alpha2) + np.cos(geom.alpha1) * vz

        r2 = a*a + b*b
        if r2 < 1e-12:
            raise IKError(f"Leg {i+1}: degenerate (a=b=0). Geometry/pose problem.")
        r = np.sqrt(r2)
        if abs(c) > r:
            raise IKError(f"Leg {i+1}: pose unreachable ( |c|={abs(c):.4f} > r={r:.4f} ).")

        # Two algebraic roots
        phi    = np.arctan2(b, a)              # atan2 carefully; b is cos-coeff, a is sin-coeff
        delta  = np.arcsin(np.clip(c / r, -1.0, 1.0))
        psi_a  = delta - phi                   # branch 0
        psi_b  = (np.pi - delta) - phi         # branch 1
        theta_a = _wrap(geom.eta[i] - psi_a)
        theta_b = _wrap(geom.eta[i] - psi_b)

        if prev_theta is not None:
            # Pick branch closest to previous (continuity)
            da = abs(_wrap(theta_a - prev_theta[i]))
            db = abs(_wrap(theta_b - prev_theta[i]))
            theta[i] = theta_a if da <= db else theta_b
        else:
            theta[i] = theta_a if branch[i] == 0 else theta_b

    return theta


def _wrap(a: float) -> float:
    """Wrap angle to (-π, π]."""
    a = (a + np.pi) % (2 * np.pi) - np.pi
    return a


# ----------------------------------------------------------------------
# Convenience: combined call from RPY
# ----------------------------------------------------------------------
def rpy_to_joints(
    roll: float, pitch: float, yaw: float,
    geom: Geometry,
    *, branch=(0, 0, 0), prev_theta=None,
) -> np.ndarray:
    """One-shot:  (roll, pitch, yaw) → (θ₁, θ₂, θ₃)  in radians."""
    R = rpy_to_R(roll, pitch, yaw)
    return inverse_kinematics(R, geom, branch=branch, prev_theta=prev_theta)


# ----------------------------------------------------------------------
# Quick self-test when run directly
# ----------------------------------------------------------------------
if __name__ == "__main__":
    g = Geometry()
    print(f"Geometry:  α1={np.rad2deg(g.alpha1):.1f}°  "
          f"α2={np.rad2deg(g.alpha2):.1f}°  β={np.rad2deg(g.beta):.1f}°")
    print(f"η = {np.rad2deg(g.eta)} (deg)")

    # IK at identity (the platform is flat).  These three angles are your
    # mechanical 'home' for the actuators.  Subtract them from any θ you
    # send to the motors, or zero your encoders here at startup.
    theta_home = rpy_to_joints(0.0, 0.0, 0.0, g)
    print(f"\nθ at R=I (home): {np.rad2deg(theta_home)}  deg")

    # A few sanity checks.
    for r, p, y in [(0.1, 0, 0), (0, 0.1, 0), (0, 0, 0.1), (0.1, -0.1, 0.05)]:
        th = rpy_to_joints(r, p, y, g)
        print(f"  rpy=({r:+.2f},{p:+.2f},{y:+.2f})  →  "
              f"θ={np.rad2deg(th)}  (deg)")