#!/usr/bin/env python3
"""
3-axis attitude stabilization demo for a spacecraft/UAV rigid body.

Includes:
- Nonlinear rotational dynamics (body rates + quaternion attitude)
- PID attitude controller

Run:
    python3 controller_3axis.py --plot
    python3 controller_3axis.py --animate
    python3 controller_3axis.py --target-rpy 10 -5 20 --init-rpy 30 -20 40 --init-rate 0.8 -0.5 0.2
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np


def normalize_quat(q: np.ndarray) -> np.ndarray:
    # Keep quaternion unit-length so it remains a valid rotation.
    return q / np.linalg.norm(q)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    # Hamilton product: composition of two rotations.
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    # Quaternion inverse for unit quaternions.
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    # Convert roll/pitch/yaw (rad) to quaternion [w, x, y, z].
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=float,
    )
    return normalize_quat(q)


def quat_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    # Convert quaternion back to roll/pitch/yaw (rad) for readable output.
    w, x, y, z = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quat_error(q_current: np.ndarray, q_desired: np.ndarray) -> np.ndarray:
    # Error quaternion that rotates current attitude to desired attitude.
    q_err = quat_multiply(quat_conjugate(q_current), q_desired)
    # Keep scalar part positive so we always use the shortest rotation.
    if q_err[0] < 0.0:
        q_err = -q_err
    return q_err


def omega_matrix(w: np.ndarray) -> np.ndarray:
    # Matrix form of angular velocity used in quaternion kinematics.
    wx, wy, wz = w
    return np.array(
        [
            [0.0, -wx, -wy, -wz],
            [wx, 0.0, wz, -wy],
            [wy, -wz, 0.0, wx],
            [wz, wy, -wx, 0.0],
        ],
        dtype=float,
    )


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    # Convert unit quaternion [w, x, y, z] to a 3x3 rotation matrix.
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


@dataclass
class RigidBody:
    inertia: np.ndarray

    def step(self, q: np.ndarray, w: np.ndarray, tau: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        # Euler rotational dynamics: I*w_dot = tau - w x (I*w)
        inv_i = np.linalg.inv(self.inertia)
        w_dot = inv_i @ (tau - np.cross(w, self.inertia @ w))
        w_next = w + w_dot * dt

        # Quaternion kinematics: q_dot = 0.5 * Omega(w) * q
        q_dot = 0.5 * omega_matrix(w_next) @ q
        q_next = normalize_quat(q + q_dot * dt)
        return q_next, w_next


@dataclass
class PIDController:
    kp: np.ndarray
    kd: np.ndarray
    ki: np.ndarray
    i_limit: float

    integral: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def compute(self, q: np.ndarray, w: np.ndarray, q_des: np.ndarray, dt: float) -> np.ndarray:
        # Attitude error between current and target orientation.
        q_err = quat_error(q, q_des)
        # For small angles, quaternion vector part is proportional to attitude error.
        e = q_err[1:]
        # Integrator helps remove steady-state bias/disturbance.
        self.integral = np.clip(self.integral + e * dt, -self.i_limit, self.i_limit)
        # P on angle error, D on body rates, I on accumulated error.
        tau = self.kp * e + self.ki * self.integral - self.kd * w
        return tau


def run_sim(
    sim_time: float,
    dt: float,
    plot: bool,
    target_rpy_deg: np.ndarray,
    init_rpy_deg: np.ndarray,
    init_rate: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    ki: np.ndarray,
    i_limit: float,
    tau_limit_val: float,
    animate: bool,
) -> None:
    # Build the simulated rigid body and controller from user inputs.
    # Example principal moments of inertia for a small rigid vehicle.
    inertia = np.diag([0.08, 0.09, 0.12])
    body = RigidBody(inertia=inertia)

    # Commanded attitude (roll/pitch/yaw in deg from CLI).
    q_des = euler_to_quat(*np.radians(target_rpy_deg))
    # Initial disturbed attitude/rates from CLI.
    q = euler_to_quat(*np.radians(init_rpy_deg))
    w = init_rate.astype(float)

    # Actuator saturation limits.
    tau_limit = np.array([tau_limit_val, tau_limit_val, tau_limit_val], dtype=float)

    ctrl = PIDController(
        kp=kp,
        kd=kd,
        ki=ki,
        i_limit=i_limit,
    )

    n = int(sim_time / dt)
    # Logs used for printed summary and optional plotting.
    ts = np.zeros(n)
    quats = np.zeros((n, 4))
    eulers = np.zeros((n, 3))
    rates = np.zeros((n, 3))
    torques = np.zeros((n, 3))

    for k in range(n):
        tau_cmd = ctrl.compute(q, w, q_des, dt)
        tau_cmd = np.clip(tau_cmd, -tau_limit, tau_limit)

        # Propagate plant state one time step.
        q, w = body.step(q, w, tau_cmd, dt)

        ts[k] = k * dt
        quats[k] = q
        eulers[k] = np.degrees(np.array(quat_to_euler(q)))
        rates[k] = w
        torques[k] = tau_cmd

    final_roll, final_pitch, final_yaw = eulers[-1]
    final_rate_norm = np.linalg.norm(rates[-1])
    print("Controller: PID")
    print(f"Final attitude [deg] roll={final_roll:.3f}, pitch={final_pitch:.3f}, yaw={final_yaw:.3f}")
    print(f"Final rate norm [rad/s]: {final_rate_norm:.4f}")

    if plot:
        try:
            import matplotlib.pyplot as plt  # type: ignore
        except Exception:
            print("matplotlib not available; skipping plots.")
            return

        # Three stacked plots: attitude, angular rate, and commanded torque.
        fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        labels = ["Roll", "Pitch", "Yaw"]
        for i in range(3):
            axes[0].plot(ts, eulers[:, i], label=labels[i])
            axes[1].plot(ts, rates[:, i], label=f"w{i+1}")
            axes[2].plot(ts, torques[:, i], label=f"tau{i+1}")

        axes[0].set_ylabel("Angle [deg]")
        axes[0].grid(True)
        axes[0].legend()
        axes[1].set_ylabel("Rate [rad/s]")
        axes[1].grid(True)
        axes[1].legend()
        axes[2].set_ylabel("Torque [N m]")
        axes[2].set_xlabel("Time [s]")
        axes[2].grid(True)
        axes[2].legend()
        fig.suptitle("3-axis stabilization with PID")
        plt.tight_layout()
        plt.show()

    if animate:
        try:
            import matplotlib.pyplot as plt  # type: ignore
            from matplotlib.animation import FuncAnimation  # type: ignore
        except Exception:
            print("matplotlib not available; skipping animation.")
            return

        # Downsample frames so long simulations remain smooth.
        frame_step = max(1, len(quats) // 400)
        quats_ds = quats[::frame_step]
        ts_ds = ts[::frame_step]

        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_title("3-axis orientation animation")
        lim = 1.2
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-lim, lim)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_box_aspect((1, 1, 1))

        # Fixed world axes for reference.
        ax.plot([-1, 1], [0, 0], [0, 0], "k--", alpha=0.2)
        ax.plot([0, 0], [-1, 1], [0, 0], "k--", alpha=0.2)
        ax.plot([0, 0], [0, 0], [-1, 1], "k--", alpha=0.2)

        line_x, = ax.plot([], [], [], "r-", lw=3, label="Body X")
        line_y, = ax.plot([], [], [], "g-", lw=3, label="Body Y")
        line_z, = ax.plot([], [], [], "b-", lw=3, label="Body Z")
        time_text = ax.text2D(0.03, 0.95, "", transform=ax.transAxes)
        ax.legend(loc="upper right")

        body_axes = np.eye(3)

        def init() -> tuple:
            line_x.set_data([], [])
            line_x.set_3d_properties([])
            line_y.set_data([], [])
            line_y.set_3d_properties([])
            line_z.set_data([], [])
            line_z.set_3d_properties([])
            time_text.set_text("")
            return line_x, line_y, line_z, time_text

        def update(i: int) -> tuple:
            r = quat_to_rotmat(quats_ds[i])
            axes_world = r @ body_axes

            line_x.set_data([0, axes_world[0, 0]], [0, axes_world[1, 0]])
            line_x.set_3d_properties([0, axes_world[2, 0]])
            line_y.set_data([0, axes_world[0, 1]], [0, axes_world[1, 1]])
            line_y.set_3d_properties([0, axes_world[2, 1]])
            line_z.set_data([0, axes_world[0, 2]], [0, axes_world[1, 2]])
            line_z.set_3d_properties([0, axes_world[2, 2]])
            time_text.set_text(f"t = {ts_ds[i]:.2f} s")
            return line_x, line_y, line_z, time_text

        interval_ms = max(1, int(dt * frame_step * 1000))
        anim = FuncAnimation(
            fig,
            update,
            init_func=init,
            frames=len(quats_ds),
            interval=interval_ms,
            blit=False,
            repeat=False,
        )
        # Keep a reference so animation is not garbage collected before show().
        _ = anim
        plt.show()


def prompt_vec3(label: str, default: np.ndarray) -> np.ndarray:
    # Parse three numbers from prompt; Enter keeps default.
    raw = input(f"{label} [{default[0]} {default[1]} {default[2]}]: ").strip()
    if not raw:
        return default.copy()
    parts = raw.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError(f"{label} requires exactly 3 numbers.")
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)


def prompt_scalar(label: str, default: float) -> float:
    # Parse single numeric input; Enter keeps default.
    raw = input(f"{label} [{default}]: ").strip()
    return default if not raw else float(raw)


def parse_args() -> argparse.Namespace:
    # CLI path for scripted tests and reproducible runs.
    p = argparse.ArgumentParser(description="3-axis rigid-body stabilization with PID")
    p.add_argument("--sim-time", type=float, default=12.0)
    p.add_argument("--dt", type=float, default=0.01)
    p.add_argument("--target-rpy", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("ROLL", "PITCH", "YAW"))
    p.add_argument("--init-rpy", type=float, nargs=3, default=[20.0, -15.0, 30.0], metavar=("ROLL", "PITCH", "YAW"))
    p.add_argument("--init-rate", type=float, nargs=3, default=[0.6, -0.4, 0.35], metavar=("WX", "WY", "WZ"))
    p.add_argument("--kp", type=float, nargs=3, default=[9.0, 9.0, 8.0], metavar=("KPX", "KPY", "KPZ"))
    p.add_argument("--kd", type=float, nargs=3, default=[2.0, 2.0, 1.8], metavar=("KDX", "KDY", "KDZ"))
    p.add_argument("--ki", type=float, nargs=3, default=[0.4, 0.4, 0.3], metavar=("KIX", "KIY", "KIZ"))
    p.add_argument("--i-limit", type=float, default=0.8)
    p.add_argument("--tau-limit", type=float, default=0.5)
    p.add_argument("--interactive", action="store_true", help="Prompt for all inputs in terminal")
    p.add_argument("--plot", action="store_true")
    p.add_argument("--animate", action="store_true", help="Show 3D orientation animation")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    # Convert argparse lists into numpy vectors used by the simulator.
    target_rpy = np.array(args.target_rpy, dtype=float)
    init_rpy = np.array(args.init_rpy, dtype=float)
    init_rate = np.array(args.init_rate, dtype=float)
    kp = np.array(args.kp, dtype=float)
    kd = np.array(args.kd, dtype=float)
    ki = np.array(args.ki, dtype=float)
    sim_time = float(args.sim_time)
    dt = float(args.dt)
    i_limit = float(args.i_limit)
    tau_limit = float(args.tau_limit)

    if args.interactive:
        # Prompt mode for quick manual tuning without long CLI flags.
        print("Interactive mode: press Enter to keep defaults.")
        target_rpy = prompt_vec3("Target RPY deg", target_rpy)
        init_rpy = prompt_vec3("Initial RPY deg", init_rpy)
        init_rate = prompt_vec3("Initial body rates rad/s", init_rate)
        kp = prompt_vec3("Kp gains", kp)
        kd = prompt_vec3("Kd gains", kd)
        ki = prompt_vec3("Ki gains", ki)
        i_limit = prompt_scalar("Integrator limit", i_limit)
        tau_limit = prompt_scalar("Torque limit (Nm)", tau_limit)
        sim_time = prompt_scalar("Simulation time (s)", sim_time)
        dt = prompt_scalar("Time step dt (s)", dt)

    run_sim(
        # All inputs are now normalized into numeric arrays/scalars.
        sim_time=sim_time,
        dt=dt,
        plot=args.plot,
        animate=args.animate,
        target_rpy_deg=target_rpy,
        init_rpy_deg=init_rpy,
        init_rate=init_rate,
        kp=kp,
        kd=kd,
        ki=ki,
        i_limit=i_limit,
        tau_limit_val=tau_limit,
    )


if __name__ == "__main__":
    main()
