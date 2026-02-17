# 3-Axis PID Attitude Controller Simulator

Python project that simulates a 3-axis rigid body (spacecraft/UAV style) and stabilizes it with a PID controller.

## Table of Contents

1. [What This Project Is](#what-this-project-is)
2. [What You Can Do Right Now](#what-you-can-do-right-now)
3. [Project Files](#project-files)
4. [Installation and Requirements](#installation-and-requirements)
5. [Interactive Prompt Mode](#interactive-prompt-mode)
6. [Command-Line Inputs Explained](#command-line-inputs-explained)
7. [Examples from Simple to Advanced](#examples-from-simple-to-advanced)
8. [How the Physics Model Works](#how-the-physics-model-works)
9. [How the PID Controller Works](#how-the-pid-controller-works)
10. [How to Tune PID Gains](#how-to-tune-pid-gains)
11. [Reading the Output](#reading-the-output)
12. [Plot and Animation Guide](#plot-and-animation-guide)
13. [Troubleshooting](#troubleshooting)
14. [Ideas for Extensions](#ideas-for-extensions)
15. [Notes](#notes)

## What This Project Is

This script simulates rotational motion in 3D and tries to point the body toward a desired orientation.

- State representation: quaternion + body angular rates
- Dynamics: nonlinear rigid-body rotational dynamics
- Controller: PID on attitude error and angular-rate damping
- Extras: terminal input mode, plots, and 3D orientation animation

Main script:
- `controller_3axis.py`

## What You Can Do Right Now

- Stabilize from a tilted, spinning start
- Command a custom target roll/pitch/yaw
- Tune `Kp`, `Ki`, `Kd`
- Limit actuator torque
- Visualize response with plots
- Watch orientation in a 3D animation

## Project Files

- `controller_3axis.py`: simulation, PID controller, CLI, plotting, animation
- `README.md`: full usage and theory guide

## Installation and Requirements

### Required

- Python 3.9+
- `numpy`

### Optional (for visuals)

- `matplotlib` for `--plot` and `--animate`

Install:

```bash
python3 -m pip install numpy matplotlib
```

Verify Python:

```bash
python3 --version
```

### 1. Default run

```bash
python3 controller_3axis.py
```

### 2. Show all options

```bash
python3 controller_3axis.py --help
```

### 3. Plot time histories

```bash
python3 controller_3axis.py --plot
```

### 4. 3D animation

```bash
python3 controller_3axis.py --animate
```

### 5. Plot + animation together

```bash
python3 controller_3axis.py --plot --animate
```

## Interactive Prompt Mode

If you prefer being asked questions instead of typing long flags:

```bash
python3 controller_3axis.py --interactive
```

You will be prompted for:

- target RPY (deg)
- initial RPY (deg)
- initial rates (rad/s)
- PID gains
- integrator/torque limits
- simulation time and timestep

Press `Enter` to keep defaults for any prompt.

## Command-Line Inputs Explained

### Attitude and rates

- `--target-rpy ROLL PITCH YAW`
  - Desired orientation in **degrees**
- `--init-rpy ROLL PITCH YAW`
  - Starting orientation in **degrees**
- `--init-rate WX WY WZ`
  - Starting angular rate in **rad/s**

### PID gains

- `--kp KPX KPY KPZ`
  - Proportional gains (higher = stronger correction)
- `--kd KDX KDY KDZ`
  - Derivative gains (higher = more damping)
- `--ki KIX KIY KIZ`
  - Integral gains (removes steady-state bias)
- `--i-limit VALUE`
  - Clamp for integral term to prevent windup

### Actuator and simulation settings

- `--tau-limit VALUE`
  - Max torque magnitude per axis (Nm)
- `--sim-time VALUE`
  - Total simulation time in seconds
- `--dt VALUE`
  - Simulation timestep in seconds

### Visuals

- `--plot`
  - Plots angle/rate/torque vs time
- `--animate`
  - Shows a 3D orientation animation

## Examples from Simple to Advanced

### Basic default

```bash
python3 controller_3axis.py
```

### Command a non-zero target

```bash
python3 controller_3axis.py --target-rpy 10 -5 20
```

### Harder initial disturbance

```bash
python3 controller_3axis.py --init-rpy 45 -30 60 --init-rate 1.0 -0.8 0.6
```

### Tune gains + plot

```bash
python3 controller_3axis.py --kp 12 12 10 --kd 3 3 2.5 --ki 0.2 0.2 0.2 --plot
```

### Strong torque limits (harder control)

```bash
python3 controller_3axis.py --tau-limit 0.2 --plot
```

### Full custom test

```bash
python3 controller_3axis.py \
  --target-rpy 15 -10 25 \
  --init-rpy 40 -25 55 \
  --init-rate 0.9 -0.6 0.4 \
  --kp 11 11 9 \
  --kd 2.8 2.8 2.2 \
  --ki 0.25 0.25 0.2 \
  --i-limit 0.9 \
  --tau-limit 0.45 \
  --sim-time 15 \
  --dt 0.01 \
  --plot --animate
```

## How the Physics Model Works

The model is rotational-only (attitude), not full XYZ position.

### State variables

- Quaternion `q = [w, x, y, z]` for orientation
- Angular velocity `w = [wx, wy, wz]` in body frame

### Rigid-body dynamics

The simulator uses:

`I * w_dot = tau - w x (I * w)`

Where:

- `I` = inertia matrix
- `tau` = commanded torque from PID
- `w x (I*w)` = gyroscopic coupling

### Quaternion integration

`q_dot = 0.5 * Omega(w) * q`

Quaternion is normalized each step so it stays a valid rotation.

## How the PID Controller Works

At each timestep:

1. Compute orientation error (quaternion error)
2. Convert small-angle error from quaternion vector part
3. Compute control torque:

`tau = Kp*e + Ki*integral(e) - Kd*w`

4. Clamp torque to `[-tau_limit, +tau_limit]` per axis
5. Step dynamics forward

### Intuition

- `Kp`: how hard to push toward target
- `Kd`: how much to resist spin (reduces overshoot)
- `Ki`: clears leftover offset over time

## How to Tune PID Gains

A practical tuning flow:

1. Start with `Ki = 0`.
2. Increase `Kp` until response is fast but not wildly oscillatory.
3. Increase `Kd` to reduce overshoot and ringing.
4. Add small `Ki` to remove remaining offset.
5. If controller saturates too often, raise `tau-limit` or reduce gains.

Symptoms and fixes:

- Too slow: increase `Kp`
- Overshoot/oscillation: increase `Kd` or reduce `Kp`
- Small final error remains: increase `Ki` slightly
- Unstable growth: gains too high or timestep too large

## Reading the Output

Script prints:

- `Final attitude [deg] roll=..., pitch=..., yaw=...`
- `Final rate norm [rad/s]: ...`

Good run signs:

- Final attitude near target
- Final rate norm close to `0`
- Smooth curves in plots

## Plot and Animation Guide

### Plot (`--plot`)

Three subplots:

- Angle (roll/pitch/yaw) in degrees
- Angular rates (rad/s)
- Commanded torques (Nm)

Use plot to judge:

- settling time
- overshoot
- oscillations
- control saturation behavior

### Animation (`--animate`)

Shows body axes in 3D over time.

- Red: body X
- Green: body Y
- Blue: body Z

Use it to visually confirm reorientation behavior.

## Troubleshooting

### `python3: command not found`

Install Python and retry.

### `No module named numpy` or `matplotlib`

Install packages:

```bash
python3 -m pip install numpy matplotlib
```

### Plot/animation window does not appear

- Make sure `matplotlib` is installed
- Run in local terminal (not headless environment)
- Try only one visual mode first (`--plot` or `--animate`)

### Simulation looks unstable

Try:

- smaller `--dt` (e.g. `0.005`)
- lower `Kp`
- higher `Kd`
- larger `--tau-limit` if saturated

### Interactive input error

For vector prompts, enter exactly 3 numbers.
Example: `10 -5 20`

## Ideas for Extensions

- Add disturbance torques (wind/impacts)
- Add sensor noise models
- Add gain sweep auto-tuner
- Add metrics (rise time, settling time, overshoot)
- Export logs to CSV
- Add actuator dynamics/latency
- Add LQR or MPC controller option

## Notes

- This project is a simulation and educational controller framework, not production flight software.
- Quaternions are used for orientation because they avoid singularities and provide robust 3D rotation integration.
- The simulator is suitable for learning and prototyping attitude-control ideas for UAV/spacecraft-style rigid bodies.
- Real hardware use requires additional work: sensor/actuator models, safety layers, real-time constraints, and validation.
- Units used in this project:
  - Angles in CLI inputs and printed attitude: degrees
  - Angular rates: rad/s
  - Torque: Nm
  - Time: seconds
- Recommended first tuning order:
  - Set `--target-rpy`, `--init-rpy`, `--init-rate`
  - Tune `--kp` and `--kd`
  - Add `--ki` after transient behavior is acceptable
