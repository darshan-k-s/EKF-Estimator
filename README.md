# EKF-Based Attitude & Gyroscope Bias Estimator

A real-time **Extended Kalman Filter** implementation in MATLAB for 3-DOF attitude estimation with simultaneous gyroscope bias identification. The system fuses IMU angular rate data with planar surface observations extracted from a depth camera to produce robust, drift-corrected roll/pitch/yaw estimates.

> *Developed as part of UNSW MTRN4010: Advanced Autonomous Systems coursework.*

***

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [EKF Formulation](#ekf-formulation)
  - [State Representation](#state-representation)
  - [Process Model & Prediction](#process-model--prediction)
  - [Measurement Model & Update](#measurement-model--update)
  - [Augmented State — Bias Estimation](#augmented-state--bias-estimation)
- [Gyroscope Bias Pre-Estimator](#gyroscope-bias-pre-estimator)
- [Implementation Details](#implementation-details)
- [Getting Started](#getting-started)

***

## Overview

Gyroscopes drift over time due to a slowly-varying bias on each axis. Without correcting for this bias, integrating angular rate produces attitude estimates that diverge unboundedly. This project addresses the problem with two complementary strategies:

1. **Offline bias pre-estimation** — a lightweight least-squares estimator bootstraps the initial bias from sparse attitude snapshots (Part A).
2. **Online bias estimation via augmented EKF** — the filter jointly tracks attitude and 3-axis bias in a 6-dimensional state, using planar surface observations (floor and walls) from a depth camera as aperiodic measurement updates.

**Key results:**
- Bias estimation accuracy: all axes within **0.15 deg/s** of injected ground-truth
- Yaw-observable updates via wall normals; roll/pitch updates via floor normals
- Real-time operation validated against a reference API implementation

***

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     Sensor Stream                       │
│   IMU (gyro ωx,ωy,ωz)      RGB-D Depth Camera          │
└────────────┬──────────────────────┬────────────────────┘
             │ every sample         │ per frame
             ▼                      ▼
     ┌───────────────┐    ┌──────────────────────┐
     │  EKF Predict  │    │  Plane Fitting (ROI)  │
     │  (Euler       │    │  ApproxPlaneFromPoints│
     │   kinematics) │    │  → normal vector vn   │
     └───────┬───────┘    └──────────┬───────────┘
             │                       │
             │            ┌──────────▼───────────┐
             │            │  Plane Classification │
             │            │  floor / wall / skip  │
             │            └──────────┬───────────┘
             │                       │
             └───────────┬───────────┘
                         ▼
               ┌──────────────────┐
               │   EKF Update     │
               │  (floor or wall  │
               │   measurement    │
               │   model)         │
               └────────┬─────────┘
                        ▼
            [φ, θ, ψ, bx, by, bz]
        roll  pitch  yaw  gyro biases
```

***

## EKF Formulation

### State Representation

The filter maintains a Gaussian belief over the platform state, described by a mean **X̂** and covariance **P**.

**3-state attitude-only EKF (Parts C1–C4):**

```
X = [φ, θ, ψ]ᵀ       roll, pitch, yaw (rad)
```

**6-state augmented EKF with bias (Part C5 / Part D):**

```
X = [φ, θ, ψ, bx, by, bz]ᵀ       attitude + gyro biases (rad, rad/s)
```

***

### Process Model & Prediction

The process model is the **Euler kinematic equation** mapping body-frame angular rates to attitude rates:

```
ḟ(X, u) = [ ωx + (ωy·sinφ + ωz·cosφ)·tanθ  ]
           [      ωy·cosφ − ωz·sinφ           ]
           [ (ωy·sinφ + ωz·cosφ) / cosθ       ]
```

Euler-discretised with timestep T:

```
X̂(k+1|k) = X̂(k|k) + T · f(X̂(k|k), u(k))
P(k+1|k)  = Jx · P(k|k) · Jxᵀ + Q
```

The **state Jacobian** `Jx = ∂f/∂X` linearises the nonlinear kinematics at the current estimate. The **process noise covariance** is mapped from raw gyroscope noise through the input Jacobian `Ju = ∂f/∂u`:

```
Q = Ju · Qu · Juᵀ,    Qu = σω²·I₃,    σω = 2.5 × π/180 rad/s
```

This ensures Q is state-dependent and recomputed at every prediction step, accurately reflecting how sensor noise projects into attitude uncertainty at the current orientation.

**Initial covariance** (independent 1σ bounds):

| Angle | Uncertainty |
|-------|-------------|
| Roll φ | ±5° |
| Pitch θ | ±5° |
| Yaw ψ | ±10° |

```
P(0|0) ≈ diag(7.62×10⁻³, 7.62×10⁻³, 3.05×10⁻²)  [rad²]
```

***

### Measurement Model & Update

Planar surfaces detected in the depth stream provide an aperiodic measurement of the platform's orientation relative to global reference planes. The expected platform-frame normal of a global surface is:

```
h(X) = Rᵀ(φ,θ,ψ) · n_global
```

where `R = Rz(ψ)·Ry(θ)·Rx(φ)` is the ZYX rotation matrix.

#### Floor Observation (`n_global = [0, 0, 1]ᵀ`)

```
h(X) = [ −sinθ,  cosθ·sinφ,  cosθ·cosφ ]ᵀ
```

**Observability:** ∂h/∂ψ = **0** — the floor normal is invariant to yaw rotation about the vertical axis. Floor updates correct **roll and pitch only**.

#### Wall Observation (`n_global = [−1, 0, 0]ᵀ`)

```
h(X) = [ −cosθ·cosψ,
         −(sinφ·sinθ·cosψ − cosφ·sinψ),
         −(cosφ·sinθ·cosψ + sinφ·sinψ) ]ᵀ
```

**Observability:** ∂h/∂ψ ≠ **0** — the wall normal has heading-dependent components. Wall updates correct **roll, pitch, and yaw**.

#### Update Equations (standard EKF)

```
S = H · P(k+1|k) · Hᵀ + R
K = P(k+1|k) · Hᵀ · S⁻¹
X̂(k+1|k+1) = X̂(k+1|k) + K · (z − h(X̂(k+1|k)))
P(k+1|k+1) = (I − K·H) · P(k+1|k)
```

Measurement noise: `R = 0.0025 · I₃` (σ = 0.05 per normal component)

***

### Augmented State — Bias Estimation

The 6-state EKF augments attitude with bias states, correcting gyroscope inputs online:

```
ω_corrected = ω_raw − b
```

**Bias process model:** biases are modelled as constants (`b(k+1) = b(k)`), with zero process noise in the bias block:

```
Q_aug = [ Ju·Qu·Juᵀ   0₃ₓ₃ ]    (6×6)
        [   0₃ₓ₃     0₃ₓ₃ ]
```

**Augmented measurement Jacobian:**

```
H_aug = [ H_wall  |  0₃ₓ₃ ]    (3×6)
```

The zero block reflects that wall normals do not depend on bias directly — biases are inferred indirectly through the off-diagonal covariance coupling between attitude and bias states as corrections propagate over time.

**Initial bias uncertainty:** σ_b = 2 deg/s (each axis bounded by ±2 deg/s, treated as 1σ)

***

## Gyroscope Bias Pre-Estimator

Before the EKF runs, a lightweight **least-squares bias estimator** bootstraps an initial bias estimate from sparse attitude snapshots:

- Integrates raw gyroscope data between known attitude samples
- Forms: `Δattitude_measured = Δattitude_integrated − Δt · b`
- Solves for `b` via least squares across all available intervals
- Robust to missing (`NaN`) attitude entries — invalid intervals are skipped

**Accuracy:** all three axes estimated within **0.15 deg/s** of injected ground-truth bias across two test modes (complete and sparse attitude data).

***

## Implementation Details

The core EKF is implemented as two functions in `P2PartD_Main.m`:

```matlab
% Prediction — called at every IMU sample
[Xnew, Pnew] = myPredictionStep(X, P, dt, w, varW, ApiRef)

% Update — called when a valid planar surface is detected
[Xnew, Pnew, okUpdate] = myUpdateStep(X, P, vn, stdC)
```

**Plane classification** inside `myUpdateStep` uses a 35° cone tolerance around the vertical axis to robustly separate floor/ceiling from wall observations, using the current attitude estimate to disambiguate wall identity when necessary.

**Singularity protection:** the `cosθ` denominator in the Euler equations is clamped to a minimum magnitude of `1×10⁻⁴` to prevent division by zero near ±90° pitch (gimbal lock).

**Validation approach:** the filter output is compared in real time against a reference API implementation (`xAPI4010v06`), with live oscilloscopes for attitude and bias, and full-history figures for error analysis.

***

## Getting Started

### Requirements

- MATLAB R2021b or later
- Course validation API (`xAPI4010v06.p`) — provided separately

### Run the bias pre-estimator

```matlab
P2PartA_Main()
% Runs both complete and missing-data test modes
% Prints estimated vs ground-truth biases and pass/fail
```

### Run the full augmented EKF

```matlab
P2PartD_Main()
% Loads sensor dataset, injects [-1, +1.5, -1] deg/s artificial bias
% Opens live attitude + bias oscilloscopes
% Use On/Off button to start, Go to t0 to reset, END to quit
```
