# BotLab W24: Autonomous MBot

Software for an **MBot Classic**, a two-wheel differential-drive robot with a 2D LiDAR, an IMU and a Raspberry Pi Pico motor controller. With this code the robot can drive closed-loop, build a map, localize itself with a particle filter, plan collision-free paths with A\*, and explore an unknown maze on its own before returning to where it started.

Built for the University of Michigan BotLab, Winter 2024 (Section PM, Team 5).

| | |
|---|---|
| **Team** | Jayaprakash Harshavardhan · Nithish Kumar · Surya Pratap Singh |
| **Hardware** | MBot Classic with a Jetson Nano / RPi host, RPi Pico (RP2040) control board, 2D LiDAR, BHI160 IMU, and a 3D-printed forklift driven by a Dynamixel XL-320 |
| **Stack** | C/C++, CMake, [LCM](https://lcm-proj.github.io/) messaging, Pico SDK |

---

## Table of contents

- [System overview](#system-overview)
- [Repository layout](#repository-layout)
- [Getting started](#getting-started)
  - [1. Firmware (Pico)](#1-firmware-pico)
  - [2. Autonomy stack (Jetson / RPi)](#2-autonomy-stack-jetson--rpi)
- [Running the robot](#running-the-robot)
- [How it works](#how-it-works)
- [Results](#results)
- [Known issues & future work](#known-issues--future-work)

---

## System overview

```
┌──────────────────────────── Jetson Nano / Raspberry Pi ─────────────────────────────┐
│                                                                                      │
│   exploration ──goal──▶ motion_planning_server ──path──▶ mbot_motion_controller      │
│        ▲                     (A* + obstacle grid)               │                    │
│        │ map / pose                                             │ velocity cmds      │
│        │                                                        ▼                    │
│   mbot_slam  ◀──── LiDAR scans ────  LiDAR driver          LCM ⇄ serial bridge       │
│   (mapping + particle filter)                                   │                    │
│        ▲                                                        │                    │
│        └──────────────── odometry ◀─────────────────────────────┤                    │
└─────────────────────────────────────────────────────────────────┼────────────────────┘
                                                                  │ USB serial
┌─────────────────────────────── RPi Pico (mbot firmware) ────────┴────────────────────┐
│  encoders + IMU ─▶ odometry (gyro-fused heading) ─▶ wheel PID + feed-forward ─▶ PWM │
└──────────────────────────────────────────────────────────────────────────────────────┘
```

All processes on the host talk over **LCM**. The Pico firmware packs the same LCM message types over USB serial (`comms/`), so the host sees odometry, encoder and IMU data as ordinary LCM channels.

## Repository layout

```
.
├── mbot_autonomy-main/        # High-level autonomy (runs on the Jetson / RPi)
│   ├── include/               #   headers: slam/, planning/, mbot/, utils/
│   ├── src/
│   │   ├── mbot/              #   motion controllers (diff + omni), drive_square test
│   │   ├── slam/              #   occupancy mapping, action/sensor models, particle filter
│   │   ├── planning/          #   A*, obstacle distance grid, frontiers, exploration, planner server
│   │   └── utils/             #   getopt, timestamps, geometry helpers
│   ├── services/              #   systemd units for SLAM + motion controller
│   ├── scripts/install.sh     #   build + install + enable services
│   └── lcmlog-2024-05-02.00   #   recorded LCM log from a test run
│
└── mbot_firmware-harsh/       # Low-level firmware (runs on the RPi Pico)
    ├── src/                   #   main loop, odometry, wheel controller
    ├── mbot/                  #   drivers: motor, encoder, IMU (BHI160), FRAM, servo, barometer
    ├── comms/                 #   LCM-over-serial protocol
    ├── rc/                    #   math library: matrices, filters, Kalman, quaternions
    ├── tests/                 #   hardware tests + motor calibration programs
    ├── python/                #   quick drive-test scripts
    └── setup.sh, upload*.sh, debug*.sh
```

Each `src/*/` folder in `mbot_autonomy-main` has its own `README` describing every file in it.

## Getting started

### Prerequisites

| On the host (Jetson / RPi) | For the firmware |
|---|---|
| CMake ≥ 3.1, a C++14 compiler | `arm-none-eabi` GCC toolchain |
| [LCM](https://lcm-proj.github.io/) | Pico SDK (pulled in as a submodule by `setup.sh`) |
| `mbot_lcm_msgs` (MBot message definitions) | `picotool` or `openocd` for flashing |
| GTK2 | |

### 1. Firmware (Pico)

```bash
cd mbot_firmware-harsh
./setup.sh                 # installs deps and inits the pico-sdk submodule (asks for sudo)
mkdir -p build && cd build
cmake ..
make
```

This builds `build/src/mbot.uf2` (the main firmware) and the test and calibration programs under `build/tests/`.

**Calibrate first.** Flash `mbot_calibrate_classic.uf2` and let the robot run on the floor. It measures each motor's polarity and PWM slope/intercept and writes them to FRAM. The main firmware reads these coefficients at boot.

**Flash** using any of these methods:

```bash
# Bootloader: hold BOOTSEL while plugging in, then
picotool load build/src/mbot.uf2 && picotool reboot

# Upload script (needs BTLD/RUN pins wired to the host GPIO)
./upload.sh build/src/mbot.uf2

# SWD via openocd (no bootloader mode needed; uses the .elf)
./upload_swd.sh build/src/mbot.elf
```

In `src/mbot.h`, `OPEN_LOOP` switches between open-loop and PID wheel control, and `MBOT_DRIVE_TYPE` selects differential or omni drive.

For SWD wiring and GDB debugging (`debug.sh` / `debug_attach.sh`), see [`mbot_firmware-harsh/README.md`](mbot_firmware-harsh/README.md).

### 2. Autonomy stack (Jetson / RPi)

```bash
cd mbot_autonomy-main
./scripts/install.sh
```

The script builds everything, installs `mbot_slam` and `mbot_motion_controller` to the system, and enables them as systemd services that start on boot:

| Service | Command |
|---|---|
| `mbot-motion-controller` | `mbot_motion_controller` |
| `mbot-slam` | `mbot_slam --num-particles 200 --map /home/mbot/current.map --listen-for-mode` |

To build without installing:

```bash
mkdir -p build && cd build
cmake .. && make
```

Build targets:

| Binary | Purpose |
|---|---|
| `mbot_motion_controller` | Follows a path of waypoints using a rotate–translate–rotate maneuver controller |
| `mbot_slam` | Occupancy-grid mapping + Monte Carlo localization |
| `motion_planning_server` | Serves A\* path requests against the current map |
| `exploration` | Frontier-based exploration state machine |

## Running the robot

```bash
# Stop the services if you want to run things by hand
sudo systemctl stop mbot-slam mbot-motion-controller

# Full SLAM (map + localize) with 300 particles
./build/mbot_slam --num-particles 300

# Mapping only, e.g. against the recorded log with ground-truth poses
./build/mbot_slam --mapping-only
lcm-logplayer lcmlog-2024-05-02.00

# Localize against a saved map
./build/mbot_slam --localization-only --map current.map

# Planning + autonomous exploration
./build/mbot_motion_controller &
./build/motion_planning_server &
./build/exploration
```

Useful `mbot_slam` flags: `--hit-odds` / `--miss-odds` (log-odds update sizes, default 3 / 2), `--action-only`, `--random-initial-pos`, `--listen-for-mode`. Run `mbot_slam -h` for the full list.

For quick drive tests without the autonomy stack, use the scripts in `mbot_firmware-harsh/python/`, for example `python3 mbot_test_drive.py`.

## How it works

### Motion control & odometry (firmware)

- **Calibration**: each motor's PWM↔velocity mapping is fit as a line (slope + intercept) for each direction and stored in FRAM.
- **Odometry**: wheel encoders give linear velocity `v = R(ω_L − ω_R)/2`. Heading comes from the IMU yaw (`ω_z = (φ_t − φ_{t−1})/T`) rather than from encoder differences, which gives a much cleaner dead-reckoning estimate.
- **Wheel control**: a feed-forward term from calibration plus a per-wheel PID on velocity error, with integral windup reset. Final gains: **K<sub>P</sub> = 1.0, K<sub>I</sub> = 0.0001, K<sub>D</sub> = 0**.
- **Maneuver control**: `diff_motion_controller.cpp` drives waypoint to waypoint with a rotate–translate–rotate strategy.

### SLAM (`src/slam/`)

- **Mapping**: LiDAR rays are traced through the grid with Bresenham's line algorithm. Hit cells gain log-odds and passed-through cells lose them (int8 range −127…127). Scans are motion-corrected with `MovingLaserScan`.
- **Action model**: the odometry-based rotate–translate–rotate model with Gaussian noise. Tuned values are **k₁ = 0.0025** (rotation) and **k₂ = 0.0005** (translation).
- **Sensor model**: each particle is scored by comparing its projected LiDAR endpoints against the occupancy grid.
- **Particle filter**: predict, weight, normalize and resample, then report the weighted-mean pose. Particles can be initialized at a known pose or spread randomly across the map.

### Planning & exploration (`src/planning/`)

- **Obstacle distance grid**: gives each cell its distance to the nearest obstacle, so the planner can keep the robot's radius clear of walls.
- **A\***: an 8-connected grid search with an admissible heuristic that rejects cells closer to obstacles than the robot radius. Paths are pruned to drop redundant waypoints.
- **Exploration**: a state machine that finds frontiers (the boundary between known and unknown space), plans to the nearest reachable one, repeats until none remain, then drives back home.

### Forklift

The front of the robot carries a 3D-printed (PLA, 20% infill, nylon shaft) timing-belt forklift for lifting and stacking crates. A Dynamixel XL-320 drives it through a 3:1 gear ratio and 400 mm belt, and a U-frame clamps it to the chassis.

## Results

**Motor calibration** (mean of 5 runs on concrete)

| Direction | Slope R | Intercept R | Slope L | Intercept L |
|---|---|---|---|---|
| Forward | 0.0594 | 0.0524 | 0.0514 | 0.0594 |
| Reverse | 0.0527 | −0.0677 | 0.0589 | −0.0628 |

Intercepts varied more than slopes across runs. The slope depends mostly on the motor's mechanics, while the intercept tracks static friction and floor conditions.

**SLAM accuracy** (SLAM pose vs. ground truth)

| Metric | Value |
|---|---|
| RMSE | 0.146 m |
| Max absolute error | 0.173 m |

**Particle filter update time**

| Particles | 100 | 500 | 1000 | ~2050 |
|---|---|---|---|---|
| Update time | 12.11 | 26.89 | 47.38 | 97.80 |

About 2050 particles is the most the filter can handle while keeping up with a 10 Hz update rate.

**A\* planning** (`astar_test`, nodes expanded)

| Test | Min | Max | Mean |
|---|---|---|---|
| Maze grid | 3,853 | 6,503 | 4,876 |
| Narrow constriction | 131 | 359,721 | 116,663 |
| Wide constriction | 1,302 | 36,001 | 13,228 |
| Convex grid | 469 | 2,031 | 1,320.5 |

## Known issues & future work

- **Odometry drift**: small heading errors add up, so after exploring the robot comes back close to its start pose but not exactly on it. Better odometry would help most.
- **Low-pass filters**: the velocity low-pass filters caused instability and are currently disabled, so angular velocity readings are still noisy.
- **Uneven wheel start-up**: one wheel sometimes spins up before the other and the robot curves. Per-wheel PID tuning should fix this.
- **Exploration efficiency**: the planner currently takes the first valid frontier. Picking the frontier closest to the robot would cut down on driving.
- **Forklift**: the fastener slots in the 3D-printed parts wore out and weakened the assembly.

## Acknowledgements

Built on the University of Michigan [MBot](https://mbot.robotics.umich.edu/) platform. The starter code for `mbot_autonomy` and `mbot_firmware` comes from the MBot project, and the firmware math library under `rc/` is adapted from the Robot Control Library.
