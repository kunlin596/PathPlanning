<div align="center">

# CarND Path Planning

**Frenet-frame, jerk-minimized trajectory planning for highway driving**

<sub>Udacity Self-Driving Car Engineer Nanodegree — Term 3</sub>

[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-3.16%2B-064F8C?logo=cmake&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)
[![Demo](https://img.shields.io/badge/demo-YouTube-FF0000?logo=youtube&logoColor=white)](https://youtu.be/_f_T_y8JZSE)

<img src="./images/sample2.png" width="46%" alt="Lane changing" />
<img src="./images/full_cycle.png" width="46%" alt="Full cycle, 4.25 miles" />

<sub>Lane changing (left) · one full 4.25-mile loop (right)</sub>

</div>

---

## Overview

A real-time planner that safely drives a simulated highway: it cruises near the
50 MPH limit, follows and overtakes slower traffic, and changes lanes — while
keeping total acceleration under 10 m/s² and jerk under 10 m/s³.

Trajectories are generated in **Frenet coordinates** as jerk-minimized quintic
polynomials ([Werling et al., 2010](#references)), scored for safety and comfort,
and checked for collisions against tracked vehicles. The car completes the
6 945.554 m loop in a little over five minutes.

```
sensor fusion ─► tracker ─► trajectory generation (PTG) ─► collision check ─► waypoints ─► simulator
```

📄 Full write-up: [`writeup.pdf`](./writeup.pdf)

## Highlights

- **Frenet-frame planning** — decoupled longitudinal/lateral quintic (JMT) trajectories
- **Behaviors** — lane keeping, lane changing, and vehicle following
- **Collision checking** against the predicted paths of tracked vehicles
- **Cost-based selection** balancing speed, safety, and comfort
- **OpenMP-parallelized** trajectory sampling
- **uWebSockets** server driving the Udacity Term3 simulator

## Build

### Docker (recommended)

The quickest reproducible build. See the [install guide](https://docs.docker.com/engine/install/).

```shell
docker build -t pathplanningserver:latest .
docker run -p 4567:4567 pathplanningserver:latest
```

### Local build

Most dependencies come from your system package manager. **GoogleTest** and
**Eigen** are fetched automatically at configure time if missing, so the only
dependency you build by hand is **uWebSockets**.

**Debian / Ubuntu dependencies**

```shell
sudo apt-get install \
  build-essential cmake \
  python3-dev pybind11-dev \
  libfmt-dev libspdlog-dev libboost-all-dev \
  libeigen3-dev libssl-dev libz-dev libuv1-dev
```

**uWebSockets** — the server uses the legacy API, so build the pinned commit:

```shell
git clone https://github.com/uWebSockets/uWebSockets
cd uWebSockets && git checkout e94b6e1
cmake -S . -B build && cmake --build build -j && sudo cmake --install build
sudo ln -sf /usr/lib64/libuWS.so /usr/lib/libuWS.so   # so the linker finds it
```

> [!NOTE]
> If uWebSockets / libuv are absent, the libraries and unit tests still build —
> only the `pathplanningserver` executable is skipped.

Then configure and build with the presets:

```shell
git clone https://github.com/kunlin596/PathPlanning
cd PathPlanning
cmake --preset default
cmake --build --preset default
```

<sub>Bundled single-header libraries: [spline](http://kluge.in-chemnitz.de/opensource/spline/) · [json](https://github.com/nlohmann/json)</sub>

## Run

Launch the simulator and the planning server side by side:

```shell
./build/pathplanningserver --conf default_conf.json -m data/highway_map.csv --loglevel=info
```

Download the simulator from the
[Term3 releases](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2),
then:

```shell
chmod u+x term3_sim.x86_64
./term3_sim.x86_64
```

## Testing

The unit tests (`test_math`, `test_jmt`) build alongside the libraries and do
not require uWebSockets:

```shell
ctest --preset default
```

## Development

The Python utilities live in `python/pathplanning`. Install them in editable
mode and point `MAP_DATA` at the waypoint file:

```shell
pip install -e .
export MAP_DATA=$PWD/data/highway_map.csv
```

Python is linted and formatted with [ruff](https://docs.astral.sh/ruff/); C++ is
formatted with `clang-format` (a slightly modified `Mozilla` style — see
[`.clang-format`](.clang-format)). Both run via [pre-commit](https://pre-commit.com/),
which also enforces Conventional Commit messages with a mandatory scope
(e.g. `feat(ptg): ...`):

```shell
pre-commit install      # installs the pre-commit and commit-msg hooks
pre-commit run --all-files
```

## Simulator protocol

<details>
<summary><b>Behavior contract</b></summary>

1. The ego uses a perfect controller and visits every `(x, y)` point it receives
   every **0.02 s**.
2. Point units are meters; spacing between points sets the speed.
3. The vector from one point to the next sets the car's heading.
4. Tangential and normal acceleration are measured, along with jerk (the rate of
   change of total acceleration).
5. Returned paths must keep total acceleration under 10 m/s² and jerk under
   50 m/s³.
6. There is 1–3 time steps of latency; the simulator keeps using the last points
   it was given. Use `previous_path_x` / `previous_path_y` to extend the prior
   path smoothly.

</details>

<details>
<summary><b>Map data</b> — <code>data/highway_map.csv</code></summary>

Each waypoint is `[x, y, s, dx, dy]`: `x`/`y` are the map position, `s` is the
distance along the road (meters), and `(dx, dy)` is the unit normal pointing
outward from the loop. The Frenet `s` value wraps from 0 to 6 945.554.

</details>

<details>
<summary><b>Telemetry from the simulator</b></summary>

**Ego localization** (noise-free):

```
x, y      car position in map coordinates
s, d      car position in Frenet coordinates
yaw       heading in the map (degrees)
speed     speed (MPH)
```

**Previous path** (waypoints not yet consumed):

```
previous_path_x, previous_path_y   last points handed to the simulator
end_path_s, end_path_d             Frenet position of that path's last point
```

**Sensor fusion** — `sensor_fusion[i]` per tracked car:

```
[ id, x, y, vx, vy, s, d ]
```

</details>

## References

- M. Werling, J. Ziegler, S. Kammel, and S. Thrun, "Optimal trajectory
  generation for dynamic street scenarios in a Frenet frame," *ICRA*, 2010,
  pp. 987–993.

## License

Released under the [MIT License](LICENSE).
