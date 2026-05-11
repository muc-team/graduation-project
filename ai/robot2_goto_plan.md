# Robot 2 — Click-to-Navigate Implementation Plan

## What You Have Right Now

| Component | Robot 1 (Alpha) | Robot 2 (Beta) |
|-----------|-----------------|----------------|
| Lidar | ✅ Yes | ❌ No |
| IMU | ❌ No | ✅ GY-87 (MPU6050 + HMC5883L + BMP180) |
| Encoders | ❌ No | ✅ 4× Quadrature |
| SLAM Map | ✅ `/map` topic | ❌ No map of its own |
| Odometry (`/odom`) | ✅ From SLAM | ❌ **Missing — must add** |
| Navigation | ✅ `simple_explorer.py` (lidar-based) | ❌ **Missing — must add** |

## What We Need to Build (4 Layers)

```mermaid
graph TD
    A[Arduino Mega<br>Encoders + IMU @ 50Hz] -->|Serial USB| B[robot2_bridge.py<br>Parse D: packets]
    B -->|/encoders| C[robot2_odom.py<br>Dead-Reckoning]
    B -->|/imu/data_raw| C
    C -->|/robot2/odom| D[Dashboard<br>dash.py]
    C -->|/robot2/odom| E[robot2_goto.py<br>GoTo Navigator]
    D -->|Click on map| F[/robot2/goal_pose]
    F --> E
    E -->|/cmd_vel| B
    B -->|Serial F/B/L/R/S| A
```

---

## Layer 1: Odometry Node — `robot2_odom.py`

> [!IMPORTANT]
> This is the **most critical** piece. Without it, Robot 2 has no idea where it is.

**What it does:** Converts raw encoder ticks + IMU gyro into an `(x, y, θ)` position that gets published as `/robot2/odom`.

**Algorithm — Differential Drive Dead-Reckoning:**
```
Every 20ms (50 Hz):
  1. Read encoder deltas: Δleft, Δright (average of front+rear per side)
  2. Convert ticks → meters: Δleft_m = Δleft * (π * wheel_diameter) / ticks_per_revolution
  3. Compute distance & rotation:
     - distance = (Δleft_m + Δright_m) / 2
     - Δθ_encoders = (Δright_m - Δleft_m) / wheel_base
  4. Fuse with IMU gyro for better heading:
     - Δθ = α * Δθ_encoders + (1-α) * gyro_z * dt   (α ≈ 0.3)
  5. Update pose:
     - x += distance * cos(θ)
     - y += distance * sin(θ)
     - θ += Δθ
  6. Publish nav_msgs/Odometry on /robot2/odom
```

**File:** `navigation/robot2_odom.py`

**You need to measure:**
- `WHEEL_DIAMETER` — in meters (e.g., 0.065 for 65mm wheels)
- `TICKS_PER_REV` — spin one wheel 1 full turn, read encoder count
- `WHEEL_BASE` — distance between left and right wheel centers (in meters)

---

## Layer 2: GoTo Navigator — `robot2_goto.py`

**What it does:** Receives a goal `(x, y)` coordinate, computes the bearing & distance from current position, drives the robot there.

**Algorithm — Rotate-then-Drive:**
```
On new goal:
  1. Compute angle_to_goal = atan2(goal_y - robot_y, goal_x - robot_x)
  2. Compute angle_error = angle_to_goal - robot_θ  (normalize to [-π, π])
  3. Compute distance = sqrt((goal_x - x)² + (goal_y - y)²)

  Phase 1 — ROTATE: if |angle_error| > 0.15 rad (~8°)
    → publish Twist(angular.z = Kp_angle * angle_error)
    → clamp to ±0.4 rad/s

  Phase 2 — DRIVE: if distance > 0.10 m
    → publish Twist(linear.x = Kp_dist * distance, angular.z = Kp_angle * angle_error)
    → clamp linear to 0.15 m/s max

  Phase 3 — ARRIVED: if distance < 0.10 m
    → publish Twist(0, 0) — stop
    → publish "ARRIVED" status
```

**File:** `navigation/robot2_goto.py`

> [!WARNING]
> **No obstacle avoidance!** Robot 2 has no Lidar. It will drive in a straight line toward the goal. This is fine for open areas but will hit walls in corridors. This is a known limitation.

---

## Layer 3: Dashboard Changes — `dash.py`

### 3a. Click-on-Map → Publish Goal

The map image needs a click handler that:
1. Converts pixel `(click_x, click_y)` → world `(world_x, world_y)` using the map's resolution and origin
2. Publishes a ROS message to `/robot2/goal_pose`

```python
# Pixel → World conversion:
world_x = origin_x + (click_x / scale) * resolution
world_y = origin_y + ((h - click_y / scale)) * resolution  # flip Y
```

### 3b. Show Robot 2 on the Map

Subscribe to `/robot2/odom` (just like Robot 1's `/odom`) and draw a **blue** dot for Robot 2 alongside the green dot for Robot 1.

### 3c. Show Goal Marker

Draw a red crosshair / target icon at the clicked goal location on the map.

### 3d. Show Navigation Status

Display a small label: "Robot 2: Navigating..." / "Robot 2: Arrived!" / "Robot 2: Idle"

---

## Layer 4: Startup Script Update — `robot2.sh`

Add the new nodes to the tmux session:

```bash
# 4. ODOMETRY (Fuses encoders + IMU → /robot2/odom)
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "$ROS_SOURCE; cd $DIR/../navigation && python3 robot2_odom.py; exec bash" C-m

# 5. GOTO NAVIGATOR (Listens for /robot2/goal_pose → drives to it)
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "$ROS_SOURCE; cd $DIR/../navigation && python3 robot2_goto.py; exec bash" C-m
```

---

## Implementation Order

| Step | File | What | Difficulty |
|------|------|------|-----------|
| **1** | `navigation/robot2_odom.py` | Dead-reckoning odometry node | ⭐⭐⭐ Medium |
| **2** | `navigation/robot2_goto.py` | GoTo navigator node | ⭐⭐ Easy |
| **3** | `dashboard/dash.py` | Map click handler + Robot 2 overlay | ⭐⭐⭐ Medium |
| **4** | `rasp_cmd/robot2.sh` | Add new nodes to startup | ⭐ Easy |

> [!NOTE]
> **Before coding**, you must measure three physical values from Robot 2:
> 1. **Wheel diameter** (in meters)
> 2. **Ticks per revolution** (spin wheel 1 full turn, read encoder count from serial monitor)
> 3. **Wheel base** (center-to-center distance between left and right wheels, in meters)
>
> These are needed for the odometry math to convert encoder ticks into real-world meters.

---

## Limitations (Be Honest About These)

| Limitation | Why | Workaround |
|-----------|-----|------------|
| No obstacle avoidance | No Lidar on Robot 2 | Only navigate in areas already mapped by Robot 1 |
| Drift over time | Dead-reckoning accumulates error | Magnetometer heading correction helps; reset encoders at known positions |
| No map of its own | Can't do SLAM | Uses Robot 1's `/map` as the shared background |
| Wheel slip | Smooth floors or carpet transitions | Keep speed low (≤ 0.15 m/s) |

---

## Summary

The full flow when you click the map:

1. **You click** a spot on the SLAM map in the dashboard
2. **Dashboard** converts pixel → world coordinates → publishes to `/robot2/goal_pose`
3. **`robot2_goto.py`** receives the goal, reads `/robot2/odom`, computes bearing → publishes `/cmd_vel`
4. **`robot2_bridge.py`** receives `/cmd_vel` → converts to `F`/`B`/`L`/`R` → sends to Arduino
5. **Arduino** drives motors → encoders + IMU stream back at 50 Hz
6. **`robot2_bridge.py`** parses data → publishes `/encoders` + `/imu/data_raw`
7. **`robot2_odom.py`** fuses them → publishes `/robot2/odom`
8. **Dashboard** reads `/robot2/odom` → draws blue dot on map moving toward the goal

**Ready to start building? Tell me your wheel measurements and I'll create all 3 files.**
