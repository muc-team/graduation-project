# Shared Map & Multi-Robot Localization — Detailed Roadmap

## Current State

| | Robot 1 (Alpha) | Robot 2 (Beta) |
|---|---|---|
| **Sensors** | LiDAR (RPLiDAR A1) | 4× wheel encoders, camera, IMU (GY-87) |
| **Localization** | SLAM Toolbox → accurate pose | Encoder Odom + IMU Fusion (EKF) |
| **Map** | Builds and publishes `/map` | No map awareness |
| **Connection** | ROS 2 + rosbridge on `robot.local` | ROS 2 + rosbridge on `robot2.local` |
| **IMU** | None | GY-87 10-DOF (Connected to Arduino Mega) |

## IMU (GY-87) Connection Details

The **GY-87 10-DOF module** (includes MPU6050, HMC5883L, and BMP180) communicates via **I2C**.

### Wiring: GY-87 to Arduino Mega 2560

| GY-87 Pin | Arduino Mega 2560 Pin | Note |
| :--- | :--- | :--- |
| **VCC** | 5V | Power supply |
| **GND** | GND | Ground |
| **SDA** | 20 (SDA) | I2C Data |
| **SCL** | 21 (SCL) | I2C Clock |

> [!NOTE]
> The Arduino Mega 2560 has dedicated I2C pins at 20 (SDA) and 21 (SCL). Do not use the analog pins A4/A5 which are used for I2C on the Arduino Uno/Nano.

## Goal

A **single shared map** displayed on the dashboard, showing **both robots** with accurate, real-time positions.

---

## The Honest Assessment

### What works well
- Robot 1 on the map: **Excellent**. LiDAR + SLAM gives centimeter-level accuracy with continuous self-correction. This is already working.

### The hard problem: Robot 2
Robot 2 has **no absolute position sensor**. Encoders give you *relative* movement (odometry), which **drifts over time**. Here's what that means practically:

| Approach | Accuracy after 1m | After 10m | After 5 min of driving |
|---|---|---|---|
| Encoders only | ±5 cm | ±30-50 cm | ±1-2 meters |
| Encoders + IMU (EKF) | ±3 cm | ±15-25 cm | ±50 cm - 1 m |
| Encoders + IMU + visual corrections | ±3 cm | ±10 cm | ±15-20 cm |

> [!WARNING]
> **Encoder-only odometry has unbounded drift.** No matter how good the calibration, errors accumulate with every wheel rotation. The IMU significantly helps with rotation accuracy but does NOT fix translational drift. There is no software fix for this — it's a physics limitation.

### Recommendation
**Add the IMU.** Going from encoders-only to encoders+IMU is the single biggest accuracy improvement you can get without adding expensive hardware. It takes the rotational drift from "terrible" to "acceptable" for a demo.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    PC (Dashboard)                           │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐    │
│  │              SHARED MAP RENDERER                     │    │
│  │                                                      │    │
│  │   Subscribe to Robot 1:                              │    │
│  │     /map              → draw occupancy grid          │    │
│  │     /odom             → draw Robot 1 (green)         │    │
│  │     /scan             → draw laser overlay           │    │
│  │                                                      │    │
│  │   Subscribe to Robot 2:                              │    │
│  │     /robot2/odom      → draw Robot 2 (orange)        │    │
│  │                                                      │    │
│  │   Both robots rendered on ONE map canvas             │    │
│  └─────────────────────────────────────────────────────┘    │
│         ▲ WebSocket                    ▲ WebSocket          │
│         │ (port 9090)                  │ (port 9090)        │
└─────────┼──────────────────────────────┼────────────────────┘
          │                              │
   ┌──────┴──────┐               ┌──────┴──────┐
   │  Robot 1 Pi │               │  Robot 2 Pi │
   │  (Alpha)    │               │  (Beta)     │
   │             │               │             │
   │  LiDAR ──→ SLAM ──→ /map   │  Encoders ──┐
   │             ├──→ /odom      │  IMU ───────┤
   │             ├──→ /scan      │         EKF ──→ /robot2/odom
   │             │               │             │
   │  Motors     │               │  Motors     │
   │  rosbridge  │               │  Camera     │
   └─────────────┘               │  rosbridge  │
                                 └─────────────┘
```

### Key Design Decision: Two separate rosbridge connections

The dashboard already connects to whichever robot is "active." For shared map, we need to connect to **both simultaneously**:
- Robot 1's rosbridge → for `/map`, `/odom`, `/scan`  
- Robot 2's rosbridge → for `/robot2/odom`

### Coordinate Frame Alignment

> [!IMPORTANT]
> Both robots must share the **same coordinate origin**. Since Robot 1 builds the map, its SLAM origin (where it started) becomes the reference frame. Robot 2 must know its **starting position relative to Robot 1's map origin**.

Options:
1. **Manual calibration**: Place Robot 2 at a known position on Robot 1's map, enter the offset in config. Simple, works for demos.
2. **Same starting point**: Both robots start at the same physical location. Robot 2's (0,0) = Robot 1's (0,0). Simplest approach.
3. **ArUco marker**: Robot 2 uses its camera to detect a marker at a known map position for periodic corrections. Most robust, but more complex.

**Recommendation for graduation project**: Option 2 (same starting point). It's simple and effective for demos.

---

## Implementation Phases

### Phase 1: Encoder Odometry Node for Robot 2
**Effort**: ~2 hours | **Risk**: Low

Create a ROS 2 node on Robot 2's Pi that converts raw encoder ticks into proper `/robot2/odom` (`nav_msgs/Odometry`) messages.

**Requires knowing:**
- Wheel diameter (measure in meters)
- Encoder CPR (counts per revolution) — check your encoder datasheet
- Wheelbase width (distance between left and right wheels, in meters)

**What it does:**
- Subscribes to `/encoders` (the Int32MultiArray we just set up)
- Computes differential drive kinematics: `Δx, Δy, Δθ`
- Publishes `/robot2/odom` with position and orientation
- Publishes TF transform `odom → base_link`

**Deliverable:** Robot 2 publishes a proper odometry topic that any ROS tool can consume.

### Phase 2: Add IMU + EKF Fusion (Recommended)
**Effort**: ~3-4 hours | **Risk**: Medium

Wire the GY-87 10-DOF IMU to your Arduino Mega 2560 (I2C). Create a node that publishes `/robot2/imu` (`sensor_msgs/Imu`) to the Raspberry Pi.

Use the `robot_localization` package's Extended Kalman Filter to fuse:
- Encoder odometry → position + velocity
- IMU → orientation + angular velocity

**Why this matters:**
- Encoders are terrible at measuring rotation (wheel slip during turns)
- The gyroscope in the IMU measures rotation directly and accurately
- The EKF combines both: encoders for distance, IMU for heading
- Result: ~2-3× better accuracy than encoders alone

**Deliverable:** A fused `/robot2/odom` that is significantly more accurate than encoders alone.

### Phase 3: Dashboard Multi-Robot Map
**Effort**: ~3 hours | **Risk**: Low

Modify `dash_mapping.py` to:
1. Maintain a **second rosbridge connection** to Robot 2 simultaneously
2. Subscribe to Robot 2's `/robot2/odom`
3. Render Robot 2 as an **orange marker** on Robot 1's map (alongside Robot 1's green marker)
4. Show both robots' positions, headings, and trails

**Deliverable:** Single map view showing both robots in real-time.

### Phase 4: Clean Up Robot 1 (Remove Camera)
**Effort**: ~15 min | **Risk**: None

Update `start_robot1.sh` to ensure no camera service starts. Already done in the script we created — just verify on the actual Pi.

---

## Potential Drawbacks & Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| **Robot 2 odometry drift** | Position drifts 30-50 cm per 10m traveled | Add IMU (Phase 2) to reduce to ~15 cm |
| **WiFi latency** | Position updates may lag 100-300ms | Use CONFLATE on ZMQ, keep rosbridge lightweight |
| **Coordinate misalignment** | Robots appear in wrong positions on map | Use same starting point (calibrated origin) |
| **Two rosbridge connections** | Dashboard complexity increases | Clean separation with namespaced topics |
| **IMU calibration** | Bad IMU calibration = worse than no IMU | Calibrate magnetometer, use only gyro+accel initially |
| **Encoder resolution** | Low CPR encoders = coarse odometry | Measure actual CPR and wheel diameter precisely |

---

## What I Need From You Before Starting

1. **Wheel diameter** — measure one wheel in millimeters
2. **Wheelbase** — distance between left wheel center and right wheel center (mm)
3. **Encoder CPR** — counts per full wheel revolution (spin wheel by hand and read the encoder count)
4. **Starting position** — will both robots always start from the same spot?

---

## Suggested Order of Execution

```
Phase 1 (Encoder Odom)     ██████████░░░░░░░░░░  ~2 hrs
Phase 4 (Cleanup)          ██░░░░░░░░░░░░░░░░░░  ~15 min
Phase 3 (Dashboard Map)    ████████░░░░░░░░░░░░  ~3 hrs
Phase 2 (IMU + EKF)        ████████████░░░░░░░░  ~3-4 hrs
                           ─────────────────────
                           Total: ~8-9 hours
```

> [!TIP]
> I recommend doing Phase 1 → Phase 3 first so you can **see both robots on the map immediately** (even with drifty encoder-only odometry). Then add the IMU in Phase 2 to improve accuracy. This gives you a working demo at every stage.
