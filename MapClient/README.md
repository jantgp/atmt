# Truck Simulator & Room Mapper

Two cooperating scripts:

- **`truck_sim.py`** — simulates an autonomous truck driving around a room and publishes sensor telemetry over MQTT.
- **`room_mapper.py`** — subscribes to the telemetry, dead-reckons the truck position, builds a live occupancy grid, and estimates the room boundary polygon.

---

## Quick start

```bash
# Terminal 1 — run the simulator
python truck_sim.py

# Terminal 2 — run the mapper
python room_mapper.py
```

---

## truck_sim.py

Simulates a truck exploring a room using an Ackermann bicycle-model physics engine. Publishes sensor telemetry to `TRUCK_SIM/telemetry` at 2 Hz.

### Usage

```bash
python truck_sim.py [OPTIONS]
```

### Options

| Option | Default | Description |
|---|---|---|
| `--room` | `l_shape` | Room geometry preset (see below) |
| `--strategy` | `embedded` | Exploration strategy (see below) |
| `--speed` | `80` | Forward motor PWM % (0–100) |
| `--hz` | `10` | Simulation tick rate (ticks/second) |
| `--broker` | `192.168.2.2` | MQTT broker host |
| `--port` | `1883` | MQTT broker port |
| `--no-mqtt` | off | Disable MQTT publishing (visualizer only) |
| `--list-rooms` | — | Print available room presets and exit |

### Room presets

| Name | Shape | Dimensions |
|---|---|---|
| `rectangle` | Rectangle | 240 × 120 cm |
| `square` | Square | 250 × 250 cm |
| `l_shape` | L-shape | 120 × 120 cm, wing 60 × 60 cm |
| `donut` | Rectangular ring | 300 × 300 cm outer, 120 × 120 cm hole |
| `circular_donut` | Circular ring | outer radius 150 cm, inner radius 60 cm |

### Exploration strategies

| Name | Behaviour |
|---|---|
| `embedded` | DRIVE → RECOVER state machine mirroring the C++ firmware. Drives forward until obstacle at < 20 cm, then reverses with full steering toward the open side for 1.5 s. |
| `reactive` | Goes straight, stops and steers toward the side with more clearance when front sensor < 20 cm, resumes when front clears 30 cm. |
| `wall_right` | Proportional controller that maintains ~25 cm from the right wall. Steers right to find a wall, turns left to avoid front obstacles. |

### Examples

```bash
python truck_sim.py --room rectangle --strategy reactive
python truck_sim.py --room donut --strategy wall_right
python truck_sim.py --room l_shape --speed 60 --hz 20
python truck_sim.py --no-mqtt --room square          # no MQTT, visualizer only
python truck_sim.py --list-rooms                     # print room sizes and exit
```

### MQTT telemetry payload

Published to `TRUCK_SIM/telemetry` at 2 Hz:

```json
{
  "truck_id": "TRUCK_SIM",
  "seq": 42,
  "t_ms": 21000,
  "mode": "EXPLORE",
  "ul": 55.2,   "ur": 120.4,  "uf": 18.7,  "ub": 201.0,
  "yaw_rate": 1.23,
  "gy_x": 0.01, "gy_y": -0.02, "gy_z": 1.23,
  "heading": 92.4,
  "compass": 91.8,
  "mag_x": 29.8, "mag_y": 1.2,  "mag_z": 40.1,
  "acc_x": 0.01, "acc_y": 0.00, "acc_z": 9.81,
  "width": 175.6,
  "center_error": 65.2,
  "front_blocked": false,
  "cmd_pwm": 80,
  "cmd_steer": "STRAIGHT"
}
```

---

## room_mapper.py

Subscribes to `TRUCK_SIM/telemetry`, estimates the truck position via dead reckoning, updates a 2-D occupancy grid from ultrasonic rays, and overlays an estimated room boundary polygon on a live map.

### Usage

```bash
python room_mapper.py [OPTIONS]
```

### Options

| Option | Default | Description |
|---|---|---|
| `--broker` | `192.168.2.2` | MQTT broker host |
| `--port` | `1883` | MQTT broker port |
| `--size` | `1000` | Grid side length in cm (10 m × 10 m) |
| `--resolution` | `5` | Cell size in cm — smaller = more detail, more memory |
| `--speed` | `70.0` | Nominal forward speed for dead reckoning (cm/s) |
| `--recover-speed` | `15.0` | Net speed during recovery/turning (cm/s) |
| `--gy-thresh` | `15.0` | Yaw-rate threshold to detect recovery state (deg/s) |
| `--rdp` | `15.0` | RDP polygon simplification tolerance (cm) |

### Auto-calibration

On startup the mapper collects **40 telemetry messages** before beginning to map. During this warmup it automatically calibrates:

| Parameter | Method |
|---|---|
| `gy_threshold` | Otsu's method on the `\|gy_z\|` histogram — finds the valley between DRIVE (low yaw rate) and RECOVER (high yaw rate) modes |
| `forward_speed` | Median of `-Δuf/Δt` measured during straight DRIVE segments where the front sensor is decreasing |
| `recover_speed` | Fixed at 20 % of `forward_speed` |

The values passed via `--speed` and `--gy-thresh` are used as starting defaults before calibration completes.

### Dead reckoning

Position is estimated using a **complementary filter** on heading:

```
heading = 0.95 × (heading_prev + gy_z × dt) + 0.05 × compass
```

- Gyroscope (`gy_z`) tracks fast heading changes without noise jumps.
- Compass corrects slow gyro drift (5 % weight per tick).

Speed state is inferred from `|gy_z|`:
- `|gy_z| ≤ gy_threshold` → **DRIVE** at `forward_speed`
- `|gy_z| > gy_threshold` → **RECOVER** at `recover_speed` (reversing)

### Occupancy grid

The grid is centred on the truck's starting position (origin = 0, 0).

| Cell value | Meaning |
|---|---|
| `0` (grey) | Unknown — not yet observed |
| `< 0` (light grey) | Free space — ray passed through |
| `> 0` (orange) | Occupied — wall hit detected |

Each ultrasonic reading (front, back, left, right) casts a ray from the estimated truck position. Intermediate cells are marked free; the endpoint cell is marked occupied if `dist < 280 cm`.

### Boundary estimation

Every 2 seconds the mapper estimates the room polygon:

1. Threshold occupied cells (log-odds > 0)
2. Binary dilation (3 iterations) to bridge gaps between sparse hits
3. Fill interior holes
4. Trace outer contour (Moore-neighbourhood border following)
5. RDP simplification to the `--rdp` tolerance
6. Overlay as a green polygon on the map

### Examples

```bash
python room_mapper.py                              # defaults
python room_mapper.py --resolution 10             # coarser grid, faster
python room_mapper.py --size 500                  # 5 m × 5 m grid
python room_mapper.py --speed 80 --gy-thresh 20   # manual tuning
python room_mapper.py --rdp 5                     # tighter boundary polygon
```

---

## Architecture

```
truck_sim.py  ──MQTT──►  room_mapper.py
   │                          │
   │  TRUCK_SIM/telemetry     │  OccupancyGrid  (log-odds cells)
   │                          │  DeadReckoning  (compass + gy_z)
   └── TruckVisualizer        │  BoundaryEstimator (RDP polygon)
       (matplotlib, live)     └── MapVisualizer (matplotlib, live)
```

## Dependencies

```bash
pip install paho-mqtt numpy scipy matplotlib
```
