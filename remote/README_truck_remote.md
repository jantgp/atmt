# truck_remote.py — Autonomous Remote Controller

Subscribes to the real truck's telemetry over MQTT, runs an autonomous
exploration strategy, and publishes control commands back to the truck.

---

## Quick start

```bash
python truck_remote.py
```

Connects to the broker at `192.168.2.2:1883`, subscribes to
`38504720f540/telemetry`, and starts the `embedded` strategy.

---

## Usage

```bash
python truck_remote.py [OPTIONS]
```

### Options

| Option | Default | Description |
|---|---|---|
| `--broker` | `192.168.2.2` | MQTT broker host |
| `--port` | `1883` | MQTT broker port |
| `--id` | `38504720f540` | Vehicle ID |
| `--strategy` | `embedded` | Exploration strategy (see below) |

### Examples

```bash
python truck_remote.py                                    # defaults
python truck_remote.py --strategy reactive                # reactive explorer
python truck_remote.py --strategy wall_right              # wall follower
python truck_remote.py --id b4328a0a8ab4                  # other vehicle
python truck_remote.py --broker 10.0.0.1 --port 1883     # custom broker
```

---

## MQTT topics

| Direction | Topic | Description |
|---|---|---|
| Subscribe | `{vehicle_id}/telemetry` | Incoming sensor data from the truck |
| Publish | `{vehicle_id}/control` | Outgoing control commands |

### Control payload

```json
{"motor": 100, "direction": 60}
```

| Field | Range | Description |
|---|---|---|
| `motor` | `-100` … `100` | `100` = full forward, `-100` = full reverse, `0` = stop |
| `direction` | `-100` … `100` | `60` = full left, `-60` = full right, `0` = straight |

---

## Exploration strategies

### `embedded` (default)

DRIVE / RECOVER state machine — mirrors the C++ firmware logic on the truck.

| State | Behaviour | Transition |
|---|---|---|
| **DRIVE** | Full speed forward, steering centred | → RECOVER when `uf < 20 cm` |
| **RECOVER** | Full reverse + full steering toward the open side for **4 seconds** | → DRIVE after 4 s |

The open side is chosen once at recovery entry: steers toward whichever side (`ul` or `ur`) has more clearance.

### `reactive`

Simple bounce-off-walls behaviour.

- Drives forward at full speed.
- When `uf < 20 cm`: stops, steers full lock toward the side with more clearance, reverses.
- When `uf ≥ 30 cm`: straightens up and drives forward again.

### `wall_right`

Keeps the truck's right side near a target wall distance.

- Normal: steers left / right / straight based on `ur` vs target (25 cm).
- Front blocked (`uf < 20 cm`): steers hard left until clear.
- No right wall detected (`ur > 60 cm`): steers hard right to find it.

---

## Telemetry fields used

The script reads these fields from each incoming JSON message:

| Field | Unit | Used for |
|---|---|---|
| `uf`, `ub`, `ul`, `ur` | cm | Obstacle detection and steering decisions |
| `t_ms` | ms | Recovery timer in `embedded` strategy |
| `heading` | ° | Console display |
| `cmd_pwm`, `cmd_steer` | — | Console display |

---

## Known vehicle IDs

| ID | Label |
|---|---|
| `38504720f540` | JCA01 |
| `b4328a0a8ab4` | PAR01 |

---

## Dependencies

```bash
pip install paho-mqtt
```
