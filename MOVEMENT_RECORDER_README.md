# Movement Recorder - Robot Training System

## Overview

The Movement Recorder is a new feature that allows you to record and replay movement sequences for your rescue robot **without requiring functional sensors**. This is perfect for:

- Training the robot with predefined paths while debugging sensors
- Creating competition strategies that run independently of sensor input
- Testing motor functionality in isolation
- Building a library of reusable movement patterns

## Features

✅ **Sensor-independent**: Only uses MotorController (no ultrasonic or IMU required)
✅ **Interactive recording**: Step-by-step wizard to build sequences
✅ **JSON storage**: Portable, human-readable movement files
✅ **Test before save**: Preview each step as you record
✅ **Multiple sequences**: Save and manage a library of paths

## How to Access

### Method 1: From Main Menu
```bash
python3 main.py
# Select option 5: Movement Recorder
```

### Method 2: Direct Command
```bash
python3 main.py --mode movement
```

### Method 3: Standalone
```bash
python3 movement_recorder.py
```

## Usage Guide

### 1. Recording a New Sequence

1. Select **"Grabar nueva secuencia"** from the menu
2. Enter a name (e.g., "path_to_hospital")
3. For each step:
   - Choose action type (forward, backward, spin_left, spin_right, turn_left, turn_right, stop)
   - Enter speed (0-255, default: 150)
   - Enter duration in seconds (default: 1.0)
   - Optionally test the step before adding it
4. Press Enter without input to finish recording
5. Save the sequence

### 2. Available Actions

| Action | Description | Speed Parameter |
|--------|-------------|-----------------|
| `forward` | Move straight ahead | Both wheels forward at same speed |
| `backward` | Move straight back | Both wheels backward at same speed |
| `spin_left` | Rotate in place left | Left wheel backward, right forward |
| `spin_right` | Rotate in place right | Left wheel forward, right backward |
| `turn_left` | Curved turn left | Left wheel slower than right |
| `turn_right` | Curved turn right | Right wheel slower than left |
| `stop` | Pause/stop motors | Speed ignored (set to 0) |

### 3. Example Recording Session

```
Nombre de la secuencia: navigate_to_zone_1

--- Paso 1 ---
Acciones disponibles:
  1. forward      - Avanzar hacia adelante
  2. backward     - Retroceder
  ...
Elegir acción: 1
Velocidad [0-255, Enter=150]: 160
Duración (segundos) [Enter=1.0]: 2.5
¿Probar este paso? [y/N]: y
Ejecutando paso...
✓ Paso 1 añadido: forward @ 160 por 2.5s

--- Paso 2 ---
Elegir acción: 4
Velocidad [0-255, Enter=150]: 100
Duración (segundos) [Enter=1.0]: 1.2
✓ Paso 2 añadido: spin_right @ 100 por 1.2s

...
[Press Enter to finish]
```

### 4. Replaying a Sequence

1. Load a saved sequence (option 4)
2. Select **"Reproducir secuencia actual"** (option 2)
3. Press Enter to start
4. Watch the robot execute each step
5. Press Ctrl+C to emergency stop

### 5. Managing Sequences

- **List sequences**: View all saved sequences with details
- **Delete sequences**: Remove unwanted sequences
- **View details**: See all steps in the current sequence

## File Format

Sequences are saved as JSON in `data/movements/`:

```json
{
  "name": "path_to_hospital",
  "created_at": "2025-01-22T10:30:00",
  "steps": [
    {
      "action": "forward",
      "speed": 150,
      "duration": 2.0
    },
    {
      "action": "spin_right",
      "speed": 100,
      "duration": 1.5
    }
  ],
  "total_duration": 3.5
}
```

You can also edit these files manually in a text editor!

## Tips & Best Practices

### Speed Selection
- **Slow (50-100)**: Precise movements, turning, positioning
- **Medium (120-180)**: General navigation
- **Fast (200-255)**: Long straight paths

### Duration Estimation
Test on a clear surface and measure:
- How far does the robot travel at speed X in 1 second?
- How many degrees does it rotate at speed Y in 1 second?

Use these measurements to calculate durations for your desired movements.

### Building Competition Paths

1. **Break down into segments**:
   - Start → First block: Record as "segment_1_to_block"
   - Block → Hospital: Record as "segment_block_to_hospital"
   - etc.

2. **Test individually**: Test each segment before combining

3. **Add safety stops**: Include short `stop` actions between major turns

4. **Create variants**: Save multiple versions (e.g., "fast_path", "safe_path")

### Calibration Integration

Once you fix your sensors, you can:
1. Record a path using Movement Recorder
2. Replay it while logging sensor data
3. Compare expected vs. actual movement (using sensor fusion)
4. Refine your motor calibration values

## Troubleshooting

### "Error: No se pudieron inicializar los motores"
- Check L298N motor driver connections
- Verify GPIO pins in `config.py` match your wiring
- Ensure sufficient power supply to motors

### Robot doesn't move during replay
- Check motor connections (IN1, IN2, IN3, IN4)
- Verify ENA/ENB pins are connected
- Try increasing speed values (motors may have minimum threshold)

### Robot deviates from expected path
- Motors may be unbalanced (use calibration mode)
- Surface friction affects movement
- Battery voltage affects motor speed
- Record new sequences on the actual competition surface

## Example Sequences

### Simple Test Pattern
```
forward @ 150, 2s
spin_right @ 100, 1.5s
forward @ 150, 2s
spin_right @ 100, 1.5s
(Makes a square)
```

### Competition-Style Path
```
forward @ 180, 3s       (Fast approach to debris zone)
spin_left @ 80, 0.8s    (Precise turn to block)
forward @ 60, 1s        (Slow approach to block)
stop @ 0, 2s            (Time to grab block)
backward @ 100, 1.5s    (Back away from debris)
spin_right @ 100, 2s    (Turn toward hospital)
forward @ 200, 4s       (Fast delivery run)
stop @ 0, 1s            (Release block)
```

## Integration with Competition Strategy

The Movement Recorder gives your robot a "baseline path" it can follow even without sensors. Once sensors work, you can:

1. **Hybrid mode**: Use recorded path as primary route, sensors for obstacle avoidance
2. **Fallback**: If sensors fail, fall back to recorded path
3. **Training data**: Compare sensor-based navigation vs. recorded path to improve algorithms

## Next Steps

1. ✅ Record a simple test sequence (forward, turn, forward)
2. ✅ Test replay on a clear surface
3. ✅ Measure actual distances/rotations and refine
4. ✅ Build competition path segments
5. ⏳ Fix sensors and integrate with sensor fusion
6. ⏳ Create hybrid autonomous/recorded system

---

**Note**: This feature is designed to work independently of the broken ultrasonic and IMU sensors. It only requires the motor controller to be functional. Once you fix the sensors, the movement recorder can complement your autonomous navigation system.
