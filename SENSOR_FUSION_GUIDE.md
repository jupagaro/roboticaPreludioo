# Sensor Fusion Integration Guide

## Overview

This guide explains how to use the **enriched movement recording system** that integrates sensor fusion with your baseline movement recordings. This allows you to:

1. **Record movements WITH sensor data** - Capture the actual trajectory while teaching the robot
2. **Replay movements WITH sensor tracking** - See what really happens during playback
3. **Analyze drift and accuracy** - Compare expected vs actual execution
4. **Make informed decisions** - Use data to decide if/when corrections are needed

## Quick Start

### Step 1: Record a Baseline Movement with Sensors

```bash
python3 movement_recorder.py
```

When prompted:
- Select **[y]** to enable sensor fusion
- Choose option **3** - "Grabar con sensores (enriquecida)"
- Teach the robot your desired path step by step

**What you get:**
- A movement recording file (`.json`) with motor commands
- Position data at start/end of each step
- Sensor readings during execution
- A linked sensor fusion session with full trajectory

### Step 2: Replay and Analyze

From the same menu:
- Choose option **4** - "Reproducir con sensores (análisis)"
- Load your recorded sequence (option 6 first if needed)
- Watch it execute and capture real-time sensor data

**What you get:**
- Real trajectory data from actual execution
- Step-by-step position tracking
- Comparison with expected positions (if available)
- Error metrics (drift, heading error)

### Step 3: Generate Analysis Report

From the menu:
- Choose option **a** - "Analizar última reproducción"
- Select your playback session
- Review the comprehensive analysis

**What you get:**
- Position drift analysis
- Heading drift analysis
- Sensor reliability report
- Recommendations for improvements

## Detailed Workflow

### Recording Modes

#### Simple Recording (Option 1)
- **Uses:** Motors only
- **Speed:** Fast
- **Data:** Motor commands only
- **Best for:** Quick teaching, no analysis needed

#### Enriched Recording (Option 3)
- **Uses:** Motors + Sensor Fusion
- **Speed:** Slightly slower (sensor overhead)
- **Data:** Motor commands + trajectory + obstacles
- **Best for:** Creating baseline with reference data

### Playback Modes

#### Simple Playback (Option 2)
- **Open-loop execution** - Just follows motor commands blindly
- **No sensor feedback**
- **Fast and simple**

#### Smart Playback (Option 4)
- **Tracks actual trajectory** using sensor fusion
- **Compares with expected trajectory** (if enriched recording)
- **Generates drift/error metrics**
- **Saves detailed session data for analysis**

## Understanding the Data

### Enriched Recording Output

```json
{
  "name": "path_to_hospital",
  "recording_mode": "sensor_fusion_enabled",
  "steps": [
    {
      "action": "forward",
      "speed": 150,
      "duration": 2.0,
      "sensor_fusion": {
        "start_position": {"x": 0.0, "y": 0.0, "heading": 0.0},
        "end_position": {"x": 21.7, "y": 0.3, "heading": 1.2},
        "sensor_summary": {
          "navigation": {...},
          "sensors_ok": true,
          "obstacles_detected": 0
        }
      }
    }
  ],
  "sensor_fusion_session": "/path/to/full/session.json"
}
```

### Playback Analysis Output

```json
{
  "sequence_name": "path_to_hospital",
  "steps_executed": [
    {
      "step_number": 1,
      "action": "forward",
      "actual_execution": {
        "start_position": {"x": 0.0, "y": 0.0, "heading": 0.0},
        "end_position": {"x": 20.5, "y": 1.2, "heading": 2.1},
        "distance_traveled": 20.8
      },
      "position_error": {
        "x": -1.2,
        "y": 0.9,
        "heading": 0.9
      }
    }
  ]
}
```

## Analysis Report Explained

### Position Drift
- **What it measures:** How far the robot deviates from expected position
- **Good value:** < 10cm cumulative error for short paths (< 2m)
- **Concerning:** > 50cm - indicates calibration issues

### Heading Drift
- **What it measures:** How much orientation drifts from expected
- **Good value:** < 5° cumulative error
- **Concerning:** > 15° - check gyroscope calibration

### Sensor Reliability
- **Success rate:** % of valid sensor readings
- **Good value:** > 90%
- **Concerning:** < 80% - indicates blocking or hardware issues

**Key insight for your robot:** Front sensors will be blocked when carrying green/red pieces. The analyzer will show this clearly in the sensor reliability section.

## Use Cases

### Use Case 1: Validate Simple Recording
**Goal:** You have a simple recording and want to see what actually happens

**Steps:**
1. Load simple recording (option 6)
2. Run smart playback (option 4)
3. Analyze results (option a)

**Output:** Real trajectory data showing actual drift/accuracy

### Use Case 2: Create Baseline with Reference
**Goal:** Teach robot a path AND capture ideal execution data

**Steps:**
1. Record with sensors (option 3)
2. Save recording (option 5)

**Output:** Enriched recording you can replay and compare against

### Use Case 3: Compare Executions Over Time
**Goal:** See if performance degrades (battery, wheel wear, etc.)

**Steps:**
1. Create enriched baseline (option 3)
2. Replay multiple times with sensors (option 4)
3. Compare analysis reports

**Output:** Trends in drift and accuracy over multiple runs

### Use Case 4: Test with Cargo
**Goal:** See how carrying pieces affects execution

**Steps:**
1. Record/load path without cargo
2. Smart playback without cargo (option 4)
3. Smart playback WITH cargo (option 4)
4. Compare sensor reliability reports

**Output:** Clear view of which sensors get blocked, how accuracy changes

## Sensor Blocking Scenario

**Your Configuration:**
- **Front sensors:** `frontal_izq`, `frontal_der` (angle 0°)
- **Back sensors:** `trasero_izq`, `trasero_der` (angles 135°, 225°)

**When carrying pieces:**
- Front sensors return `-1` (blocked/no reading)
- Back sensors remain functional
- Analyzer reports < 80% success rate for front sensors

**What to look for in reports:**
```
CONFIABILIDAD DE SENSORES
🔴 FRONTAL - frontal_izq:
  Lecturas válidas: 23 (12.5%)  ← Low! Likely blocked
  Lecturas bloqueadas/error: 161
  ⚠ Tasa de éxito baja - posible bloqueo u obstrucción

🔵 TRASERO - trasero_izq:
  Lecturas válidas: 178 (98.8%)  ← Good! Still working
```

## Recommendations Based on Analysis

### High Position Drift (> 50cm)
**Causes:**
- Incorrect `speed_factor` in `config.py`
- Wheels slipping on surface
- Battery voltage drop

**Actions:**
1. Run calibration: `python3 main.py --mode calibration`
2. Check wheel traction
3. Verify surface is consistent

### High Heading Drift (> 15°)
**Causes:**
- Gyroscope drift/bias
- Unbalanced differential drive
- Incorrect `gyro_alpha` weight

**Actions:**
1. Recalibrate IMU in `calibration.py`
2. Test straight-line movement
3. Adjust `gyro_alpha` in `config.py` (default: 0.98)

### Low Sensor Reliability (< 80%)
**Causes:**
- Physical obstruction (carrying pieces)
- Loose connections
- Environmental interference

**Actions:**
1. Check wiring and connections
2. If carrying cargo, rely on back sensors only
3. Consider sensor placement adjustments

## File Organization

After using the system, you'll have:

```
data/
├── movements/
│   ├── path_to_hospital.json          # Movement recording
│   ├── square_test.json                # Another recording
│
├── sessions/
│   ├── recording_path_to_hospital_20251110_143052.json  # Enriched recording session
│   ├── playback_path_to_hospital_20251110_143521.json   # Playback execution
│   ├── playback_path_to_hospital_20251110_144102.json   # Another playback
│   └── analysis_path_to_hospital_20251110_144215.json   # Saved analysis report
```

## Next Steps: Smart Corrections (Phase 4)

**IMPORTANT:** Only implement corrections AFTER reviewing your enriched data!

Based on your analysis, you might want to add:

1. **Sensor-aware obstacle avoidance**
   - Use back sensors when front blocked
   - Implement different strategies based on cargo state

2. **Drift compensation**
   - Periodic position corrections
   - Heading realignment at checkpoints

3. **Adaptive timing**
   - Adjust durations based on actual progress
   - Compensate for battery voltage drop

These will be implemented in Phase 4, ONLY if your analysis shows they're needed.

## Troubleshooting

### "Sensor fusion no está habilitado"
**Solution:** Restart menu and choose [y] when asked about sensor fusion

### "No playback files available"
**Solution:** You need to run smart playback (option 4) first before analyzing

### Sensor fusion initialization fails
**Possible causes:**
- IMU not connected (I2C)
- Ultrasonic sensors not wired correctly
- GPIO permissions

**Solution:** Run hardware tests first:
```bash
python3 test_sensors.py
```

### Analysis shows huge drift immediately
**Solution:** Robot might not be starting from expected position. Make sure:
- Robot starts at same position/orientation each time
- Surface hasn't changed
- Wheels are clean and not slipping

## Performance Tips

### For Best Results:
1. **Calibrate before each session** - Temperature affects sensors
2. **Use consistent surface** - Different floors = different friction
3. **Check battery** - Low voltage affects motor performance
4. **Start from same position** - Set up a marked starting zone

### For Faster Testing:
1. **Use simple recording** (option 1) for quick iterations
2. **Only use enriched mode** when you need data
3. **Smart playback is slower** due to sensor polling - plan accordingly

## Summary

This system gives you **visibility into what's actually happening** when your robot executes movements. Use it to:

✅ Validate that simple recordings work as expected
✅ Measure accuracy and drift quantitatively
✅ Identify when/where corrections are needed
✅ Understand sensor blocking scenarios
✅ Make data-driven decisions about implementing AI corrections

**Remember:** The goal is SIMPLE AI. Use sensor fusion for intelligence, add MINIMAL reactive rules only where data shows they're needed.
