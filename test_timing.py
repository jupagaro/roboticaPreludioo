#!/usr/bin/env python3
"""
test_timing.py
Verify that new timing settings are loaded and sensors work
"""

import sys
import time
import RPi.GPIO as GPIO

sys.path.insert(0, '/home/pi/roboticaPreludioo')

# Import config to verify settings
from config import CALIBRATION, DETECTION_THRESHOLDS
from hardware.ultrasonic import UltrasonicArray

print("=" * 70)
print("TIMING SETTINGS VERIFICATION")
print("=" * 70)

# Check config values
print("\nConfig Values:")
print(f"  sensor_timeout: {CALIBRATION['sensor_timeout']}s")
print(f"  sensor_valid_min: {DETECTION_THRESHOLDS['sensor_valid_min']}cm")
print(f"  sensor_valid_max: {DETECTION_THRESHOLDS['sensor_valid_max']}cm")

if CALIBRATION['sensor_timeout'] < 0.3:
    print("\n⚠️ WARNING: sensor_timeout is still too low!")
    print("   Expected: 0.3s or higher")
    print("   Current: {}s".format(CALIBRATION['sensor_timeout']))
else:
    print("\n✅ Timeout setting looks good")

# Initialize sensors
print("\n" + "=" * 70)
print("TESTING SENSORS WITH NEW SETTINGS")
print("=" * 70)

try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    ultrasonic = UltrasonicArray()

    print("\nTesting each sensor 10 times...")
    print("Place objects in front of sensors for valid readings")
    print()

    # Test each sensor multiple times
    for i in range(10):
        print(f"\n--- Reading {i+1}/10 ---")

        readings = ultrasonic.read_all_sensors()

        for sensor_name, distance in sorted(readings.items()):
            if distance > 0:
                print(f"  {sensor_name:15}: {distance:6.1f} cm ✅")
            else:
                print(f"  {sensor_name:15}: ERROR ❌")

        time.sleep(0.5)

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)

    status = ultrasonic.get_array_status()

    print(f"\nActive sensors: {status['active_sensors']}/{status['total_sensors']}")
    print(f"Total errors: {status['total_errors']}")

    print("\nPer-sensor statistics:")
    for sensor_name, details in status['sensor_details'].items():
        print(f"  {sensor_name:15}: {details['error_count']:3d} errors, last: {details['last_reading']:6.1f}cm")

    # Test individual sensor (like calibration does)
    print("\n" + "=" * 70)
    print("SIMULATING CALIBRATION READ PATTERN")
    print("=" * 70)
    print("\nThis mimics what calibration.py does...")

    sensor_name = 'frontal_izq'
    print(f"\nReading {sensor_name} 10 times with 0.2s delay (like calibration):")

    readings = []
    for i in range(10):
        reading = ultrasonic.read_sensor(sensor_name)
        if reading > 0:
            readings.append(reading)
            print(f"  Reading {i+1}: {reading:.1f} cm ✅")
        else:
            print(f"  Reading {i+1}: ERROR ❌")
        time.sleep(0.2)

    print(f"\nResults: {len(readings)}/10 successful")

    if len(readings) == 0:
        print("❌ PROBLEM: No successful readings!")
        print("\nPossible causes:")
        print("  1. Nothing in front of sensor (place object 10-50cm away)")
        print("  2. Config changes not loaded (try: sudo systemctl restart or reboot)")
        print("  3. Sensor hardware issue")
    elif len(readings) < 5:
        print("⚠️ INTERMITTENT: Some readings failing")
    else:
        avg = sum(readings) / len(readings)
        print(f"✅ SUCCESS: Average distance {avg:.1f}cm")

except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()

finally:
    GPIO.cleanup()
    print("\nGPIO cleaned up")
