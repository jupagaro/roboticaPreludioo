#!/usr/bin/env python3
"""
diagnose_ultrasonic.py
Deep diagnostics for HC-SR04 ultrasonic sensors
Tests hardware at low level to identify exact issues
"""

import sys
import time
import RPi.GPIO as GPIO

sys.path.insert(0, '/home/pi/roboticaPreludioo')
from config import ULTRASONIC_PINS


def test_gpio_basics():
    """Test if GPIO is working at all"""
    print("\n" + "=" * 70)
    print("TEST 1: GPIO BASIC FUNCTIONALITY")
    print("=" * 70)

    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        print("✅ GPIO mode set successfully")
        return True
    except Exception as e:
        print(f"❌ GPIO setup failed: {e}")
        return False


def test_pin_setup(trig_pin, echo_pin, sensor_name):
    """Test if specific pins can be configured"""
    print(f"\n{sensor_name}:")
    print(f"  Trig: GPIO{trig_pin}, Echo: GPIO{echo_pin}")

    try:
        GPIO.setup(trig_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)
        print(f"  ✅ Pins configured successfully")
        return True
    except Exception as e:
        print(f"  ❌ Pin setup failed: {e}")
        return False


def test_trigger_output(trig_pin, sensor_name):
    """Test if trigger pin can output signals"""
    print(f"\n  Testing trigger output on GPIO{trig_pin}...")

    try:
        # Try to set HIGH and LOW
        GPIO.output(trig_pin, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(0.00001)  # 10us pulse
        GPIO.output(trig_pin, GPIO.LOW)

        print(f"  ✅ Trigger pulse sent")
        return True
    except Exception as e:
        print(f"  ❌ Trigger failed: {e}")
        return False


def test_echo_response(trig_pin, echo_pin, sensor_name):
    """Test if echo pin ever responds"""
    print(f"\n  Testing echo response on GPIO{echo_pin}...")

    try:
        # Send trigger
        GPIO.output(trig_pin, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig_pin, GPIO.LOW)

        # Check initial echo state
        initial_state = GPIO.input(echo_pin)
        print(f"  Echo pin initial state: {'HIGH' if initial_state else 'LOW'}")

        # Wait for echo to go HIGH with longer timeout
        timeout_start = time.time()
        timeout = 0.5  # 500ms timeout (much longer than normal)

        print(f"  Waiting for echo HIGH...", end=" ")
        while GPIO.input(echo_pin) == GPIO.LOW:
            if (time.time() - timeout_start) > timeout:
                print(f"TIMEOUT after {timeout}s")
                print(f"  ❌ Echo pin never went HIGH - possible issues:")
                print(f"     1. Sensor not powered (needs 5V)")
                print(f"     2. Defective sensor")
                print(f"     3. Echo wire disconnected")
                print(f"     4. Wrong pin mapping")
                return False

        pulse_start = time.time()
        wait_time = pulse_start - timeout_start
        print(f"HIGH after {wait_time*1000:.1f}ms")

        # Wait for echo to go LOW
        print(f"  Waiting for echo LOW...", end=" ")
        while GPIO.input(echo_pin) == GPIO.HIGH:
            if (time.time() - pulse_start) > timeout:
                print(f"TIMEOUT")
                print(f"  ⚠️ Echo stuck HIGH - sensor may be malfunctioning")
                return False

        pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150

        print(f"LOW after {pulse_duration*1000:.1f}ms")
        print(f"  ✅ Calculated distance: {distance:.1f} cm")

        if distance < 2 or distance > 400:
            print(f"  ⚠️ Distance out of valid range (2-400cm)")

        return True

    except Exception as e:
        print(f"  ❌ Echo test failed: {e}")
        return False


def test_sensor_detailed(sensor_name, trig_pin, echo_pin):
    """Detailed test of a single sensor"""
    print("\n" + "-" * 70)
    print(f"DETAILED TEST: {sensor_name.upper()}")
    print("-" * 70)

    # Step 1: Pin setup
    if not test_pin_setup(trig_pin, echo_pin, sensor_name):
        return False

    # Step 2: Trigger output
    if not test_trigger_output(trig_pin, sensor_name):
        return False

    # Step 3: Echo response (most critical)
    if not test_echo_response(trig_pin, echo_pin, sensor_name):
        return False

    # Step 4: Multiple readings
    print(f"\n  Taking 5 consecutive readings:")
    readings = []

    for i in range(5):
        try:
            # Send trigger
            GPIO.output(trig_pin, GPIO.LOW)
            time.sleep(0.05)  # Longer delay between readings
            GPIO.output(trig_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(trig_pin, GPIO.LOW)

            # Measure echo
            timeout_start = time.time()

            # Wait for HIGH
            while GPIO.input(echo_pin) == GPIO.LOW:
                if (time.time() - timeout_start) > 0.1:
                    print(f"    Reading {i+1}: ERROR (no echo)")
                    break
            else:
                pulse_start = time.time()

                # Wait for LOW
                while GPIO.input(echo_pin) == GPIO.HIGH:
                    if (time.time() - pulse_start) > 0.1:
                        break

                pulse_end = time.time()
                pulse_duration = pulse_end - pulse_start
                distance = pulse_duration * 17150

                if 2 <= distance <= 400:
                    readings.append(distance)
                    print(f"    Reading {i+1}: {distance:.1f} cm ✅")
                else:
                    print(f"    Reading {i+1}: {distance:.1f} cm (out of range)")

            time.sleep(0.06)  # Wait between readings

        except Exception as e:
            print(f"    Reading {i+1}: ERROR ({e})")

    # Analysis
    print(f"\n  Analysis:")
    if len(readings) >= 3:
        avg = sum(readings) / len(readings)
        print(f"    Success rate: {len(readings)}/5 ({len(readings)*20}%)")
        print(f"    Average distance: {avg:.1f} cm")
        print(f"    ✅ Sensor appears to be working!")
        return True
    elif len(readings) > 0:
        print(f"    Success rate: {len(readings)}/5 ({len(readings)*20}%)")
        print(f"    ⚠️ Intermittent - check connections")
        return False
    else:
        print(f"    Success rate: 0/5 (0%)")
        print(f"    ❌ Sensor not working")
        return False


def check_power_supply():
    """Provide guidance on checking power supply"""
    print("\n" + "=" * 70)
    print("POWER SUPPLY CHECK")
    print("=" * 70)
    print("\nHC-SR04 sensors require:")
    print("  • Voltage: 5V DC (NOT 3.3V!)")
    print("  • Current: ~15mA per sensor (60mA total for 4 sensors)")
    print("\n⚠️ Common power issues:")
    print("  1. Using 3.3V instead of 5V → sensors won't work")
    print("  2. Insufficient current from Raspberry Pi 5V pin")
    print("  3. Long/thin wires causing voltage drop")
    print("  4. Sharing power with motors (causes voltage sags)")
    print("\n✅ How to check:")
    print("  1. Measure voltage at VCC pin of each sensor (should be 4.8-5.2V)")
    print("  2. Use external 5V power supply if Raspberry Pi can't provide enough")
    print("  3. Ensure good GND connection between Pi and sensors")
    print("  4. Use thick wires (22 AWG or better) for power")


def check_voltage_levels():
    """Provide guidance on voltage level compatibility"""
    print("\n" + "=" * 70)
    print("VOLTAGE LEVEL COMPATIBILITY")
    print("=" * 70)
    print("\nRaspberry Pi GPIO:")
    print("  • Logic HIGH: 3.3V")
    print("  • Logic LOW: 0V")
    print("  • Input pins are NOT 5V tolerant!")
    print("\nHC-SR04 Echo pin:")
    print("  • Outputs 5V when HIGH")
    print("  • This can potentially damage Raspberry Pi!")
    print("\n✅ Solutions:")
    print("  1. Use voltage divider on echo pin (recommended)")
    print("     - 1kΩ resistor from echo to GPIO")
    print("     - 2kΩ resistor from GPIO to GND")
    print("  2. Use level shifter module")
    print("  3. Some HC-SR04 modules have built-in 3.3V output")
    print("\n⚠️ Current setup status:")
    print("  Your echo pins are connected directly - this usually works")
    print("  but may cause issues with some sensor modules")


def main():
    """Main diagnostic routine"""
    print("=" * 70)
    print("ULTRASONIC SENSOR DEEP DIAGNOSTICS")
    print("=" * 70)
    print("\nThis tool will test each sensor at a low level to identify issues")
    print()

    # Test 1: GPIO basics
    if not test_gpio_basics():
        return

    # Test 2: Each sensor in detail
    print("\n" + "=" * 70)
    print("TEST 2: INDIVIDUAL SENSOR TESTING")
    print("=" * 70)

    working_sensors = []
    failed_sensors = []

    for sensor_name, pins in ULTRASONIC_PINS.items():
        result = test_sensor_detailed(sensor_name, pins['trig'], pins['echo'])

        if result:
            working_sensors.append(sensor_name)
        else:
            failed_sensors.append(sensor_name)

    # Summary
    print("\n" + "=" * 70)
    print("DIAGNOSTIC SUMMARY")
    print("=" * 70)

    print(f"\nWorking sensors: {len(working_sensors)}/4")
    if working_sensors:
        for sensor in working_sensors:
            print(f"  ✅ {sensor}")

    print(f"\nFailed sensors: {len(failed_sensors)}/4")
    if failed_sensors:
        for sensor in failed_sensors:
            print(f"  ❌ {sensor}")

    # Recommendations
    print("\n" + "=" * 70)
    print("RECOMMENDATIONS")
    print("=" * 70)

    if len(failed_sensors) == 4:
        print("\n🔴 ALL SENSORS FAILED - Likely systemic issue:")
        print("\n1. CHECK POWER SUPPLY FIRST (most common issue):")
        print("   • Are sensors connected to 5V pin? (NOT 3.3V)")
        print("   • Is GND connected?")
        print("   • Measure voltage at sensor VCC pins")
        print("\n2. CHECK WIRING:")
        print("   • Verify pin connections match config:")
        for sensor_name, pins in ULTRASONIC_PINS.items():
            print(f"     {sensor_name}: Trig=GPIO{pins['trig']}, Echo=GPIO{pins['echo']}")
        print("\n3. TEST ONE SENSOR IN ISOLATION:")
        print("   • Disconnect 3 sensors")
        print("   • Test remaining sensor alone")
        print("   • If it works, issue is power or interference")
        print("\n4. CHECK SENSOR QUALITY:")
        print("   • Try a known-good sensor")
        print("   • Some cheap HC-SR04 clones are unreliable")

    elif len(failed_sensors) > 0:
        print(f"\n🟡 {len(failed_sensors)} SENSOR(S) FAILED:")
        for sensor in failed_sensors:
            pins = ULTRASONIC_PINS[sensor]
            print(f"\n  {sensor}:")
            print(f"    • Check VCC and GND connections")
            print(f"    • Verify Trig=GPIO{pins['trig']}, Echo=GPIO{pins['echo']}")
            print(f"    • Test with a multimeter")
            print(f"    • Try swapping with a working sensor")
    else:
        print("\n✅ ALL SENSORS WORKING!")
        print("   If you're still having issues in the main program,")
        print("   the problem may be with timing or interference.")

    # Additional resources
    check_power_supply()
    check_voltage_levels()

    print("\n" + "=" * 70)
    print("DIAGNOSTIC COMPLETE")
    print("=" * 70)
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 Interrupted by user")
    finally:
        try:
            GPIO.cleanup()
            print("GPIO cleaned up")
        except:
            pass
