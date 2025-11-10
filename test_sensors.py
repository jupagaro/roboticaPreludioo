#!/usr/bin/env python3
"""
test_sensors.py
Comprehensive hardware testing script for ultrasonic sensors and IMU
Tests HC-SR04 array and MPU6050 to diagnose sensor issues
"""

import sys
import time
import RPi.GPIO as GPIO

# Ensure config module can be imported
sys.path.insert(0, '/home/pi/roboticaPreludioo')

from hardware.ultrasonic import UltrasonicArray
from hardware.imu import IMUController


class HardwareTester:
    """Unified hardware testing system"""

    def __init__(self):
        print("=" * 70)
        print("RESCUE ROBOT 2025 - HARDWARE TEST SUITE")
        print("=" * 70)
        print()

        # Initialize GPIO
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            print("✅ GPIO initialized")
        except Exception as e:
            print(f"❌ GPIO initialization failed: {e}")
            sys.exit(1)

        # Initialize sensors
        self.ultrasonic = None
        self.imu = None

        self._init_sensors()

    def _init_sensors(self):
        """Initialize all sensors"""
        print("\n" + "=" * 70)
        print("INITIALIZING SENSORS")
        print("=" * 70)

        # Initialize ultrasonic array
        try:
            print("\n[1/2] Initializing Ultrasonic Sensors...")
            self.ultrasonic = UltrasonicArray()
            status = self.ultrasonic.get_array_status()

            if status['active_sensors'] > 0:
                print(f"✅ Ultrasonic array ready: {status['active_sensors']}/{status['total_sensors']} sensors active")
            else:
                print("⚠️ No ultrasonic sensors initialized - check wiring!")
        except Exception as e:
            print(f"❌ Ultrasonic initialization failed: {e}")

        # Initialize IMU
        try:
            print("\n[2/2] Initializing IMU (MPU6050)...")
            self.imu = IMUController()

            if self.imu.is_connected:
                print("✅ IMU ready")
            else:
                print("⚠️ IMU not connected - check I2C connection!")
        except Exception as e:
            print(f"❌ IMU initialization failed: {e}")

        print("\n" + "=" * 70)

    def quick_test(self):
        """Quick test of all sensors"""
        print("\n[QUICK TEST] Reading all sensors once...")
        print("-" * 70)

        # Test ultrasonics
        if self.ultrasonic:
            print("\nUltrasonic Sensors:")
            readings = self.ultrasonic.read_all_sensors()

            for sensor_name, distance in sorted(readings.items()):
                if distance > 0:
                    status_icon = "✅" if distance > 10 else "⚠️"
                    print(f"  {sensor_name:15} : {distance:6.1f} cm {status_icon}")
                else:
                    print(f"  {sensor_name:15} : ERROR ❌")

        # Test IMU
        if self.imu and self.imu.is_connected:
            print("\nIMU (MPU6050):")
            gyro = self.imu.read_gyro_scaled()
            accel = self.imu.read_accel_scaled()
            temp = self.imu.read_temperature_celsius()

            print(f"  Gyroscope (°/s)  : X={gyro['x']:7.2f}  Y={gyro['y']:7.2f}  Z={gyro['z']:7.2f}")
            print(f"  Accelerometer (g): X={accel['x']:7.3f}  Y={accel['y']:7.3f}  Z={accel['z']:7.3f}")
            print(f"  Temperature      : {temp:.1f} °C")

        print("-" * 70)

    def test_ultrasonic_continuous(self, duration=30):
        """Continuous ultrasonic monitoring"""
        if not self.ultrasonic:
            print("❌ Ultrasonic array not available")
            return

        print(f"\n[ULTRASONIC CONTINUOUS TEST] Running for {duration} seconds...")
        print("Place obstacles at different distances to test detection")
        print("-" * 70)

        start_time = time.time()
        iteration = 0

        try:
            while (time.time() - start_time) < duration:
                readings = self.ultrasonic.read_all_sensors()
                iteration += 1

                # Display readings
                print(f"\r[{iteration:04d}] ", end="")

                sensor_order = ['frontal_izq', 'frontal_der', 'trasero_izq', 'trasero_der']
                display_parts = []

                for sensor_name in sensor_order:
                    if sensor_name in readings:
                        distance = readings[sensor_name]
                        if distance > 0:
                            if distance < 10:
                                status = "🔴"  # Very close
                            elif distance < 30:
                                status = "🟡"  # Close
                            else:
                                status = "🟢"  # Clear
                            display_parts.append(f"{sensor_name[:3]}:{distance:5.1f}cm{status}")
                        else:
                            display_parts.append(f"{sensor_name[:3]}:ERROR❌")

                print(" | ".join(display_parts), end="", flush=True)
                time.sleep(0.3)

        except KeyboardInterrupt:
            print("\n⏹️ Test stopped by user")

        print("\n" + "-" * 70)

        # Show statistics
        status = self.ultrasonic.get_array_status()
        print("\nTest Statistics:")
        print(f"  Total readings: {iteration}")
        print(f"  Active sensors: {status['active_sensors']}/{status['total_sensors']}")
        print(f"  Total errors  : {status['total_errors']}")

        for sensor_name, details in status['sensor_details'].items():
            error_rate = (details['error_count'] / iteration * 100) if iteration > 0 else 0
            print(f"  {sensor_name:15} : {details['error_count']:4d} errors ({error_rate:.1f}%)")

    def test_imu_continuous(self, duration=30):
        """Continuous IMU monitoring"""
        if not self.imu or not self.imu.is_connected:
            print("❌ IMU not available")
            return

        print(f"\n[IMU CONTINUOUS TEST] Running for {duration} seconds...")
        print("Move and rotate the robot to see sensor changes")
        print("-" * 70)

        start_time = time.time()
        self.imu.reset_integrated_heading(0)

        try:
            while (time.time() - start_time) < duration:
                gyro = self.imu.read_gyro_scaled()
                accel = self.imu.read_accel_scaled()
                heading = self.imu.update_integrated_heading()

                print(f"\r", end="")
                print(f"Gyro Z:{gyro['z']:7.2f}°/s | ", end="")
                print(f"Accel X:{accel['x']:6.3f} Y:{accel['y']:6.3f} Z:{accel['z']:6.3f}g | ", end="")
                print(f"Heading:{heading:6.1f}°", end="", flush=True)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n⏹️ Test stopped by user")

        print("\n" + "-" * 70)

    def test_both_sensors(self, duration=30):
        """Test both ultrasonic and IMU together"""
        print(f"\n[COMBINED SENSOR TEST] Running for {duration} seconds...")
        print("This simulates real robot operation")
        print("-" * 70)

        start_time = time.time()
        self.imu.reset_integrated_heading(0) if self.imu else None

        try:
            while (time.time() - start_time) < duration:
                # Read ultrasonics
                us_readings = {}
                if self.ultrasonic:
                    us_readings = self.ultrasonic.read_all_sensors()

                # Read IMU
                heading = 0
                if self.imu and self.imu.is_connected:
                    heading = self.imu.update_integrated_heading()

                # Display
                print(f"\r", end="")

                # Ultrasonic summary
                if us_readings:
                    front_min = min([us_readings.get('frontal_izq', -1), us_readings.get('frontal_der', -1)])
                    if front_min > 0:
                        print(f"Front:{front_min:5.1f}cm | ", end="")
                    else:
                        print("Front:ERROR | ", end="")

                # IMU summary
                print(f"Heading:{heading:6.1f}°", end="", flush=True)

                time.sleep(0.2)

        except KeyboardInterrupt:
            print("\n⏹️ Test stopped by user")

        print("\n" + "-" * 70)

    def calibrate_imu(self):
        """Calibrate IMU sensors"""
        if not self.imu or not self.imu.is_connected:
            print("❌ IMU not available for calibration")
            return

        print("\n[IMU CALIBRATION]")
        print("-" * 70)

        print("\n1. Calibrating Gyroscope Z (for heading)")
        print("   ⚠️ Keep robot COMPLETELY STILL!")
        input("   Press Enter when ready...")
        self.imu.calibrate_gyro_z(samples=100)

        print("\n2. Calibrating Accelerometer")
        print("   ⚠️ Place robot on FLAT surface!")
        input("   Press Enter when ready...")
        self.imu.calibrate_accelerometer(samples=50)

        print("\n✅ IMU calibration complete!")
        print("-" * 70)

    def diagnose_ultrasonic(self):
        """Diagnose ultrasonic sensor issues"""
        if not self.ultrasonic:
            print("❌ Ultrasonic array not available")
            return

        print("\n[ULTRASONIC DIAGNOSTICS]")
        print("-" * 70)

        status = self.ultrasonic.get_array_status()

        print(f"\nArray Status:")
        print(f"  Total sensors: {status['total_sensors']}")
        print(f"  Active sensors: {status['active_sensors']}")
        print(f"  Failed sensors: {status['total_sensors'] - status['active_sensors']}")

        print(f"\nSensor Details:")
        for sensor_name, details in status['sensor_details'].items():
            init_icon = "✅" if details['initialized'] else "❌"
            print(f"\n  {sensor_name}:")
            print(f"    Initialized: {init_icon}")
            print(f"    Pins: Trig={details['pins']['trig']}, Echo={details['pins']['echo']}")
            print(f"    Last reading: {details['last_reading']:.1f} cm")
            print(f"    Error count: {details['error_count']}")

            if not details['initialized']:
                print(f"    ⚠️ Check wiring for Trig pin {details['pins']['trig']} and Echo pin {details['pins']['echo']}")
            elif details['error_count'] > 10:
                print(f"    ⚠️ High error count - possible wiring or interference issue")

        # Test each sensor individually
        print("\n\nTesting each sensor individually (5 readings each)...")
        for sensor_name in status['sensor_details'].keys():
            print(f"\n  {sensor_name}:", end=" ")

            readings = []
            for i in range(5):
                distance = self.ultrasonic.read_sensor(sensor_name)
                readings.append(distance)

                if distance > 0:
                    print(f"{distance:.1f}", end=" ")
                else:
                    print("ERR", end=" ")

                time.sleep(0.1)

            # Analysis
            valid_readings = [r for r in readings if r > 0]
            if len(valid_readings) >= 3:
                avg = sum(valid_readings) / len(valid_readings)
                print(f"| Avg: {avg:.1f}cm ✅")
            elif len(valid_readings) > 0:
                print(f"| Intermittent ⚠️")
            else:
                print(f"| Failed ❌")

        print("\n" + "-" * 70)

        # Recommendations
        print("\nRecommendations:")
        if status['active_sensors'] < status['total_sensors']:
            print("  ⚠️ Some sensors failed to initialize")
            print("     - Check power supply (HC-SR04 needs stable 5V)")
            print("     - Verify GPIO pin connections")
            print("     - Check for loose wires")

        if status['total_errors'] > 50:
            print("  ⚠️ High error rate detected")
            print("     - Sensors may be too close together (cross-talk)")
            print("     - Check for interference from motors")
            print("     - Try increasing delay between sensor readings")

    def interactive_menu(self):
        """Main interactive menu"""
        while True:
            print("\n" + "=" * 70)
            print("HARDWARE TEST MENU")
            print("=" * 70)
            print()
            print("Basic Tests:")
            print("  1. Quick test (all sensors once)")
            print("  2. Continuous ultrasonic test (30s)")
            print("  3. Continuous IMU test (30s)")
            print("  4. Combined sensor test (30s)")
            print()
            print("Diagnostics:")
            print("  5. Diagnose ultrasonic issues")
            print("  6. Sensor status report")
            print()
            print("Calibration:")
            print("  7. Calibrate IMU")
            print()
            print("  0. Exit")
            print()

            try:
                choice = input("Select option: ").strip()

                if choice == '1':
                    self.quick_test()

                elif choice == '2':
                    duration = input("Duration in seconds (default 30): ").strip()
                    duration = int(duration) if duration else 30
                    self.test_ultrasonic_continuous(duration)

                elif choice == '3':
                    duration = input("Duration in seconds (default 30): ").strip()
                    duration = int(duration) if duration else 30
                    self.test_imu_continuous(duration)

                elif choice == '4':
                    duration = input("Duration in seconds (default 30): ").strip()
                    duration = int(duration) if duration else 30
                    self.test_both_sensors(duration)

                elif choice == '5':
                    self.diagnose_ultrasonic()

                elif choice == '6':
                    self.show_status_report()

                elif choice == '7':
                    self.calibrate_imu()

                elif choice == '0':
                    print("\n👋 Exiting...")
                    break

                else:
                    print("❌ Invalid option")

            except KeyboardInterrupt:
                print("\n\n👋 Exiting...")
                break
            except Exception as e:
                print(f"\n❌ Error: {e}")

    def show_status_report(self):
        """Show comprehensive status report"""
        print("\n[STATUS REPORT]")
        print("=" * 70)

        # Ultrasonic status
        if self.ultrasonic:
            status = self.ultrasonic.get_array_status()
            print("\nUltrasonic Array:")
            print(f"  Active: {status['active_sensors']}/{status['total_sensors']}")
            print(f"  Errors: {status['total_errors']}")

            print("\n  Current readings:")
            for sensor, distance in status['current_readings'].items():
                icon = "✅" if distance > 0 else "❌"
                print(f"    {sensor:15} : {distance:6.1f} cm {icon}")
        else:
            print("\nUltrasonic Array: ❌ Not available")

        # IMU status
        if self.imu and self.imu.is_connected:
            status = self.imu.get_sensor_status()
            print("\nIMU (MPU6050):")
            print(f"  Connected: ✅")
            print(f"  Gyro Z offset: {status['gyro_z_offset']:.3f} °/s")
            print(f"  Integrated heading: {status['integrated_heading']:.1f}°")
            print(f"  Temperature: {status['last_temp']:.1f}°C")
        else:
            print("\nIMU (MPU6050): ❌ Not connected")

        print("\n" + "=" * 70)

    def cleanup(self):
        """Cleanup GPIO"""
        try:
            GPIO.cleanup()
            print("\n✅ GPIO cleanup complete")
        except:
            pass


def main():
    """Main entry point"""
    try:
        tester = HardwareTester()
        tester.interactive_menu()

    except KeyboardInterrupt:
        print("\n\n👋 Interrupted by user")

    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        try:
            GPIO.cleanup()
        except:
            pass


if __name__ == "__main__":
    main()
