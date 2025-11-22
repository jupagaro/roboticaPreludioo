#!/usr/bin/env python3
"""
zone_calibration.py
Interactive tool to calibrate competition zone positions

This tool helps you:
1. Position the robot at each zone center
2. Record the actual sensor fusion coordinates
3. Automatically update config.py with calibrated values
"""

import time
import sys
import termios
import tty
from sensor_fusion import SensorFusion
from config import COMPETITION_ZONES

class ZoneCalibrator:
    def __init__(self):
        """Initialize zone calibration tool"""
        print("\n" + "="*70)
        print("ZONE POSITION CALIBRATION TOOL")
        print("="*70)
        print("\nThis tool will help you calibrate the actual positions of")
        print("competition zones by recording sensor fusion coordinates.")
        print()

        # Initialize sensor fusion with calibration
        print("Initializing sensor fusion...")
        self.sf = self.initialize_sensor_fusion()

        # Zones to calibrate
        self.zones_to_calibrate = [
            ('hospital', 'waiting_room', 'Waiting Room'),
            ('hospital', 'consultation_room', 'Consultation Room'),
            ('hospital', 'parking', 'Parking'),
            ('hospital', 'roof', 'Roof (Elevated)'),
            ('refuge', 'lower_level', 'Refuge Lower Level'),
            ('refuge', 'upper_level', 'Refuge Upper Level (Elevated)'),
            ('start_zone', 'area', 'Start Zone')
        ]

        # Store calibrated positions
        self.calibrated_positions = {}

    def initialize_sensor_fusion(self):
        """Initialize sensor fusion with gyro calibration"""
        sf = SensorFusion(session_name="zone_calibration")

        print("\n⚠️  Gyroscope calibration required")
        print("   Robot must be COMPLETELY STILL!")
        input("\nPress Enter when ready...")

        print("\n🔧 Calibrating gyroscope...")
        calibration_success = sf.imu.calibrate_gyro_z(samples=100)

        if calibration_success:
            print(f"✅ Gyroscope calibrated! Bias: {sf.imu.gyro_z_offset:.3f} °/s")
        else:
            print("⚠️  Calibration failed - heading may drift")

        # Start sensor fusion
        print("\n🚀 Starting sensor fusion...")
        sf.start_sensor_loop(update_rate=10)
        time.sleep(0.5)

        print("✅ Sensor fusion ready!")
        return sf

    def get_key(self):
        """Get single keypress (non-blocking)"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def manual_control(self):
        """
        Manual control mode for positioning robot

        Controls:
            w/s: Forward/Backward
            a/d: Spin left/right
            space: Stop
            Enter: Record position
            q: Quit manual control
        """
        print("\n" + "="*70)
        print("MANUAL CONTROL MODE")
        print("="*70)
        print("\nControls:")
        print("  w/s     - Forward/Backward (220 PWM)")
        print("  a/d     - Spin left/right (210 PWM)")
        print("  space   - Stop")
        print("  Enter   - Record this position")
        print("  q       - Quit manual control")
        print()

        speed = 220
        turn_speed = 210

        try:
            while True:
                pos = self.sf.get_position()
                nav_data = self.sf.get_navigation_data()

                print(f"\rPos: ({pos['x']:6.1f}, {pos['y']:6.1f}) | "
                      f"Heading: {pos['heading']:6.1f}° | "
                      f"Front: {nav_data['front_min']:5.1f}cm", end="")

                # Check for keypress
                key = self.get_key()

                if key == 'w':
                    self.sf.motors.move_forward(speed)
                    time.sleep(0.1)
                    self.sf.motors.stop()
                elif key == 's':
                    self.sf.motors.move_backward(speed)
                    time.sleep(0.1)
                    self.sf.motors.stop()
                elif key == 'a':
                    self.sf.motors.spin_left(turn_speed)
                    time.sleep(0.1)
                    self.sf.motors.stop()
                elif key == 'd':
                    self.sf.motors.spin_right(turn_speed)
                    time.sleep(0.1)
                    self.sf.motors.stop()
                elif key == ' ':
                    self.sf.motors.stop()
                elif key == '\n' or key == '\r':
                    # Record position
                    print(f"\n\n📍 Position recorded: ({pos['x']:.1f}, {pos['y']:.1f})")
                    return (pos['x'], pos['y'])
                elif key == 'q':
                    print("\n\nExiting manual control...")
                    return None

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n\nManual control interrupted")
            return None
        finally:
            self.sf.motors.stop()

    def calibrate_zone(self, zone_category, zone_name, zone_display_name):
        """
        Calibrate a single zone

        Args:
            zone_category: Category (e.g., 'hospital', 'refuge')
            zone_name: Zone name (e.g., 'waiting_room')
            zone_display_name: Human-readable name

        Returns:
            tuple: (x, y) coordinates or None if skipped
        """
        print("\n\n" + "="*70)
        print(f"CALIBRATING: {zone_display_name}")
        print("="*70)

        # Show current config value
        current_center = COMPETITION_ZONES[zone_category][zone_name].get('center', None)
        if current_center:
            print(f"\nCurrent config value: {current_center}")

        print("\nOptions:")
        print("  1. Use manual control to position robot")
        print("  2. Enter coordinates manually")
        print("  3. Skip this zone")

        choice = input("\nSelect option [1-3]: ").strip()

        if choice == '1':
            print("\nPosition the robot at the CENTER of the zone")
            print("Press Enter when positioned correctly")
            position = self.manual_control()
            return position

        elif choice == '2':
            try:
                x = float(input("Enter X coordinate (cm): "))
                y = float(input("Enter Y coordinate (cm): "))
                print(f"✅ Coordinates set: ({x:.1f}, {y:.1f})")
                return (x, y)
            except ValueError:
                print("❌ Invalid coordinates, skipping zone")
                return None

        elif choice == '3':
            print("⏭️  Skipping zone")
            return None

        else:
            print("Invalid choice, skipping zone")
            return None

    def run_calibration(self):
        """Run the full calibration process"""
        print("\n" + "="*70)
        print("STARTING ZONE CALIBRATION")
        print("="*70)
        print(f"\nYou will calibrate {len(self.zones_to_calibrate)} zones")
        print("\nFor each zone, you can:")
        print("  - Use manual control to drive to the zone center")
        print("  - Enter coordinates manually if you know them")
        print("  - Skip the zone to keep current config value")

        input("\nPress Enter to start calibration...")

        # Calibrate each zone
        for zone_category, zone_name, zone_display_name in self.zones_to_calibrate:
            position = self.calibrate_zone(zone_category, zone_name, zone_display_name)

            if position:
                key = f"{zone_category}.{zone_name}"
                self.calibrated_positions[key] = position
                print(f"✅ Recorded: {zone_display_name} → {position}")

        # Show summary
        print("\n\n" + "="*70)
        print("CALIBRATION COMPLETE")
        print("="*70)
        print(f"\nCalibrated {len(self.calibrated_positions)} zones:")

        for key, (x, y) in self.calibrated_positions.items():
            print(f"  {key:30s} → ({x:6.1f}, {y:6.1f})")

        # Ask to save
        if self.calibrated_positions:
            print("\n" + "="*70)
            save = input("\nSave these values to config.py? [y/N]: ").strip().lower()

            if save == 'y':
                self.update_config()
            else:
                print("\n❌ Calibration NOT saved")
                print("\nCalibrated values (for manual entry):")
                for key, (x, y) in self.calibrated_positions.items():
                    print(f"  {key}: 'center': ({x:.1f}, {y:.1f})")
        else:
            print("\n⚠️  No zones were calibrated")

    def update_config(self):
        """Update config.py with calibrated values"""
        import re

        print("\n🔧 Updating config.py...")

        # Read current config
        with open('config.py', 'r') as f:
            config_content = f.read()

        # Update each calibrated position
        updates_made = 0
        for key, (x, y) in self.calibrated_positions.items():
            zone_category, zone_name = key.split('.')

            # Find the zone definition and update center
            # Pattern: 'zone_name': { ... 'center': (old_x, old_y) ... }
            pattern = rf"('{zone_name}':\s*\{{[^}}]*)'center':\s*\([^)]*\)"
            replacement = rf"\1'center': ({x:.1f}, {y:.1f})"

            new_content = re.sub(pattern, replacement, config_content)

            if new_content != config_content:
                config_content = new_content
                updates_made += 1
                print(f"  ✅ Updated {key}")
            else:
                print(f"  ⚠️  Could not update {key} (pattern not found)")

        # Write updated config
        if updates_made > 0:
            with open('config.py', 'w') as f:
                f.write(config_content)

            print(f"\n✅ Successfully updated {updates_made} zones in config.py")
            print("\n⚠️  IMPORTANT: Restart any running programs to load new config")
        else:
            print("\n❌ No updates were made to config.py")

    def cleanup(self):
        """Clean up resources"""
        print("\n\n🧹 Cleaning up...")
        self.sf.motors.stop()
        self.sf.stop()
        print("✅ Sensor fusion stopped")


def main():
    """Main calibration program"""
    calibrator = None

    try:
        calibrator = ZoneCalibrator()
        calibrator.run_calibration()

    except KeyboardInterrupt:
        print("\n\n⚠️  Calibration interrupted by user")

    except Exception as e:
        print(f"\n\n❌ Error during calibration: {e}")
        import traceback
        traceback.print_exc()

    finally:
        if calibrator:
            calibrator.cleanup()

        print("\n" + "="*70)
        print("Zone calibration complete!")
        print("="*70)


if __name__ == "__main__":
    main()
