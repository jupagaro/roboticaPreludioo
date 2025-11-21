#!/usr/bin/env python3
"""
smart_replay.py
Minimal Smart Replay with Behavioral Corrections

Features:
- Pre-flight validation (starting position check)
- Heading correction (after each step)
- Position correction (if drift >10cm)
- Crash detection (if stuck, backup and retry)
- Comprehensive logging

Author: Robot de Rescate 2025
"""

import time
import json
import os
from datetime import datetime
from math import sqrt, atan2, degrees, radians, cos, sin
from movement_recorder import MovementRecorder
from sensor_fusion import SensorFusion
from utils.logging import DataLogger
from config import DATA_PATHS


# ============================================================================
# CONFIGURATION
# ============================================================================

THRESHOLDS = {
    'starting_position_tolerance': 15.0,   # cm - max position error at start
    'starting_heading_tolerance': 20.0,    # degrees - max heading error at start
    'heading_correction': 10.0,             # degrees - trigger heading correction
    'position_correction': 10.0,            # cm - trigger position correction
    'stuck_detection': 0.5,                 # 50% - movement must be at least this % of expected
    'backup_distance': 10.0,                # cm - how far to back up when stuck
    'correction_speed': 100,                # PWM - speed for correction movements
}


class SmartReplay:
    """
    Smart replay system with minimal behavioral corrections

    Corrections:
    1. Pre-flight: Validate starting position
    2. Heading: Correct drift >10°
    3. Position: Correct drift >10cm
    4. Crash: Detect stuck, backup and retry
    """

    def __init__(self, sequence_filename, enable_logging=True):
        """
        Initialize smart replay

        Args:
            sequence_filename: Movement sequence to replay (e.g., "2.json")
            enable_logging: Enable session logging
        """
        self.sequence_filename = sequence_filename
        self.enable_logging = enable_logging

        # Load sequence
        self.recorder = MovementRecorder(enable_sensors=True)

        if not self.recorder.load_sequence(sequence_filename):
            raise Exception(f"Failed to load sequence: {sequence_filename}")

        self.sequence = self.recorder.current_sequence

        # Verify it's an enriched recording
        if self.sequence.get('recording_mode') != 'sensor_fusion_enabled':
            raise Exception(
                "This sequence is not enriched with sensor fusion data!\n"
                "Please record with sensors enabled (option 3 in movement_recorder menu)"
            )

        # Sensor fusion (from recorder)
        self.sensor_fusion = self.recorder.sensor_fusion
        self.motors = self.recorder.motors

        # Statistics
        self.stats = {
            'total_steps': len(self.sequence['steps']),
            'heading_corrections': 0,
            'position_corrections': 0,
            'crash_detections': 0,
            'crash_recoveries': 0,
            'steps_completed': 0,
            'corrections_log': []
        }

        print(f"✓ Smart Replay initialized")
        print(f"  Sequence: {self.sequence['name']}")
        print(f"  Steps: {self.stats['total_steps']}")
        print(f"  Duration: {self.sequence['total_duration']:.1f}s")

    # ========================================================================
    # PRE-FLIGHT VALIDATION
    # ========================================================================

    def validate_starting_position(self):
        """
        Validate robot is at the same starting position as recording

        Returns:
            bool: True if valid, False otherwise
        """
        print("\n" + "="*70)
        print("PRE-FLIGHT VALIDATION")
        print("="*70)

        # Get first step's starting position (reference)
        first_step = self.sequence['steps'][0]

        if 'sensor_fusion' not in first_step:
            print("❌ No reference position data in recording")
            return False

        ref_pos = first_step['sensor_fusion']['start_position']

        print(f"\nReference starting position (from recording):")
        print(f"  X: {ref_pos['x']:.1f} cm")
        print(f"  Y: {ref_pos['y']:.1f} cm")
        print(f"  Heading: {ref_pos['heading']:.1f}°")

        # Get current robot position
        print(f"\nChecking current robot position...")
        time.sleep(0.5)  # Let sensors stabilize

        current_pos = self.sensor_fusion.get_position()

        print(f"\nCurrent robot position:")
        print(f"  X: {current_pos['x']:.1f} cm")
        print(f"  Y: {current_pos['y']:.1f} cm")
        print(f"  Heading: {current_pos['heading']:.1f}°")

        # Calculate errors
        pos_error_x = abs(current_pos['x'] - ref_pos['x'])
        pos_error_y = abs(current_pos['y'] - ref_pos['y'])
        pos_error = sqrt(pos_error_x**2 + pos_error_y**2)

        heading_error = abs(current_pos['heading'] - ref_pos['heading'])
        # Normalize heading error
        if heading_error > 180:
            heading_error = 360 - heading_error

        print(f"\nPosition Error: {pos_error:.1f} cm")
        print(f"Heading Error: {heading_error:.1f}°")

        # Validate
        pos_tolerance = THRESHOLDS['starting_position_tolerance']
        heading_tolerance = THRESHOLDS['starting_heading_tolerance']

        position_ok = pos_error <= pos_tolerance
        heading_ok = heading_error <= heading_tolerance

        print(f"\nValidation:")
        print(f"  Position: {'✅ PASS' if position_ok else '❌ FAIL'} "
              f"(tolerance: {pos_tolerance} cm)")
        print(f"  Heading:  {'✅ PASS' if heading_ok else '❌ FAIL'} "
              f"(tolerance: {heading_tolerance}°)")

        if position_ok and heading_ok:
            print(f"\n✅ PRE-FLIGHT PASSED - Ready to replay!")
            print("="*70)
            return True
        else:
            print(f"\n❌ PRE-FLIGHT FAILED")
            print(f"\n⚠️  Robot position doesn't match recording start")
            print(f"\nThis usually happens because:")
            print(f"  - Recording was started AFTER sensor fusion initialized")
            print(f"  - Robot was moved before hitting 'record'")
            print(f"  - Sensor fusion accumulated drift before recording")
            print(f"\nSOLUTION: Auto-reset sensor fusion to recording's coordinate system")
            print(f"This will align the robot's position to match the recording.")
            print("="*70)

            # Offer auto-reset
            reset_choice = input("\nReset sensor fusion to recording start position? [Y/n]: ").strip().lower()

            if reset_choice != 'n':
                # Reset sensor fusion position AND heading
                print(f"\n🔄 Resetting sensor fusion coordinate system...")
                print(f"   Setting position: ({ref_pos['x']:.1f}, {ref_pos['y']:.1f})")
                print(f"   Setting heading: {ref_pos['heading']:.1f}°")

                self.sensor_fusion.position = {
                    'x': ref_pos['x'],
                    'y': ref_pos['y'],
                    'heading': ref_pos['heading']
                }

                # Verify reset
                time.sleep(0.2)
                new_pos = self.sensor_fusion.get_position()

                print(f"\n✅ Sensor fusion coordinate system aligned to recording:")
                print(f"  Position: ({new_pos['x']:.1f}, {new_pos['y']:.1f})")
                print(f"  Heading: {new_pos['heading']:.1f}°")
                print(f"\n  Note: Robot's physical orientation hasn't changed -")
                print(f"  we've just set the coordinate system to match the recording.")
                print(f"\n✅ PRE-FLIGHT PASSED (with auto-reset) - Ready to replay!")
                print("="*70)
                return True
            else:
                print(f"\n❌ Auto-reset declined")
                print(f"\nManual options:")
                print(f"  1. Move robot to reference position physically")
                print(f"  2. Re-run and accept auto-reset")
                print("="*70)
                return False

    # ========================================================================
    # STEP EXECUTION WITH CORRECTIONS
    # ========================================================================

    def execute_step_with_corrections(self, step, step_number):
        """
        Execute a single step with all corrections enabled

        Args:
            step: Step dictionary from sequence
            step_number: Step index (1-based)

        Returns:
            dict: Execution results
        """
        print(f"\n[Step {step_number}/{self.stats['total_steps']}] "
              f"{step['action']} @ {step['speed']} PWM for {step['duration']:.1f}s")

        # Get expected final position
        expected_pos = step['sensor_fusion']['end_position']

        # Capture starting position
        start_pos = self.sensor_fusion.get_position()

        # Execute movement
        self._execute_movement(step)

        # Capture ending position
        end_pos = self.sensor_fusion.get_position()

        # Calculate actual distance moved
        dx = end_pos['x'] - start_pos['x']
        dy = end_pos['y'] - start_pos['y']
        actual_distance = sqrt(dx**2 + dy**2)

        # Calculate expected distance
        exp_dx = expected_pos['x'] - step['sensor_fusion']['start_position']['x']
        exp_dy = expected_pos['y'] - step['sensor_fusion']['start_position']['y']
        expected_distance = sqrt(exp_dx**2 + exp_dy**2)

        print(f"  Position: ({start_pos['x']:.1f}, {start_pos['y']:.1f}) → "
              f"({end_pos['x']:.1f}, {end_pos['y']:.1f})")
        print(f"  Distance: {actual_distance:.1f} cm (expected: {expected_distance:.1f} cm)")

        # === CRASH DETECTION ===
        crashed = False
        if step['action'] in ['forward', 'backward']:  # Only for linear movements
            movement_ratio = actual_distance / expected_distance if expected_distance > 0 else 1.0

            # Primary check: Did we move significantly less than expected?
            movement_insufficient = movement_ratio < THRESHOLDS['stuck_detection']

            if movement_insufficient:
                # Secondary check: Are ultrasonic sensors detecting an obstacle?
                nav_data = self.sensor_fusion.get_navigation_data()

                # Check relevant sensors based on movement direction
                if step['action'] == 'forward':
                    obstacle_detected = nav_data['front_min'] < 10  # Wall within 10cm
                    sensor_direction = "front"
                else:  # backward
                    obstacle_detected = nav_data['back_min'] < 10
                    sensor_direction = "back"

                # CRASH confirmed only if BOTH conditions true
                if obstacle_detected:
                    print(f"  🚨 CRASH DETECTED!")
                    print(f"     Movement: {movement_ratio*100:.0f}% of expected")
                    print(f"     {sensor_direction.capitalize()} sensor: {nav_data[sensor_direction + '_min']:.1f} cm (obstacle confirmed)")
                    crashed = True

                    # Try recovery
                    recovery_success = self._handle_crash_recovery(step)

                    if recovery_success:
                        # Re-capture position after recovery
                        end_pos = self.sensor_fusion.get_position()
                    else:
                        # Recovery failed, stop execution
                        raise Exception("Robot stuck - unable to recover")
                else:
                    # Movement slow but no obstacle = likely wheel slip or piece detection
                    print(f"  ⚠️  Low movement ({movement_ratio*100:.0f}% of expected)")
                    print(f"     But no obstacle detected - likely wheel slip or detected game piece")
                    print(f"     Continuing without crash recovery...")

        # === HEADING CORRECTION ===
        heading_error = expected_pos['heading'] - end_pos['heading']

        # Normalize to -180 to 180
        while heading_error > 180:
            heading_error -= 360
        while heading_error < -180:
            heading_error += 360

        print(f"  Heading: {end_pos['heading']:.1f}° (expected: {expected_pos['heading']:.1f}°, "
              f"error: {heading_error:+.1f}°)")

        heading_corrected = False
        if abs(heading_error) > THRESHOLDS['heading_correction']:
            print(f"  🔧 Correcting heading ({heading_error:+.1f}°)...")
            self._correct_heading(heading_error)
            heading_corrected = True
            self.stats['heading_corrections'] += 1

            # Update position after correction
            end_pos = self.sensor_fusion.get_position()

        # === POSITION CORRECTION ===
        pos_error_x = expected_pos['x'] - end_pos['x']
        pos_error_y = expected_pos['y'] - end_pos['y']
        pos_error = sqrt(pos_error_x**2 + pos_error_y**2)

        print(f"  Position error: {pos_error:.1f} cm")

        position_corrected = False
        if pos_error > THRESHOLDS['position_correction']:
            print(f"  🔧 Correcting position ({pos_error:.1f} cm off)...")
            self._correct_position(end_pos, expected_pos)
            position_corrected = True
            self.stats['position_corrections'] += 1

            # Update position after correction
            end_pos = self.sensor_fusion.get_position()

        # Log corrections
        if crashed or heading_corrected or position_corrected:
            self.stats['corrections_log'].append({
                'step': step_number,
                'crashed': crashed,
                'heading_corrected': heading_corrected,
                'position_corrected': position_corrected,
                'heading_error': heading_error,
                'position_error': pos_error
            })

        self.stats['steps_completed'] += 1

        return {
            'start_position': start_pos,
            'end_position': end_pos,
            'crashed': crashed,
            'heading_corrected': heading_corrected,
            'position_corrected': position_corrected
        }

    def _execute_movement(self, step):
        """Execute the basic movement command"""
        action = step['action']
        speed = step['speed']
        duration = step['duration']

        # Execute action
        if action == 'forward':
            self.motors.move_forward(speed)
        elif action == 'backward':
            self.motors.move_backward(speed)
        elif action == 'spin_left':
            self.motors.spin_left(speed)
        elif action == 'spin_right':
            self.motors.spin_right(speed)
        elif action == 'turn_left':
            self.motors.turn_left(speed)
        elif action == 'turn_right':
            self.motors.turn_right(speed)
        elif action == 'stop':
            self.motors.stop()

        # Wait for duration
        time.sleep(duration)

        # Stop motors
        self.motors.stop()
        time.sleep(0.1)

    def _handle_crash_recovery(self, step):
        """
        Handle crash: back up and retry step once

        Returns:
            bool: True if recovery succeeded, False otherwise
        """
        self.stats['crash_detections'] += 1

        print(f"  🔄 Attempting recovery...")

        # Back up
        backup_distance = THRESHOLDS['backup_distance']
        backup_duration = backup_distance / 10.0  # Rough estimate: ~10cm/s at speed 100

        print(f"  ⬅️  Backing up {backup_distance} cm...")
        self.motors.move_backward(THRESHOLDS['correction_speed'])
        time.sleep(backup_duration)
        self.motors.stop()
        time.sleep(0.2)

        # Retry step
        print(f"  🔄 Retrying step...")
        start_pos_retry = self.sensor_fusion.get_position()

        self._execute_movement(step)

        end_pos_retry = self.sensor_fusion.get_position()

        # Check if retry succeeded
        dx = end_pos_retry['x'] - start_pos_retry['x']
        dy = end_pos_retry['y'] - start_pos_retry['y']
        retry_distance = sqrt(dx**2 + dy**2)

        # Calculate expected distance for this step
        exp_start = step['sensor_fusion']['start_position']
        exp_end = step['sensor_fusion']['end_position']
        exp_dx = exp_end['x'] - exp_start['x']
        exp_dy = exp_end['y'] - exp_start['y']
        expected_distance = sqrt(exp_dx**2 + exp_dy**2)

        retry_ratio = retry_distance / expected_distance if expected_distance > 0 else 0

        if retry_ratio >= THRESHOLDS['stuck_detection']:
            print(f"  ✅ Recovery successful! (moved {retry_distance:.1f} cm)")
            self.stats['crash_recoveries'] += 1
            return True
        else:
            print(f"  ❌ Recovery failed (only moved {retry_distance:.1f} cm)")
            return False

    def _correct_heading(self, heading_error):
        """
        Correct heading error by spinning

        Args:
            heading_error: Error in degrees (positive = turn left, negative = turn right)
        """
        # Calculate spin duration (rough estimate: ~60°/s at speed 100)
        spin_duration = abs(heading_error) / 60.0

        if heading_error > 0:
            # Need to turn left (counter-clockwise)
            self.motors.spin_left(THRESHOLDS['correction_speed'])
        else:
            # Need to turn right (clockwise)
            self.motors.spin_right(THRESHOLDS['correction_speed'])

        time.sleep(spin_duration)
        self.motors.stop()
        time.sleep(0.1)

        # Verify correction
        corrected_pos = self.sensor_fusion.get_position()
        print(f"     New heading: {corrected_pos['heading']:.1f}°")

    def _correct_position(self, current_pos, expected_pos):
        """
        Correct position error by moving toward expected position

        Args:
            current_pos: Current position dict
            expected_pos: Expected position dict
        """
        # Calculate correction vector
        dx = expected_pos['x'] - current_pos['x']
        dy = expected_pos['y'] - current_pos['y']
        correction_distance = sqrt(dx**2 + dy**2)

        # Calculate angle to target (in global frame)
        target_angle = degrees(atan2(dy, dx))

        # Calculate how much to turn (relative to current heading)
        turn_angle = target_angle - current_pos['heading']

        # Normalize turn angle
        while turn_angle > 180:
            turn_angle -= 360
        while turn_angle < -180:
            turn_angle += 360

        print(f"     Need to move {correction_distance:.1f} cm at angle {target_angle:.1f}°")
        print(f"     Turning {turn_angle:+.1f}° first...")

        # Step 1: Turn to target angle
        if abs(turn_angle) > 5:  # Only turn if significant
            turn_duration = abs(turn_angle) / 60.0

            if turn_angle > 0:
                self.motors.spin_left(THRESHOLDS['correction_speed'])
            else:
                self.motors.spin_right(THRESHOLDS['correction_speed'])

            time.sleep(turn_duration)
            self.motors.stop()
            time.sleep(0.1)

        # Step 2: Move forward to correct position
        move_duration = correction_distance / 10.0  # Rough estimate

        print(f"     Moving forward {correction_distance:.1f} cm...")
        self.motors.move_forward(THRESHOLDS['correction_speed'])
        time.sleep(move_duration)
        self.motors.stop()
        time.sleep(0.1)

        # Step 3: Turn back to expected heading (optional, for precision)
        current_heading_after_move = self.sensor_fusion.get_position()['heading']
        heading_correction = expected_pos['heading'] - current_heading_after_move

        while heading_correction > 180:
            heading_correction -= 360
        while heading_correction < -180:
            heading_correction += 360

        if abs(heading_correction) > 5:
            print(f"     Adjusting final heading ({heading_correction:+.1f}°)...")
            turn_duration = abs(heading_correction) / 60.0

            if heading_correction > 0:
                self.motors.spin_left(THRESHOLDS['correction_speed'])
            else:
                self.motors.spin_right(THRESHOLDS['correction_speed'])

            time.sleep(turn_duration)
            self.motors.stop()
            time.sleep(0.1)

        # Verify
        final_pos = self.sensor_fusion.get_position()
        print(f"     New position: ({final_pos['x']:.1f}, {final_pos['y']:.1f})")

    # ========================================================================
    # MAIN REPLAY LOOP
    # ========================================================================

    def execute(self):
        """
        Execute the smart replay with all corrections

        Returns:
            bool: True if successful, False otherwise
        """
        print("\n" + "="*70)
        print(f"SMART REPLAY: {self.sequence['name']}")
        print("="*70)
        print("Features enabled:")
        print("  ✅ Pre-flight validation")
        print("  ✅ Heading correction (threshold: 10°)")
        print("  ✅ Position correction (threshold: 10cm)")
        print("  ✅ Crash detection & recovery")
        print("="*70)

        # Pre-flight check
        if not self.validate_starting_position():
            print("\n❌ Replay cancelled - Pre-flight validation failed")
            return False

        input("\nPress Enter to start replay...")

        # Calibrate gyroscope before starting
        print("\n" + "="*70)
        print("GYROSCOPE CALIBRATION")
        print("="*70)
        print("⚠️  Robot must be COMPLETELY STILL during calibration")
        print("   Do not touch or move the robot!")
        print("")
        input("Press Enter when robot is still and ready...")

        print("\n🔧 Calibrating gyroscope bias...")
        print("   This will take ~10 seconds...")

        calibration_success = self.sensor_fusion.imu.calibrate_gyro_z(samples=100)

        if calibration_success:
            print("✅ Gyroscope calibrated successfully!")
            print(f"   Bias offset: {self.sensor_fusion.imu.gyro_z_offset:.3f} °/s")
            print("   Heading drift should now be minimal.")
        else:
            print("⚠️  Gyroscope calibration failed - continuing anyway")
            print("   (Heading may drift during replay)")

        print("="*70)
        time.sleep(0.5)

        # Start sensor fusion
        print("\n🚀 Starting sensor fusion...")
        self.sensor_fusion.start_sensor_loop(update_rate=10)
        time.sleep(0.5)

        # Setup logging
        session_name = f"smart_replay_{self.sequence['name']}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        data_logger = DataLogger(session_name)
        self.sensor_fusion.enable_logging = True
        self.sensor_fusion.logger = data_logger

        playback_start = time.time()

        try:
            # Execute each step
            for i, step in enumerate(self.sequence['steps'], 1):
                result = self.execute_step_with_corrections(step, i)

                time.sleep(0.1)  # Small pause between steps

            duration = time.time() - playback_start

            # Final report
            self._print_final_report(duration)

            return True

        except KeyboardInterrupt:
            print("\n\n⚠️  Replay interrupted by user")
            self.motors.stop()
            return False

        except Exception as e:
            print(f"\n\n❌ Error during replay: {e}")
            import traceback
            traceback.print_exc()
            self.motors.stop()
            return False

        finally:
            # Cleanup
            self.motors.stop()
            self.sensor_fusion.stop()

            # Save session
            session_file = data_logger.save_session()
            print(f"\n✓ Session saved: {session_file}")

    def _print_final_report(self, duration):
        """Print final statistics report"""
        print("\n" + "="*70)
        print("SMART REPLAY COMPLETE")
        print("="*70)
        print(f"Duration: {duration:.1f}s")
        print(f"Steps completed: {self.stats['steps_completed']}/{self.stats['total_steps']}")
        print(f"\nCorrections applied:")
        print(f"  Heading corrections: {self.stats['heading_corrections']}")
        print(f"  Position corrections: {self.stats['position_corrections']}")
        print(f"  Crash detections: {self.stats['crash_detections']}")
        print(f"  Crash recoveries: {self.stats['crash_recoveries']}")

        if self.stats['corrections_log']:
            print(f"\nCorrection Summary (last 5):")
            for correction in self.stats['corrections_log'][-5:]:
                print(f"  Step {correction['step']}: ", end="")
                if correction['crashed']:
                    print("CRASH ", end="")
                if correction['heading_corrected']:
                    print(f"HEADING({correction['heading_error']:+.1f}°) ", end="")
                if correction['position_corrected']:
                    print(f"POSITION({correction['position_error']:.1f}cm)", end="")
                print()

        print("="*70)


# ============================================================================
# CLI INTERFACE
# ============================================================================

def main():
    """Main entry point for smart replay"""
    import argparse

    parser = argparse.ArgumentParser(description='Smart Replay with Minimal Corrections')
    parser.add_argument('--sequence', '-s', type=str,
                       help='Sequence filename (e.g., "2.json")')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List available sequences')

    args = parser.parse_args()

    if args.list:
        # List available sequences
        movements_dir = DATA_PATHS['movements']
        files = [f for f in os.listdir(movements_dir) if f.endswith('.json')]

        print("\nAvailable sequences:")
        for i, filename in enumerate(sorted(files), 1):
            filepath = os.path.join(movements_dir, filename)
            try:
                with open(filepath, 'r') as f:
                    data = json.load(f)
                mode = data.get('recording_mode', 'simple')
                steps = len(data.get('steps', []))
                duration = data.get('total_duration', 0)

                status = "✅" if mode == 'sensor_fusion_enabled' else "⚠️"
                print(f"{i}. {status} {filename:20} | {steps:2d} steps | {duration:5.1f}s | {mode}")
            except:
                print(f"{i}. ❌ {filename:20} | ERROR loading")

        print("\n✅ = Enriched (compatible)")
        print("⚠️ = Simple (not compatible with smart replay)")
        return

    # Get sequence filename
    sequence_file = args.sequence

    if not sequence_file:
        # Interactive selection
        print("\n" + "="*70)
        print("SMART REPLAY - Minimal Behavioral Corrections")
        print("="*70)

        sequence_file = input("\nEnter sequence filename (e.g., '2.json'): ").strip()

        if not sequence_file:
            print("No sequence specified.")
            return

    # Execute smart replay
    try:
        replay = SmartReplay(sequence_file)
        replay.execute()

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
