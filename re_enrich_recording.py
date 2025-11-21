#!/usr/bin/env python3
"""
re_enrich_recording.py
Re-execute existing movement commands with fresh sensor fusion data

Takes an existing movement recording and re-executes it with:
- Freshly calibrated gyroscope
- New sensor fusion data capture
- Saves as new enriched recording

Use this when:
- You have a good movement sequence but bad sensor data
- IMU wasn't calibrated during original recording
- You want to update reference data without re-teaching movements
"""

import time
import json
import os
from datetime import datetime
from sensor_fusion import SensorFusion
from hardware import MotorController
from utils.logging import DataLogger
from config import DATA_PATHS


class RecordingReEnricher:
    """Re-enrich existing recording with fresh sensor fusion data"""

    def __init__(self, source_filename):
        """
        Initialize re-enricher

        Args:
            source_filename: Existing movement file (e.g., "2.json")
        """
        self.source_filename = source_filename
        self.source_sequence = None
        self.motors = MotorController()
        self.sensor_fusion = None

        # Load source sequence
        self._load_source_sequence()

    def _load_source_sequence(self):
        """Load the source movement sequence"""
        filepath = os.path.join(DATA_PATHS['movements'], self.source_filename)

        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Source file not found: {filepath}")

        with open(filepath, 'r') as f:
            self.source_sequence = json.load(f)

        print(f"✓ Loaded source sequence: {self.source_filename}")
        print(f"  Name: {self.source_sequence.get('name', 'unnamed')}")
        print(f"  Steps: {len(self.source_sequence['steps'])}")
        print(f"  Duration: {self.source_sequence.get('total_duration', 0):.1f}s")

    def re_enrich(self, output_filename=None):
        """
        Re-execute sequence and capture fresh sensor fusion data

        Args:
            output_filename: Output filename (default: source_name + "_enriched.json")

        Returns:
            str: Path to saved enriched recording
        """
        # Determine output filename
        if output_filename is None:
            source_name = self.source_sequence.get('name', 'sequence')
            output_filename = f"{source_name}_enriched.json"

        print("\n" + "="*70)
        print("RE-ENRICHING RECORDING WITH FRESH SENSOR FUSION DATA")
        print("="*70)
        print(f"Source: {self.source_filename}")
        print(f"Output: {output_filename}")
        print("="*70)

        # Initialize sensor fusion with calibration
        print("\n🔧 Initializing sensor fusion...")
        session_name = f"recording_{self.source_sequence.get('name', 'sequence')}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.sensor_fusion = SensorFusion(session_name=session_name, enable_logging=True)

        # Calibrate gyroscope
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
        else:
            print("⚠️  Gyroscope calibration failed")
            user_continue = input("Continue anyway? [y/N]: ").strip().lower()
            if user_continue != 'y':
                print("Aborted.")
                return None

        print("="*70)

        # Start sensor fusion
        print("\n🚀 Starting sensor fusion...")
        self.sensor_fusion.start_sensor_loop(update_rate=10)
        time.sleep(1.0)  # Let it stabilize

        # Reset position to origin
        print("🔄 Resetting position to (0, 0, 0°)...")
        self.sensor_fusion.position = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        time.sleep(0.5)

        print("\n✅ Ready to record!")
        print("\n⚠️  IMPORTANT:")
        print("   Place robot at SAME starting position as original recording")
        print("   Robot should be facing the same direction")
        print("")
        input("Press Enter to start recording with sensor fusion...")

        # Execute sequence and capture data
        enriched_sequence = {
            'name': self.source_sequence.get('name', 'sequence') + '_enriched',
            'created_at': datetime.now().isoformat(),
            'recording_mode': 'sensor_fusion_enabled',
            'source_recording': self.source_filename,
            'steps': [],
            'total_duration': 0,
            'sensor_fusion_session': f"data/sessions/{session_name}.json"
        }

        recording_start_time = time.time()
        cumulative_time = 0

        print("\n" + "="*70)
        print("RECORDING IN PROGRESS")
        print("="*70)

        try:
            for i, step in enumerate(self.source_sequence['steps'], 1):
                print(f"\n[Step {i}/{len(self.source_sequence['steps'])}] "
                      f"{step['action']} @ {step['speed']} PWM for {step['duration']:.1f}s")

                # Capture start position and sensor data
                step_start_time = time.time() - recording_start_time
                start_pos = self.sensor_fusion.get_position()
                start_nav = self.sensor_fusion.get_navigation_data()

                print(f"  Start: ({start_pos['x']:.1f}, {start_pos['y']:.1f}, {start_pos['heading']:.1f}°)")

                # Execute movement
                self._execute_step(step)

                # Capture end position and sensor data
                step_end_time = time.time() - recording_start_time
                end_pos = self.sensor_fusion.get_position()
                end_nav = self.sensor_fusion.get_navigation_data()

                print(f"  End:   ({end_pos['x']:.1f}, {end_pos['y']:.1f}, {end_pos['heading']:.1f}°)")

                # Build enriched step
                enriched_step = {
                    'action': step['action'],
                    'speed': step['speed'],
                    'duration': step['duration'],
                    'start_time': step_start_time,
                    'end_time': step_end_time,
                    'sensor_fusion': {
                        'start_position': {
                            'x': start_pos['x'],
                            'y': start_pos['y'],
                            'heading': start_pos['heading']
                        },
                        'end_position': {
                            'x': end_pos['x'],
                            'y': end_pos['y'],
                            'heading': end_pos['heading']
                        },
                        'sensor_summary': {
                            'navigation': {
                                'front_min': end_nav['front_min'],
                                'back_min': end_nav['back_min'],
                                'left_min': end_nav['left_min'],
                                'right_min': end_nav['right_min'],
                                'obstacles_nearby': end_nav['obstacles_nearby'],
                                'potential_blocks': end_nav['potential_blocks']
                            },
                            'sensors_ok': True,
                            'obstacles_detected': len(end_nav['potential_blocks'])
                        }
                    }
                }

                enriched_sequence['steps'].append(enriched_step)
                cumulative_time = step_end_time

            enriched_sequence['total_duration'] = cumulative_time

            print("\n" + "="*70)
            print("RECORDING COMPLETE")
            print("="*70)
            print(f"Total steps: {len(enriched_sequence['steps'])}")
            print(f"Total duration: {cumulative_time:.1f}s")

        except KeyboardInterrupt:
            print("\n\n⚠️  Recording interrupted by user")
            print("Saving partial recording...")
            enriched_sequence['total_duration'] = cumulative_time
            enriched_sequence['interrupted'] = True

        except Exception as e:
            print(f"\n\n❌ Error during recording: {e}")
            import traceback
            traceback.print_exc()
            self.motors.stop()
            self.sensor_fusion.stop()
            return None

        finally:
            # Stop everything
            self.motors.stop()
            self.sensor_fusion.stop()

        # Save enriched sequence
        output_path = os.path.join(DATA_PATHS['movements'], output_filename)

        try:
            with open(output_path, 'w') as f:
                json.dump(enriched_sequence, f, indent=2)

            print(f"\n✅ Enriched recording saved: {output_path}")

            # Save sensor fusion session
            session_file = self.sensor_fusion.logger.save_session()
            print(f"✅ Sensor fusion session saved: {session_file}")

            return output_path

        except Exception as e:
            print(f"\n❌ Error saving recording: {e}")
            return None

    def _execute_step(self, step):
        """Execute a single movement step"""
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


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(
        description='Re-enrich existing recording with fresh sensor fusion data'
    )
    parser.add_argument('source', type=str,
                       help='Source movement file (e.g., "2.json")')
    parser.add_argument('--output', '-o', type=str,
                       help='Output filename (default: source_enriched.json)')

    args = parser.parse_args()

    print("\n🤖 RECORDING RE-ENRICHMENT TOOL")
    print("="*70)

    try:
        enricher = RecordingReEnricher(args.source)
        result = enricher.re_enrich(args.output)

        if result:
            print("\n" + "="*70)
            print("SUCCESS!")
            print("="*70)
            print(f"New enriched recording: {result}")
            print("\nYou can now use this with smart_replay.py:")
            print(f"  python3 smart_replay.py --sequence {os.path.basename(result)}")
            print("="*70)
        else:
            print("\n❌ Re-enrichment failed")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
