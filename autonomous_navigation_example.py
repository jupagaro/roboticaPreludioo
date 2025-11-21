#!/usr/bin/env python3
"""
autonomous_navigation_example.py
Practical examples of using sensor fusion for autonomous competition runs

This demonstrates how to use sensor fusion (NOT AI, but sensor data fusion)
for intelligent navigation during the rescue robot competition.

HOW IT WORKS:
1. IMU (gyroscope) → Fast, accurate heading (short-term)
2. Motor odometry → Position estimate from wheel speeds
3. Ultrasonics (4x) → Obstacle detection around robot
4. Fusion → Weighted combination gives best estimate
"""

import time
from sensor_fusion import SensorFusion
from config import COMPETITION_ZONES, FIELD_DIMENSIONS


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def initialize_sensor_fusion_with_calibration(session_name):
    """
    Initialize sensor fusion with automatic gyro calibration

    Args:
        session_name: Name for the sensor fusion session

    Returns:
        SensorFusion: Initialized and calibrated sensor fusion object
    """
    print("\n" + "="*70)
    print("INITIALIZING SENSOR FUSION")
    print("="*70)

    # Create sensor fusion
    sf = SensorFusion(session_name=session_name)

    # Calibrate gyroscope
    print("\n⚠️  Gyroscope calibration required")
    print("   Robot must be COMPLETELY STILL!")
    input("\nPress Enter when robot is still and ready...")

    print("\n🔧 Calibrating gyroscope bias...")
    print("   This will take ~10 seconds...")

    calibration_success = sf.imu.calibrate_gyro_z(samples=100)

    if calibration_success:
        print(f"✅ Gyroscope calibrated! Bias: {sf.imu.gyro_z_offset:.3f} °/s")
    else:
        print("⚠️  Calibration failed - heading may drift")

    # Start sensor fusion loop
    print("\n🚀 Starting sensor fusion loop...")
    sf.start_sensor_loop(update_rate=10)
    time.sleep(0.5)

    print("✅ Sensor fusion ready!")
    print("="*70)

    return sf


# ============================================================================
# EXAMPLE 1: BASIC SENSOR MONITORING
# ============================================================================

def example_1_basic_monitoring():
    """
    Simplest example: Just monitor what sensors see
    Good for understanding sensor fusion output
    """
    print("\n" + "="*70)
    print("EXAMPLE 1: BASIC SENSOR MONITORING")
    print("="*70)
    print("This will show real-time position and obstacle data")
    print("Move the robot around manually to see sensor fusion in action")
    print("Press Ctrl+C to stop\n")

    # Initialize sensor fusion with calibration
    sf = initialize_sensor_fusion_with_calibration("example1_monitoring")

    try:
        while True:
            # Get current position estimate
            pos = sf.get_position()

            # Get obstacle data
            nav_data = sf.get_navigation_data()

            # Display
            print(f"\rPos: ({pos['x']:6.1f}, {pos['y']:6.1f}) | "
                  f"Heading: {pos['heading']:6.1f}° | "
                  f"Front: {nav_data['front_min']:5.1f}cm | "
                  f"Blocks: {len(nav_data['potential_blocks'])}", end="")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\nStopping...")

    finally:
        sf.motors.stop()
        sf.stop()
        print("Example 1 complete!")


# ============================================================================
# EXAMPLE 2: AUTONOMOUS NAVIGATION TO POINT
# ============================================================================

def example_2_navigate_to_point():
    """
    Navigate to a specific coordinate using sensor fusion
    Demonstrates autonomous point-to-point navigation with obstacle avoidance
    """
    print("\n" + "="*70)
    print("EXAMPLE 2: NAVIGATE TO SPECIFIC POINT")
    print("="*70)

    # Get target from user
    print("\nEnter target coordinates:")
    target_x = float(input("  X (cm) [default=50]: ") or "50")
    target_y = float(input("  Y (cm) [default=30]: ") or "30")

    print(f"\nNavigating to ({target_x}, {target_y})...")
    print("Robot will:")
    print("  - Turn toward target")
    print("  - Move forward while avoiding obstacles")
    print("  - Stop when within 5cm of target")

    input("\nPress Enter to start...")

    # Initialize with calibration
    sf = initialize_sensor_fusion_with_calibration("example2_navigation")

    try:
        iteration = 0
        max_iterations = 300  # 30 seconds max at 10Hz

        while iteration < max_iterations:
            # Navigate toward point
            status = sf.navigate_to_point(target_x, target_y, speed=120)

            # Show status
            pos = sf.get_position()
            print(f"[{iteration}] Status: {status['status']:20s} | "
                  f"Distance: {status['distance']:5.1f}cm | "
                  f"Pos: ({pos['x']:5.1f}, {pos['y']:5.1f})")

            # Check if arrived
            if status['status'] == 'arrived':
                print(f"\n✅ ARRIVED! Final position: ({pos['x']:.1f}, {pos['y']:.1f})")
                break

            iteration += 1
            time.sleep(0.1)

        if iteration >= max_iterations:
            print("\n⚠️ Timeout reached")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        sf.motors.stop()
        sf.stop()
        print("Example 2 complete!")


# ============================================================================
# EXAMPLE 3: BLOCK DETECTION AND APPROACH
# ============================================================================

def example_3_find_and_approach_block():
    """
    Search for blocks using ultrasonics and approach them
    Demonstrates block detection capability of sensor fusion
    """
    print("\n" + "="*70)
    print("EXAMPLE 3: BLOCK DETECTION & APPROACH")
    print("="*70)
    print("Robot will:")
    print("  1. Wander around using reactive navigation")
    print("  2. Detect blocks 5-35cm away")
    print("  3. Approach detected blocks slowly")
    print("  4. Stop when very close (< 8cm)")
    print("\nPlace a block in front of the robot to test!\n")

    input("Press Enter to start...")

    # Initialize with calibration
    sf = initialize_sensor_fusion_with_calibration("example3_block_detection")

    try:
        search_mode = True
        approach_start = None

        while True:
            nav_data = sf.get_navigation_data()
            pos = sf.get_position()

            if search_mode:
                # SEARCH MODE: Wander until block detected
                print(f"\r🔍 SEARCHING... Pos: ({pos['x']:5.1f}, {pos['y']:5.1f}) | "
                      f"Obstacles: {nav_data['obstacles_nearby']}", end="")

                # Check for blocks
                if nav_data['potential_blocks']:
                    print(f"\n\n🎯 BLOCK DETECTED!")
                    for block in nav_data['potential_blocks']:
                        print(f"  - Sensor: {block['sensor']}, "
                              f"Distance: {block['distance']}cm, "
                              f"Confidence: {block['confidence']}")

                    search_mode = False
                    approach_start = time.time()
                    continue

                # Keep wandering
                sf.reactive_navigation(base_speed=100)

            else:
                # APPROACH MODE: Move toward block
                elapsed = time.time() - approach_start

                print(f"\r🚶 APPROACHING... Front: {nav_data['front_min']:5.1f}cm | "
                      f"Time: {elapsed:4.1f}s", end="")

                # Check if very close
                if nav_data['front_min'] < 8:
                    print(f"\n\n✅ REACHED BLOCK! Distance: {nav_data['front_min']:.1f}cm")
                    print("In competition, you would activate gripper here!")
                    sf.motors.stop()
                    break

                # Check if lost block
                if not nav_data['potential_blocks'] and elapsed > 5:
                    print("\n\n⚠️ Lost sight of block, returning to search mode...")
                    search_mode = True
                    continue

                # Move forward slowly
                sf.motors.move_forward(60)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        sf.motors.stop()
        sf.stop()
        print("Example 3 complete!")


# ============================================================================
# EXAMPLE 4: COMPETITION ZONE NAVIGATION
# ============================================================================

def example_4_visit_competition_zones():
    """
    Navigate to predefined competition zones
    Demonstrates use of COMPETITION_ZONES from config.py
    """
    print("\n" + "="*70)
    print("EXAMPLE 4: VISIT COMPETITION ZONES")
    print("="*70)
    print("Robot will navigate to hospital zones in sequence:")
    print("  1. Waiting Room")
    print("  2. Consultation Room")
    print("  3. Parking")
    print()

    input("Press Enter to start tour...")

    # Initialize with calibration
    sf = initialize_sensor_fusion_with_calibration("example4_zone_tour")

    # Define tour stops
    tour_stops = [
        ("Waiting Room", COMPETITION_ZONES['hospital']['waiting_room']['center']),
        ("Consultation Room", COMPETITION_ZONES['hospital']['consultation_room']['center']),
        ("Parking", COMPETITION_ZONES['hospital']['parking']['center'])
    ]

    try:
        for zone_name, (target_x, target_y) in tour_stops:
            print(f"\n{'='*70}")
            print(f"Navigating to: {zone_name} ({target_x}, {target_y})")
            print('='*70)

            # Navigate to this zone
            while True:
                status = sf.navigate_to_point(target_x, target_y, speed=150)
                pos = sf.get_position()

                print(f"\rStatus: {status['status']:20s} | "
                      f"Distance: {status['distance']:5.1f}cm", end="")

                if status['status'] == 'arrived':
                    print(f"\n✅ Arrived at {zone_name}!")
                    print(f"Final position: ({pos['x']:.1f}, {pos['y']:.1f})")
                    time.sleep(2)  # Pause at zone
                    break

                time.sleep(0.1)

        print("\n\n🏁 Tour complete!")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        sf.motors.stop()
        sf.stop()
        print("Example 4 complete!")


# ============================================================================
# EXAMPLE 5: FULL AUTONOMOUS MISSION
# ============================================================================

def example_5_autonomous_mission():
    """
    Complete autonomous mission combining all sensor fusion features
    This is a realistic competition scenario
    """
    print("\n" + "="*70)
    print("EXAMPLE 5: FULL AUTONOMOUS RESCUE MISSION")
    print("="*70)
    print("Mission objectives:")
    print("  1. Navigate to search zone")
    print("  2. Find and approach blocks")
    print("  3. Deliver to consultation room")
    print("  4. Return to parking")
    print()

    input("Press Enter to start mission...")

    # Initialize with calibration
    sf = initialize_sensor_fusion_with_calibration("example5_full_mission")

    blocks_found = 0
    mission_phases = [
        "NAVIGATE_TO_SEARCH",
        "SEARCH_FOR_BLOCKS",
        "APPROACH_BLOCK",
        "DELIVER",
        "RETURN_TO_PARKING",
        "COMPLETE"
    ]
    current_phase = 0

    try:
        while current_phase < len(mission_phases) - 1:
            phase_name = mission_phases[current_phase]
            nav_data = sf.get_navigation_data()
            pos = sf.get_position()

            if phase_name == "NAVIGATE_TO_SEARCH":
                print(f"\n{'='*70}")
                print("PHASE 1: Navigate to search zone (40, 40)")
                print('='*70)

                status = sf.navigate_to_point(40, 40, speed=150)

                if status['status'] == 'arrived':
                    print("✅ Arrived at search zone!")
                    current_phase += 1

            elif phase_name == "SEARCH_FOR_BLOCKS":
                print(f"\r🔍 Searching for blocks... Pos: ({pos['x']:5.1f}, {pos['y']:5.1f})", end="")

                if nav_data['potential_blocks']:
                    print(f"\n🎯 Block detected!")
                    current_phase += 1
                else:
                    sf.reactive_navigation(base_speed=100)

            elif phase_name == "APPROACH_BLOCK":
                print(f"\r🚶 Approaching block... Front: {nav_data['front_min']:5.1f}cm", end="")

                if nav_data['front_min'] < 8:
                    print("\n✅ Block reached! (Gripper would activate)")
                    blocks_found += 1
                    current_phase += 1
                else:
                    sf.motors.move_forward(60)

            elif phase_name == "DELIVER":
                print(f"\n{'='*70}")
                print("PHASE 3: Delivering to consultation room")
                print('='*70)

                target = COMPETITION_ZONES['hospital']['consultation_room']['center']
                status = sf.navigate_to_point(target[0], target[1], speed=150)

                if status['status'] == 'arrived':
                    print("✅ Delivered to consultation room!")
                    print("(Release block here)")
                    current_phase += 1

            elif phase_name == "RETURN_TO_PARKING":
                print(f"\n{'='*70}")
                print("PHASE 4: Returning to parking")
                print('='*70)

                target = COMPETITION_ZONES['hospital']['parking']['center']
                status = sf.navigate_to_point(target[0], target[1], speed=150)

                if status['status'] == 'arrived':
                    print("✅ Parked!")
                    current_phase += 1

            time.sleep(0.1)

        print(f"\n\n🎉 MISSION COMPLETE!")
        print(f"Blocks found and delivered: {blocks_found}")

    except KeyboardInterrupt:
        print("\n\nMission interrupted")

    finally:
        sf.motors.stop()
        sf.stop()
        print("Example 5 complete!")


# ============================================================================
# MAIN MENU
# ============================================================================

def main():
    """Main menu for sensor fusion examples"""
    while True:
        print("\n" + "="*70)
        print("SENSOR FUSION EXAMPLES - Autonomous Navigation")
        print("="*70)
        print("\nWhat is Sensor Fusion?")
        print("  - Combines IMU + Motor Odometry + Ultrasonics")
        print("  - NOT AI, just math (complementary filter)")
        print("  - Estimates position, heading, and obstacles")
        print()
        print("Examples:")
        print("  1. Basic Monitoring - See sensor fusion output")
        print("  2. Navigate to Point - Autonomous point-to-point")
        print("  3. Block Detection - Find and approach blocks")
        print("  4. Visit Zones - Tour competition zones")
        print("  5. Full Mission - Complete autonomous rescue")
        print()
        print("  0. Exit")
        print("="*70)

        choice = input("\nSelect example [1-5, 0=exit]: ").strip()

        if choice == '1':
            example_1_basic_monitoring()
        elif choice == '2':
            example_2_navigate_to_point()
        elif choice == '3':
            example_3_find_and_approach_block()
        elif choice == '4':
            example_4_visit_competition_zones()
        elif choice == '5':
            example_5_autonomous_mission()
        elif choice == '0':
            print("Goodbye!")
            break
        else:
            print("Invalid choice")


if __name__ == "__main__":
    print("\n🤖 AUTONOMOUS NAVIGATION WITH SENSOR FUSION")

    main()