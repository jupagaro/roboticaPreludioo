#!/usr/bin/env python3
"""
diagnose_imu.py
Deep diagnostics for MPU6050 IMU sensor
Tests I2C communication and sensor initialization
"""

import sys
import subprocess
import time

sys.path.insert(0, '/home/pi/roboticaPreludioo')

print("=" * 70)
print("MPU6050 IMU DIAGNOSTICS")
print("=" * 70)

# Test 1: Check if I2C is enabled
print("\n[TEST 1] Checking I2C interface...")
print("-" * 70)

try:
    import glob
    i2c_devices = glob.glob('/dev/i2c*')

    if i2c_devices:
        print("✅ I2C devices found:")
        for device in i2c_devices:
            print(f"   {device}")
    else:
        print("❌ No I2C devices found!")
        print("\n   I2C is likely disabled. Enable it by running:")
        print("   sudo raspi-config")
        print("   Then: Interface Options -> I2C -> Enable")
        print("   Or run: sudo raspi-config nonint do_i2c 0")
        sys.exit(1)
except Exception as e:
    print(f"❌ Error checking I2C: {e}")
    sys.exit(1)

# Test 2: Check if i2c-tools is installed
print("\n[TEST 2] Checking i2c-tools installation...")
print("-" * 70)

try:
    result = subprocess.run(['which', 'i2cdetect'], capture_output=True, text=True)
    if result.returncode == 0:
        print("✅ i2c-tools installed")
    else:
        print("⚠️ i2c-tools not found. Installing...")
        print("   Run: sudo apt-get install -y i2c-tools")
        # Try to detect without i2cdetect
except Exception as e:
    print(f"⚠️ Could not verify i2c-tools: {e}")

# Test 3: Scan I2C bus for MPU6050
print("\n[TEST 3] Scanning I2C bus for devices...")
print("-" * 70)

try:
    result = subprocess.run(['i2cdetect', '-y', '1'], capture_output=True, text=True)
    if result.returncode == 0:
        print("I2C Bus 1 scan results:")
        print(result.stdout)

        # Check for MPU6050 at address 0x68
        if '68' in result.stdout:
            print("✅ MPU6050 detected at address 0x68!")
        elif '69' in result.stdout:
            print("✅ MPU6050 detected at address 0x69!")
            print("   Note: Your sensor uses alternate address 0x69")
            print("   You may need to update config.py IMU_CONFIG['address'] = 0x69")
        else:
            print("❌ No device found at 0x68 or 0x69")
            print("\n   Common causes:")
            print("   1. MPU6050 not connected to I2C pins")
            print("      - SDA should connect to GPIO2 (pin 3)")
            print("      - SCL should connect to GPIO3 (pin 5)")
            print("   2. No power to MPU6050 (needs 3.3V or 5V)")
            print("   3. Loose wiring or bad connections")
            print("   4. Defective sensor module")
    else:
        print("❌ Could not scan I2C bus")
        print("   Error:", result.stderr)
except FileNotFoundError:
    print("⚠️ i2cdetect command not found")
    print("   Install with: sudo apt-get install -y i2c-tools")
    print("\n   Continuing with Python I2C test...")
except Exception as e:
    print(f"⚠️ Error scanning I2C bus: {e}")

# Test 4: Try to connect using Python smbus
print("\n[TEST 4] Testing Python I2C communication...")
print("-" * 70)

try:
    import smbus

    bus = smbus.SMBus(1)
    addresses_to_try = [0x68, 0x69]

    found_address = None

    for address in addresses_to_try:
        try:
            print(f"\nTrying address 0x{address:02X}...")

            # Try to read WHO_AM_I register (0x75)
            who_am_i = bus.read_byte_data(address, 0x75)
            print(f"  WHO_AM_I register: 0x{who_am_i:02X}")

            if who_am_i == 0x68:
                print(f"  ✅ MPU6050 confirmed at address 0x{address:02X}!")
                found_address = address
                break
            else:
                print(f"  ⚠️ Unexpected WHO_AM_I value")

        except OSError as e:
            print(f"  ❌ No response at 0x{address:02X} (OSError: {e})")
        except Exception as e:
            print(f"  ❌ Error at 0x{address:02X}: {e}")

    if found_address:
        print(f"\n✅ MPU6050 successfully detected at 0x{found_address:02X}")

        # Test initialization
        print("\n[TEST 5] Testing sensor initialization...")
        print("-" * 70)

        try:
            # Wake up MPU6050 (clear sleep bit)
            bus.write_byte_data(found_address, 0x6B, 0)
            time.sleep(0.1)

            # Read power management register
            pwr_mgmt = bus.read_byte_data(found_address, 0x6B)
            print(f"Power Management register: 0x{pwr_mgmt:02X}")

            if pwr_mgmt & 0x40:
                print("  ⚠️ Sensor is in sleep mode")
            else:
                print("  ✅ Sensor is awake")

            # Try reading some data
            print("\n[TEST 6] Reading sensor data...")
            print("-" * 70)

            # Read accelerometer
            accel_x_h = bus.read_byte_data(found_address, 0x3B)
            accel_x_l = bus.read_byte_data(found_address, 0x3C)
            accel_x = (accel_x_h << 8) | accel_x_l
            if accel_x > 32768:
                accel_x -= 65536

            # Read gyroscope
            gyro_z_h = bus.read_byte_data(found_address, 0x47)
            gyro_z_l = bus.read_byte_data(found_address, 0x48)
            gyro_z = (gyro_z_h << 8) | gyro_z_l
            if gyro_z > 32768:
                gyro_z -= 65536

            # Read temperature
            temp_h = bus.read_byte_data(found_address, 0x41)
            temp_l = bus.read_byte_data(found_address, 0x42)
            temp_raw = (temp_h << 8) | temp_l
            if temp_raw > 32768:
                temp_raw -= 65536
            temp_c = (temp_raw / 340.0) + 36.53

            print(f"Accelerometer X (raw): {accel_x}")
            print(f"Gyroscope Z (raw): {gyro_z}")
            print(f"Temperature: {temp_c:.1f}°C")

            if temp_c < 0 or temp_c > 60:
                print("\n⚠️ Temperature reading seems unusual")
            else:
                print("\n✅ Sensor data looks reasonable!")

            print("\n" + "=" * 70)
            print("DIAGNOSTIC SUMMARY")
            print("=" * 70)
            print(f"\n✅ MPU6050 is working at address 0x{found_address:02X}")
            print("\nYour config.py should have:")
            print(f"IMU_CONFIG = {{")
            print(f"    'address': 0x{found_address:02X},")
            print(f"    'bus': 1")
            print(f"}}")

        except Exception as e:
            print(f"\n❌ Error during sensor tests: {e}")
            import traceback
            traceback.print_exc()
    else:
        print("\n" + "=" * 70)
        print("DIAGNOSTIC SUMMARY")
        print("=" * 70)
        print("\n❌ MPU6050 not detected on I2C bus")
        print("\nTroubleshooting steps:")
        print("\n1. CHECK WIRING:")
        print("   MPU6050 -> Raspberry Pi")
        print("   VCC  -> 3.3V (pin 1) or 5V (pin 2)")
        print("   GND  -> GND (pin 6, 9, 14, 20, 25, 30, 34, or 39)")
        print("   SDA  -> GPIO2 (pin 3)")
        print("   SCL  -> GPIO3 (pin 5)")
        print("\n2. CHECK POWER:")
        print("   - Measure voltage between VCC and GND (should be 3.3V or 5V)")
        print("   - Check for loose connections")
        print("\n3. CHECK I2C CONNECTIONS:")
        print("   - Verify SDA and SCL are not swapped")
        print("   - Check for continuity with multimeter")
        print("\n4. TRY DIFFERENT MPU6050 MODULE:")
        print("   - Some modules are defective or use different addresses")

except ImportError:
    print("❌ Python smbus library not found!")
    print("   Install with: sudo apt-get install -y python3-smbus")
except Exception as e:
    print(f"❌ Unexpected error: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "=" * 70)
print("DIAGNOSTIC COMPLETE")
print("=" * 70)
