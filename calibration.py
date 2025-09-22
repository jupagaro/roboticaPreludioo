#!/usr/bin/env python3
"""
calibration.py
Herramientas de calibración para el robot de rescate
Calibra motores, sensores e IMU para máximo rendimiento
"""

import time
import json
import os
from math import sqrt, degrees
from datetime import datetime
from hardware import MotorController, UltrasonicArray, IMUController
from config import DATA_PATHS, CALIBRATION, ROBOT_DIMENSIONS, create_data_directories

class RobotCalibrator:
    """Calibrador completo para todos los componentes del robot"""
    
    def __init__(self):
        # Inicializar hardware
        self.motors = MotorController()
        self.ultrasonics = UltrasonicArray()
        self.imu = IMUController()
        
        # Resultados de calibración
        self.calibration_results = {}
        
        # Crear directorios de datos
        create_data_directories()
        
        print("Calibrador de robot inicializado")
        self._print_hardware_status()
    
    def _print_hardware_status(self):
        """Mostrar estado del hardware"""
        motor_status = "OK" if self.motors.is_initialized else "FALLO"
        ultrasonic_status = self.ultrasonics.get_array_status()
        imu_status = "OK" if self.imu.is_connected else "FALLO"
        
        print(f"  Motores: {motor_status}")
        print(f"  Ultrasonidos: {ultrasonic_status['active_sensors']}/{ultrasonic_status['total_sensors']} activos")
        print(f"  IMU: {imu_status}")
    
    # ========================================================================
    # CALIBRACIÓN DE MOTORES
    # ========================================================================
    
    def calibrate_motor_speeds(self, test_duration=5, test_speeds=None):
        """
        Calibrar velocidades de motores midiendo movimiento real
        
        Args:
            test_duration: Duración de cada test en segundos
            test_speeds: Lista de velocidades PWM a probar
        """
        if not self.motors.is_initialized:
            print("Error: Motores no inicializados")
            return None
        
        if test_speeds is None:
            test_speeds = [100, 150, 200]
        
        print(f"Calibración de velocidades de motores")
        print("IMPORTANTE: Coloca el robot en línea recta con espacio libre")
        print("Medirás la distancia real recorrida para cada velocidad")
        
        results = []
        
        for speed in test_speeds:
            print(f"\n--- Probando velocidad PWM: {speed} ---")
            input("Posiciona el robot y presiona Enter para continuar...")
            
            print(f"Robot se moverá a velocidad {speed} por {test_duration} segundos")
            input("Presiona Enter para iniciar el movimiento...")
            
            # Ejecutar movimiento
            print("MOVIENDO...")
            self.motors.set_speeds(speed, speed)
            time.sleep(test_duration)
            self.motors.stop()
            print("DETENIDO")
            
            # Solicitar medición manual
            try:
                actual_distance = float(input("Distancia real recorrida (cm): "))
                
                # Calcular velocidad real
                real_speed_cm_s = actual_distance / test_duration
                expected_speed_cm_s = (speed / 255.0) * CALIBRATION['speed_factor'] * 100
                
                correction_factor = real_speed_cm_s / expected_speed_cm_s if expected_speed_cm_s > 0 else 1
                
                result = {
                    'pwm_speed': speed,
                    'test_duration': test_duration,
                    'expected_distance': expected_speed_cm_s * test_duration,
                    'actual_distance': actual_distance,
                    'real_speed_cm_s': real_speed_cm_s,
                    'correction_factor': correction_factor
                }
                
                results.append(result)
                
                print(f"Velocidad esperada: {expected_speed_cm_s:.2f} cm/s")
                print(f"Velocidad real: {real_speed_cm_s:.2f} cm/s")
                print(f"Factor de corrección: {correction_factor:.3f}")
                
            except ValueError:
                print("Valor inválido, saltando este test")
                continue
        
        if results:
            # Calcular factor promedio
            avg_correction = sum(r['correction_factor'] for r in results) / len(results)
            new_speed_factor = CALIBRATION['speed_factor'] * avg_correction
            
            calibration_data = {
                'timestamp': time.time(),
                'test_results': results,
                'current_speed_factor': CALIBRATION['speed_factor'],
                'recommended_speed_factor': new_speed_factor,
                'average_correction': avg_correction
            }
            
            self.calibration_results['motor_speeds'] = calibration_data
            
            print(f"\n=== RESULTADOS DE CALIBRACIÓN ===")
            print(f"Factor actual: {CALIBRATION['speed_factor']}")
            print(f"Factor recomendado: {new_speed_factor:.4f}")
            print(f"Mejora promedio: {avg_correction:.3f}x")
            
            print(f"\nPara aplicar:")
            print(f"Cambiar en config.py: CALIBRATION['speed_factor'] = {new_speed_factor:.4f}")
            
            return calibration_data
        
        else:
            print("No se obtuvieron resultados válidos")
            return None
    
    def calibrate_motor_balance(self, test_duration=10, test_speed=150):
        """Calibrar balance entre motores para movimiento recto"""
        print(f"Calibración de balance de motores")
        print("El robot se moverá recto. Observa si se desvía")
        
        input("Coloca el robot en línea recta y presiona Enter...")
        
        print(f"Moviendo a velocidad {test_speed} por {test_duration} segundos...")
        self.motors.set_speeds(test_speed, test_speed)
        time.sleep(test_duration)
        self.motors.stop()
        
        print("\nResultados:")
        print("¿Hacia dónde se desvió el robot?")
        print("1. Izquierda (motor derecho más potente o problema de conexión)")
        print("2. Derecha (motor izquierdo más potente o problema de conexión)")  
        print("3. Recto (motores balanceados)")
        print("4. No avanzó / movimiento errático (verificar conexiones)")
        
        try:
            choice = input("Selecciona [1-4]: ").strip()
            
            balance_result = {
                'timestamp': time.time(),
                'test_speed': test_speed,
                'test_duration': test_duration,
                'deviation': None,
                'correction_needed': None
            }
            
            if choice == '1':
                balance_result['deviation'] = 'left'
                balance_result['correction_needed'] = 'reduce_right_motor_or_check_wiring'
                print("Recomendación: Reducir velocidad del motor derecho ~5% o verificar conexiones")
                
            elif choice == '2':
                balance_result['deviation'] = 'right'
                balance_result['correction_needed'] = 'reduce_left_motor_or_check_wiring'
                print("Recomendación: Reducir velocidad del motor izquierdo ~5% o verificar conexiones")
                
            elif choice == '3':
                balance_result['deviation'] = 'none'
                balance_result['correction_needed'] = 'none'
                print("Motores bien balanceados")
                
            elif choice == '4':
                balance_result['deviation'] = 'no_movement'
                balance_result['correction_needed'] = 'check_motor_wiring'
                print("Verificar conexiones de motores y direcciones de giro")
                
            else:
                print("Opción inválida")
                return None
            
            self.calibration_results['motor_balance'] = balance_result
            return balance_result
            
        except KeyboardInterrupt:
            print("Calibración interrumpida")
            return None
    
    def calibrate_turn_rates(self):
        """Calibrar velocidades de giro del robot"""
        print("Calibración de velocidades de giro")
        
        test_speed = 150
        test_duration = 5
        
        print(f"El robot girará en el lugar a velocidad {test_speed}")
        print("Cuenta las vueltas completas o fracciones")
        
        input("Presiona Enter para iniciar...")
        
        print("Girando a la derecha...")
        self.motors.set_speeds(test_speed, -test_speed)
        time.sleep(test_duration)
        self.motors.stop()
        
        try:
            turns_right = float(input("¿Cuántas vueltas completas? (ej: 2.5): "))
            
            # Calcular velocidad angular real
            real_angular_speed = (turns_right * 360) / test_duration  # grados/segundo
            
            # Velocidad angular esperada por odometría
            expected_linear_speed = (test_speed / 255.0) * CALIBRATION['speed_factor'] * 100
            expected_angular_speed = (2 * expected_linear_speed) / ROBOT_DIMENSIONS['wheel_base']
            expected_angular_speed = degrees(expected_angular_speed)
            
            correction = real_angular_speed / expected_angular_speed if expected_angular_speed > 0 else 1
            corrected_wheel_base = ROBOT_DIMENSIONS['wheel_base'] / correction
            
            turn_calibration = {
                'timestamp': time.time(),
                'test_speed': test_speed,
                'test_duration': test_duration,
                'turns_measured': turns_right,
                'real_angular_speed': real_angular_speed,
                'expected_angular_speed': expected_angular_speed,
                'wheel_base_current': ROBOT_DIMENSIONS['wheel_base'],
                'wheel_base_corrected': corrected_wheel_base,
                'correction_factor': correction
            }
            
            self.calibration_results['turn_rates'] = turn_calibration
            
            print(f"\nResultados:")
            print(f"Velocidad angular real: {real_angular_speed:.1f}°/s")
            print(f"Velocidad angular esperada: {expected_angular_speed:.1f}°/s")
            print(f"Wheel base actual: {ROBOT_DIMENSIONS['wheel_base']:.1f} cm")
            print(f"Wheel base corregido: {corrected_wheel_base:.1f} cm")
            
            return turn_calibration
            
        except ValueError:
            print("Valor inválido")
            return None
    
    # ========================================================================
    # CALIBRACIÓN DE SENSORES
    # ========================================================================
    
    def calibrate_ultrasonic_sensors(self, test_duration=30):
        """Calibrar sensores ultrasónicos con objetos a distancias conocidas"""
        print(f"Calibración de sensores ultrasónicos")
        print("Coloca objetos a distancias conocidas frente a cada sensor")
        
        distances_to_test = [10, 20, 30, 50, 100]  # cm
        sensor_results = {}
        
        ultrasonic_status = self.ultrasonics.get_array_status()
        active_sensors = [name for name, details in ultrasonic_status['sensor_details'].items() 
                         if details['initialized']]
        
        for sensor_name in active_sensors:
            print(f"\n--- Calibrando sensor: {sensor_name} ---")
            sensor_measurements = []
            
            for expected_distance in distances_to_test:
                print(f"\nColoca objeto a {expected_distance}cm del sensor {sensor_name}")
                input("Presiona Enter cuando esté listo...")
                
                # Tomar múltiples mediciones
                readings = []
                print("Tomando mediciones...")
                
                for i in range(10):  # 10 mediciones
                    reading = self.ultrasonics.read_sensor(sensor_name)
                    if reading > 0:
                        readings.append(reading)
                    time.sleep(0.2)
                
                if readings:
                    avg_reading = sum(readings) / len(readings)
                    std_dev = sqrt(sum((r - avg_reading)**2 for r in readings) / len(readings))
                    
                    measurement = {
                        'expected_distance': expected_distance,
                        'measured_distance': avg_reading,
                        'std_deviation': std_dev,
                        'error': abs(avg_reading - expected_distance),
                        'error_percentage': (abs(avg_reading - expected_distance) / expected_distance) * 100,
                        'readings_count': len(readings)
                    }
                    
                    sensor_measurements.append(measurement)
                    
                    print(f"Esperado: {expected_distance}cm, Medido: {avg_reading:.1f}cm ±{std_dev:.1f}")
                    print(f"Error: {measurement['error']:.1f}cm ({measurement['error_percentage']:.1f}%)")
                
                else:
                    print(f"No se pudieron obtener lecturas válidas")
            
            if sensor_measurements:
                # Calcular estadísticas del sensor
                avg_error = sum(m['error'] for m in sensor_measurements) / len(sensor_measurements)
                max_error = max(m['error'] for m in sensor_measurements)
                avg_std_dev = sum(m['std_deviation'] for m in sensor_measurements) / len(sensor_measurements)
                
                sensor_results[sensor_name] = {
                    'measurements': sensor_measurements,
                    'average_error': avg_error,
                    'max_error': max_error,
                    'average_std_dev': avg_std_dev,
                    'accuracy_rating': 'good' if avg_error < 2 else 'fair' if avg_error < 5 else 'poor'
                }
                
                print(f"\n{sensor_name} - Resumen:")
                print(f"  Error promedio: {avg_error:.1f}cm")
                print(f"  Error máximo: {max_error:.1f}cm")
                print(f"  Precisión: ±{avg_std_dev:.1f}cm")
                print(f"  Calificación: {sensor_results[sensor_name]['accuracy_rating']}")
        
        if sensor_results:
            calibration_data = {
                'timestamp': time.time(),
                'sensor_results': sensor_results,
                'test_distances': distances_to_test
            }
            
            self.calibration_results['ultrasonic_sensors'] = calibration_data
            
            print(f"\n=== RESUMEN GENERAL ===")
            for sensor_name, data in sensor_results.items():
                print(f"{sensor_name}: {data['accuracy_rating']} (±{data['average_error']:.1f}cm)")
            
            return calibration_data
        
        return None
    
    def calibrate_imu(self):
        """Calibrar IMU (giroscopio y acelerómetro)"""
        if not self.imu.is_connected:
            print("Error: IMU no conectado")
            return None
        
        print("Calibración de IMU")
        
        # Calibrar giroscopio
        print("\n1. Calibrando giroscopio...")
        print("IMPORTANTE: El robot debe estar COMPLETAMENTE QUIETO")
        input("Presiona Enter para calibrar giroscopio...")
        
        gyro_success = self.imu.calibrate_gyro_z(samples=200)
        
        # Calibrar acelerómetro  
        print("\n2. Calibrando acelerómetro...")
        print("IMPORTANTE: El robot debe estar en superficie PLANA")
        input("Presiona Enter para calibrar acelerómetro...")
        
        accel_success = self.imu.calibrate_accelerometer(samples=100)
        
        if gyro_success or accel_success:
            # Probar precisión del heading
            print("\n3. Probando precisión de heading...")
            print("Gira el robot exactamente 90° en sentido horario")
            input("Presiona Enter para comenzar...")
            
            self.imu.reset_integrated_heading(0)
            
            input("Ahora gira 90° y presiona Enter...")
            
            final_heading = self.imu.update_integrated_heading()
            heading_error = abs(final_heading - 90)
            
            imu_calibration = {
                'timestamp': time.time(),
                'gyro_calibrated': gyro_success,
                'accel_calibrated': accel_success,
                'gyro_offset': self.imu.gyro_z_offset,
                'accel_offsets': self.imu.accel_offsets.copy(),
                'heading_test': {
                    'expected_heading': 90,
                    'measured_heading': final_heading,
                    'error': heading_error,
                    'accuracy': 'good' if heading_error < 5 else 'fair' if heading_error < 15 else 'poor'
                }
            }
            
            self.calibration_results['imu'] = imu_calibration
            
            print(f"\nResultados IMU:")
            print(f"Giroscopio: {'OK' if gyro_success else 'FALLO'}")
            print(f"Acelerómetro: {'OK' if accel_success else 'FALLO'}")
            print(f"Error de heading: {heading_error:.1f}° ({imu_calibration['heading_test']['accuracy']})")
            
            return imu_calibration
        
        return None
    
    # ========================================================================
    # GESTIÓN DE RESULTADOS
    # ========================================================================
    
    def save_calibration_results(self, filename=None):
        """Guardar resultados de calibración"""
        if not self.calibration_results:
            print("No hay resultados de calibración para guardar")
            return None
        
        if filename is None:
            filename = f"calibration_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        filepath = os.path.join(DATA_PATHS['calibration'], filename)
        
        # Añadir metadata
        calibration_data = {
            'metadata': {
                'timestamp': time.time(),
                'created_at': datetime.now().isoformat(),
                'robot_config': {
                    'wheel_base': ROBOT_DIMENSIONS['wheel_base'],
                    'wheel_diameter': ROBOT_DIMENSIONS['wheel_diameter'],
                    'current_speed_factor': CALIBRATION['speed_factor']
                }
            },
            'calibration_results': self.calibration_results.copy()
        }
        
        try:
            with open(filepath, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            print(f"Resultados guardados: {filepath}")
            return filepath
            
        except Exception as e:
            print(f"Error guardando resultados: {e}")
            return None
    
    def load_calibration_results(self, filename):
        """Cargar resultados de calibración previos"""
        filepath = os.path.join(DATA_PATHS['calibration'], filename)
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            self.calibration_results = data.get('calibration_results', {})
            print(f"Resultados cargados: {filename}")
            return data
            
        except Exception as e:
            print(f"Error cargando resultados: {e}")
            return None
    
    def print_calibration_summary(self):
        """Mostrar resumen de calibraciones realizadas"""
        if not self.calibration_results:
            print("No hay calibraciones realizadas")
            return
        
        print("\n=== RESUMEN DE CALIBRACIONES ===")
        
        for component, data in self.calibration_results.items():
            print(f"\n{component.upper()}:")
            
            if component == 'motor_speeds':
                print(f"  Factor recomendado: {data['recommended_speed_factor']:.4f}")
                print(f"  Mejora promedio: {data['average_correction']:.3f}x")
                
            elif component == 'motor_balance':
                deviation = data['deviation']
                if deviation == 'none':
                    print("  Balance: Perfecto")
                else:
                    print(f"  Desviación: {deviation}")
                    print(f"  Corrección: {data['correction_needed']}")
                    
            elif component == 'turn_rates':
                print(f"  Wheel base corregido: {data['wheel_base_corrected']:.1f}cm")
                print(f"  Error angular: {data['correction_factor']:.3f}x")
                
            elif component == 'ultrasonic_sensors':
                for sensor, sensor_data in data['sensor_results'].items():
                    print(f"  {sensor}: {sensor_data['accuracy_rating']} (±{sensor_data['average_error']:.1f}cm)")
                    
            elif component == 'imu':
                print(f"  Giroscopio: {'OK' if data['gyro_calibrated'] else 'FALLO'}")
                print(f"  Acelerómetro: {'OK' if data['accel_calibrated'] else 'FALLO'}")
                print(f"  Precisión heading: {data['heading_test']['accuracy']}")
    
    def cleanup(self):
        """Limpiar recursos"""
        self.motors.cleanup()
        print("Recursos de calibración liberados")

# ============================================================================
# FUNCIONES DE UTILIDAD
# ============================================================================

def calibration_wizard():
    """Asistente de calibración completo"""
    print("ASISTENTE DE CALIBRACIÓN DEL ROBOT")
    print("=" * 40)
    
    calibrator = RobotCalibrator()
    
    try:
        while True:
            print("\nOpciones de calibración:")
            print("1. Calibrar velocidades de motores")
            print("2. Calibrar balance de motores") 
            print("3. Calibrar velocidades de giro")
            print("4. Calibrar sensores ultrasónicos")
            print("5. Calibrar IMU")
            print("6. Ejecutar calibración completa")
            print("7. Mostrar resumen")
            print("8. Guardar resultados")
            print("9. Cargar calibración anterior")
            print("0. Salir")
            
            choice = input("\nElegir opción [0-9]: ").strip()
            
            if choice == '1':
                calibrator.calibrate_motor_speeds()
            elif choice == '2':
                calibrator.calibrate_motor_balance()
            elif choice == '3':
                calibrator.calibrate_turn_rates()
            elif choice == '4':
                calibrator.calibrate_ultrasonic_sensors()
            elif choice == '5':
                calibrator.calibrate_imu()
            elif choice == '6':
                print("Ejecutando calibración completa...")
                calibrator.calibrate_motor_speeds()
                calibrator.calibrate_motor_balance()
                calibrator.calibrate_turn_rates()
                calibrator.calibrate_ultrasonic_sensors()
                calibrator.calibrate_imu()
                print("Calibración completa finalizada")
            elif choice == '7':
                calibrator.print_calibration_summary()
            elif choice == '8':
                calibrator.save_calibration_results()
            elif choice == '9':
                # Listar archivos disponibles
                cal_dir = DATA_PATHS['calibration']
                if os.path.exists(cal_dir):
                    files = [f for f in os.listdir(cal_dir) if f.endswith('.json')]
                    if files:
                        print("Archivos disponibles:")
                        for i, f in enumerate(files, 1):
                            print(f"  {i}. {f}")
                        try:
                            idx = int(input("Elegir archivo: ")) - 1
                            if 0 <= idx < len(files):
                                calibrator.load_calibration_results(files[idx])
                        except ValueError:
                            print("Opción inválida")
                    else:
                        print("No hay archivos de calibración")
            elif choice == '0':
                break
            else:
                print("Opción inválida")
    
    except KeyboardInterrupt:
        print("\nCalibración interrumpida")
    
    finally:
        calibrator.cleanup()

if __name__ == "__main__":
    calibration_wizard()