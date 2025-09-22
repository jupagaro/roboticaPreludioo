#!/usr/bin/env python3
"""
sensor_fusion.py
Sistema de sensor fusion que combina todos los sensores
Integra: 4 HC-SR04 + MPU6050 + odometría de motores
"""

import time
import threading
from math import cos, sin, radians, degrees, sqrt, atan2
from hardware import MotorController, UltrasonicArray, IMUController
from utils import DataLogger
from config import (
    CALIBRATION, SENSOR_POSITIONS, FIELD_DIMENSIONS, 
    DETECTION_THRESHOLDS, ROBOT_DIMENSIONS
)

class SensorFusion:
    """Sistema de fusión de sensores para navegación autónoma"""
    
    def __init__(self, session_name=None, enable_logging=True):
        # Estado del robot
        self.position = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        self.velocity = {'linear': 0.0, 'angular': 0.0}
        
        # Timestamps para integración
        self.last_update_time = time.time()
        self.last_motor_update = time.time()
        
        # Control de threading
        self.running = False
        self.data_lock = threading.Lock()
        
        # Inicializar hardware
        self.motors = MotorController()
        self.ultrasonics = UltrasonicArray()
        self.imu = IMUController()
        
        # Sistema de logging
        self.enable_logging = enable_logging
        if enable_logging:
            self.logger = DataLogger(session_name)
        
        # Datos de sensores
        self.sensor_data = {
            'ultrasonic_readings': {},
            'imu_data': {},
            'obstacles_global': [],
            'last_sensor_update': 0
        }
        
        print("🧠 Sistema de Sensor Fusion inicializado")
        self._print_system_status()
    
    def _print_system_status(self):
        """Mostrar estado de inicialización del sistema"""
        motor_status = "✅" if self.motors.is_initialized else "❌"
        ultrasonic_status = self.ultrasonics.get_array_status()
        imu_status = "✅" if self.imu.is_connected else "❌"
        
        print(f"  Motores L298N: {motor_status}")
        print(f"  Sensores HC-SR04: ✅ {ultrasonic_status['active_sensors']}/{ultrasonic_status['total_sensors']}")
        print(f"  IMU MPU6050: {imu_status}")
        if self.enable_logging:
            print(f"  Logging: ✅ Sesión: {self.logger.session_name}")
    
    # ========================================================================
    # FUSIÓN DE DATOS DE SENSORES
    # ========================================================================
    
    def update_sensor_data(self):
        """Actualizar datos de todos los sensores"""
        current_time = time.time()
        
        # Leer sensores ultrasónicos
        ultrasonic_readings = self.ultrasonics.read_all_sensors()
        
        # Leer IMU
        imu_data = {
            'gyro': self.imu.read_gyro_scaled(),
            'accel': self.imu.read_accel_scaled(),
            'heading_rate': self.imu.get_heading_rate(),
            'temperature': self.imu.read_temperature_celsius()
        }
        
        # Calcular obstáculos en coordenadas globales
        obstacles_global = self._calculate_global_obstacles(ultrasonic_readings)
        
        # Actualizar datos thread-safe
        with self.data_lock:
            self.sensor_data.update({
                'ultrasonic_readings': ultrasonic_readings,
                'imu_data': imu_data,
                'obstacles_global': obstacles_global,
                'last_sensor_update': current_time
            })
    
    def _calculate_global_obstacles(self, ultrasonic_readings):
        """Convertir lecturas de sensores a coordenadas globales"""
        obstacles = []
        
        for sensor_name, distance in ultrasonic_readings.items():
            if distance > 0 and sensor_name in SENSOR_POSITIONS:
                sensor_config = SENSOR_POSITIONS[sensor_name]
                
                # Posición del sensor en el robot (coordenadas locales)
                sensor_local_x = sensor_config['x']
                sensor_local_y = sensor_config['y']
                sensor_angle = sensor_config['angle']
                
                # Transformar posición del sensor a coordenadas globales
                cos_heading = cos(radians(self.position['heading']))
                sin_heading = sin(radians(self.position['heading']))
                
                sensor_global_x = (self.position['x'] + 
                                 sensor_local_x * cos_heading - 
                                 sensor_local_y * sin_heading)
                sensor_global_y = (self.position['y'] + 
                                 sensor_local_x * sin_heading + 
                                 sensor_local_y * cos_heading)
                
                # Ángulo del sensor en coordenadas globales
                sensor_global_angle = self.position['heading'] + sensor_angle
                
                # Posición del obstáculo
                obstacle_x = sensor_global_x + distance * cos(radians(sensor_global_angle))
                obstacle_y = sensor_global_y + distance * sin(radians(sensor_global_angle))
                
                obstacles.append({
                    'x': obstacle_x,
                    'y': obstacle_y,
                    'distance': distance,
                    'sensor': sensor_name,
                    'confidence': self._calculate_obstacle_confidence(distance)
                })
        
        return obstacles
    
    def _calculate_obstacle_confidence(self, distance):
        """Calcular confianza en la detección del obstáculo"""
        if distance < 10:
            return 'high'
        elif distance < 50:
            return 'medium'
        else:
            return 'low'
    
    # ========================================================================
    # ESTIMACIÓN DE POSICIÓN (ODOMETRÍA + IMU)
    # ========================================================================
    
    def update_position_estimate(self):
        """Actualizar estimación de posición usando odometría y IMU"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        if dt <= 0:
            return
        
        # Obtener velocidades actuales de los motores
        left_speed, right_speed = self.motors.get_speeds()
        
        # Convertir PWM a velocidad lineal (cm/s)
        left_velocity = (left_speed / 255.0) * CALIBRATION['speed_factor'] * 100
        right_velocity = (right_speed / 255.0) * CALIBRATION['speed_factor'] * 100
        
        # Cinemática diferencial
        linear_velocity = (left_velocity + right_velocity) / 2
        angular_velocity_odo = (right_velocity - left_velocity) / ROBOT_DIMENSIONS['wheel_base']
        
        # Fusión complementaria con IMU para heading
        with self.data_lock:
            imu_angular_velocity = radians(self.sensor_data['imu_data'].get('heading_rate', 0))
        
        # Filtro complementario
        alpha = CALIBRATION['gyro_alpha']
        fused_angular_velocity = (alpha * imu_angular_velocity + 
                                (1 - alpha) * angular_velocity_odo)
        
        # Integrar posición
        heading_change = degrees(fused_angular_velocity * dt)
        
        # Actualizar heading
        self.position['heading'] += heading_change
        
        # Normalizar heading [-180, 180]
        while self.position['heading'] > 180:
            self.position['heading'] -= 360
        while self.position['heading'] < -180:
            self.position['heading'] += 360
        
        # Actualizar posición XY
        self.position['x'] += linear_velocity * cos(radians(self.position['heading'])) * dt
        self.position['y'] += linear_velocity * sin(radians(self.position['heading'])) * dt
        
        # Actualizar velocidades
        self.velocity['linear'] = linear_velocity
        self.velocity['angular'] = degrees(fused_angular_velocity)
        
        self.last_update_time = current_time
    
    # ========================================================================
    # NAVEGACIÓN INTELIGENTE
    # ========================================================================
    
    def get_navigation_data(self):
        """Obtener datos procesados para navegación"""
        with self.data_lock:
            readings = self.sensor_data['ultrasonic_readings'].copy()
            obstacles = self.sensor_data['obstacles_global'].copy()
        
        # Clasificar obstáculos por dirección
        front_distances = []
        back_distances = []
        side_distances = {'left': [], 'right': []}
        
        for sensor_name, distance in readings.items():
            if distance > 0:
                if 'frontal' in sensor_name:
                    front_distances.append(distance)
                elif 'trasero' in sensor_name:
                    back_distances.append(distance)
                    # También clasificar por lado
                    if 'izq' in sensor_name:
                        side_distances['left'].append(distance)
                    elif 'der' in sensor_name:
                        side_distances['right'].append(distance)
        
        return {
            'front_min': min(front_distances) if front_distances else float('inf'),
            'back_min': min(back_distances) if back_distances else float('inf'),
            'left_min': min(side_distances['left']) if side_distances['left'] else float('inf'),
            'right_min': min(side_distances['right']) if side_distances['right'] else float('inf'),
            'obstacles_nearby': len([o for o in obstacles if o['distance'] < DETECTION_THRESHOLDS['obstacle_min']]),
            'potential_blocks': self._detect_potential_blocks(readings)
        }
    
    def _detect_potential_blocks(self, readings):
        """Detectar objetos pequeños que podrían ser bloques"""
        potential_blocks = []
        
        for sensor_name, distance in readings.items():
            if (DETECTION_THRESHOLDS['block_detect_min'] <= distance <= 
                DETECTION_THRESHOLDS['block_detect_max']):
                
                potential_blocks.append({
                    'sensor': sensor_name,
                    'distance': distance,
                    'confidence': 'high' if 8 <= distance <= 25 else 'medium'
                })
        
        return potential_blocks
    
    def navigate_to_point(self, target_x, target_y, speed=120):
        """
        Navegar hacia un punto específico evitando obstáculos
        
        Args:
            target_x, target_y: Coordenadas objetivo
            speed: Velocidad base
            
        Returns:
            dict: Estado de navegación
        """
        # Vector hacia objetivo
        dx = target_x - self.position['x']
        dy = target_y - self.position['y']
        distance_to_target = sqrt(dx**2 + dy**2)
        
        if distance_to_target < 5:  # Llegamos al objetivo
            self.motors.stop()
            return {'status': 'arrived', 'distance': distance_to_target}
        
        # Ángulo hacia objetivo
        target_angle = degrees(atan2(dy, dx))
        angle_diff = target_angle - self.position['heading']
        
        # Normalizar diferencia angular
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        # Obtener datos de navegación
        nav_data = self.get_navigation_data()
        
        # Lógica de navegación
        if nav_data['front_min'] < DETECTION_THRESHOLDS['obstacle_min']:
            # Obstáculo frontal - maniobra evasiva
            if nav_data['left_min'] > nav_data['right_min']:
                self.motors.spin_left(speed // 2)
                return {'status': 'avoiding_left', 'distance': distance_to_target}
            else:
                self.motors.spin_right(speed // 2)
                return {'status': 'avoiding_right', 'distance': distance_to_target}
        
        elif abs(angle_diff) > 15:
            # Girar hacia objetivo
            turn_speed = min(speed, abs(angle_diff) * 2)
            if angle_diff > 0:
                self.motors.spin_left(turn_speed)
                return {'status': 'turning_left', 'distance': distance_to_target}
            else:
                self.motors.spin_right(turn_speed)
                return {'status': 'turning_right', 'distance': distance_to_target}
        
        else:
            # Avanzar hacia objetivo
            actual_speed = min(speed, distance_to_target * 10)  # Reducir velocidad cerca del objetivo
            self.motors.move_forward(actual_speed)
            return {'status': 'moving_forward', 'distance': distance_to_target}
    
    def reactive_navigation(self, base_speed=120):
        """Navegación reactiva básica evitando obstáculos"""
        nav_data = self.get_navigation_data()
        
        front_min = nav_data['front_min']
        back_min = nav_data['back_min']
        left_min = nav_data['left_min']
        right_min = nav_data['right_min']
        
        threshold = DETECTION_THRESHOLDS['obstacle_min']
        
        if front_min > threshold * 3:
            # Camino libre - avanzar
            self.motors.move_forward(base_speed)
            return 'forward'
        
        elif front_min > threshold:
            # Obstáculo cerca - reducir velocidad
            self.motors.move_forward(base_speed // 2)
            return 'slow_forward'
        
        else:
            # Obstáculo bloqueando - decidir escape
            if left_min > right_min and left_min > threshold:
                self.motors.spin_left(base_speed // 2)
                return 'escape_left'
            elif right_min > threshold:
                self.motors.spin_right(base_speed // 2)
                return 'escape_right'
            elif back_min > threshold * 2:
                self.motors.move_backward(base_speed // 3)
                return 'backing_up'
            else:
                # Totalmente bloqueado - girar en el lugar
                self.motors.spin_right(base_speed // 3)
                return 'spinning'
    
    # ========================================================================
    # LOGGING Y ESTADO
    # ========================================================================
    
    def log_current_state(self):
        """Registrar estado actual del sistema"""
        if not self.enable_logging:
            return
        
        # Compilar datos completos del robot
        robot_data = {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'sensors': self.sensor_data['ultrasonic_readings'].copy(),
            'imu': self.sensor_data['imu_data'].copy(),
            'obstacles': self.sensor_data['obstacles_global'].copy(),
            'motors': {
                'left_speed': self.motors.left_speed,
                'right_speed': self.motors.right_speed,
                'status': self.motors.get_status()
            },
            'navigation': self.get_navigation_data()
        }
        
        self.logger.log_robot_state(robot_data)
    
    def get_system_status(self):
        """Obtener estado completo del sistema"""
        motor_status = self.motors.get_status()
        ultrasonic_status = self.ultrasonics.get_array_status()
        imu_status = self.imu.get_sensor_status()
        
        with self.data_lock:
            sensor_age = time.time() - self.sensor_data['last_sensor_update']
        
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'sensors': {
                'motors': motor_status,
                'ultrasonics': ultrasonic_status,
                'imu': imu_status,
                'data_age': sensor_age
            },
            'navigation': self.get_navigation_data(),
            'system_health': {
                'motors_ok': motor_status['initialized'],
                'ultrasonics_ok': ultrasonic_status['active_sensors'] > 0,
                'imu_ok': imu_status['connected'],
                'data_fresh': sensor_age < 1.0
            }
        }
    
    def print_status(self):
        """Mostrar estado del sistema en consola"""
        status = self.get_system_status()
        nav_data = status['navigation']
        
        print(f"\n--- SENSOR FUSION STATUS ---")
        print(f"Position: ({self.position['x']:6.1f}, {self.position['y']:6.1f}) cm")
        print(f"Heading: {self.position['heading']:6.1f}° | Velocity: {self.velocity['linear']:5.1f} cm/s")
        
        # Estado de sensores
        health = status['system_health']
        health_icons = {True: "✅", False: "❌"}
        print(f"Health: Motors{health_icons[health['motors_ok']]} "
              f"Ultrasonics{health_icons[health['ultrasonics_ok']]} "
              f"IMU{health_icons[health['imu_ok']]} "
              f"Data{health_icons[health['data_fresh']]}")
        
        # Distancias
        print(f"Distances: Front={nav_data['front_min']:5.1f} "
              f"Back={nav_data['back_min']:5.1f} "
              f"Left={nav_data['left_min']:5.1f} "
              f"Right={nav_data['right_min']:5.1f} cm")
        
        if nav_data['potential_blocks']:
            print(f"Potential blocks: {len(nav_data['potential_blocks'])}")
    
    # ========================================================================
    # CONTROL DEL SISTEMA
    # ========================================================================
    
    def start_sensor_loop(self, update_rate=10):
        """Iniciar loop principal de sensores"""
        self.running = True
        self.sensor_thread = threading.Thread(
            target=self._sensor_loop, 
            args=(update_rate,), 
            daemon=True
        )
        self.sensor_thread.start()
        print(f"🚀 Sensor fusion iniciado ({update_rate}Hz)")
    
    def _sensor_loop(self, update_rate):
        """Loop principal de actualización de sensores"""
        sleep_time = 1.0 / update_rate
        
        while self.running:
            try:
                # Actualizar sensores y posición
                self.update_sensor_data()
                self.update_position_estimate()
                
                # Logging
                if self.enable_logging:
                    self.log_current_state()
                
                time.sleep(sleep_time)
                
            except Exception as e:
                print(f"Error en sensor loop: {e}")
                time.sleep(0.1)
    
    def stop(self):
        """Detener sistema completo"""
        self.running = False
        self.motors.stop()
        
        if self.enable_logging:
            self.logger.save_session()
            print(f"Sesión guardada: {self.logger.session_name}")
        
        print("🛑 Sistema de sensor fusion detenido")
    
    def cleanup(self):
        """Limpiar recursos"""
        self.stop()
        self.motors.cleanup()
        print("🧹 Recursos liberados")

# ============================================================================
# FUNCIONES DE PRUEBA
# ============================================================================

def test_sensor_fusion():
    """Función de prueba del sistema completo"""
    print("🎯 Prueba del sistema de sensor fusion")
    
    try:
        # Crear sistema
        fusion = SensorFusion("test_fusion")
        
        # Iniciar loop de sensores
        fusion.start_sensor_loop(update_rate=5)
        
        # Menú de pruebas
        while True:
            print("\nOpciones:")
            print("1. Mostrar estado")
            print("2. Navegación reactiva (10s)")
            print("3. Ir a punto específico")
            print("4. Solo monitorear sensores")
            print("0. Salir")
            
            choice = input("Elegir: ").strip()
            
            if choice == '1':
                fusion.print_status()
            
            elif choice == '2':
                print("Navegación reactiva por 10 segundos...")
                start_time = time.time()
                while (time.time() - start_time) < 10:
                    action = fusion.reactive_navigation()
                    print(f"\rAcción: {action}", end="", flush=True)
                    time.sleep(0.5)
                fusion.motors.stop()
                print("\nNavegación detenida")
            
            elif choice == '3':
                try:
                    x = float(input("X objetivo: "))
                    y = float(input("Y objetivo: "))
                    
                    print(f"Navegando a ({x}, {y})...")
                    while True:
                        result = fusion.navigate_to_point(x, y)
                        print(f"\r{result['status']} - Distancia: {result['distance']:.1f}cm", 
                              end="", flush=True)
                        
                        if result['status'] == 'arrived':
                            print("\n¡Objetivo alcanzado!")
                            break
                        
                        time.sleep(0.5)
                        
                except ValueError:
                    print("Coordenadas inválidas")
                except KeyboardInterrupt:
                    fusion.motors.stop()
                    print("\nNavegación interrumpida")
            
            elif choice == '4':
                print("Monitoreando sensores - Ctrl+C para parar")
                try:
                    while True:
                        fusion.print_status()
                        time.sleep(2)
                except KeyboardInterrupt:
                    print("\nMonitoreo terminado")
            
            elif choice == '0':
                break
    
    except KeyboardInterrupt:
        print("\nPrueba interrumpida")
    
    finally:
        if 'fusion' in locals():
            fusion.cleanup()

if __name__ == "__main__":
    test_sensor_fusion()