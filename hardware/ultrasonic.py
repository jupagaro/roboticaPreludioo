#!/usr/bin/env python3
"""
hardware/ultrasonic.py
Controlador para sensores ultrasónicos HC-SR04
Maneja 4 sensores: 2 frontales + 2 traseros diagonales
"""

import RPi.GPIO as GPIO
import time
import threading
from math import cos, sin, radians
from config import ULTRASONIC_PINS, SENSOR_POSITIONS, DETECTION_THRESHOLDS

class UltrasonicSensor:
    """Controlador para un sensor HC-SR04 individual"""
    
    def __init__(self, name, trig_pin, echo_pin):
        self.name = name
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.last_reading = -1
        self.last_update = 0
        self.error_count = 0
        self.is_initialized = False
        
        self._setup_pins()
    
    def _setup_pins(self):
        """Configurar pines GPIO para el sensor"""
        try:
            GPIO.setup(self.trig_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            GPIO.output(self.trig_pin, GPIO.LOW)
            time.sleep(0.01)  # Estabilizar
            
            self.is_initialized = True
            print(f"✅ Sensor {self.name} inicializado (Trig:{self.trig_pin}, Echo:{self.echo_pin})")
            
        except Exception as e:
            print(f"❌ Error inicializando sensor {self.name}: {e}")
            self.is_initialized = False
    
    def read_distance(self):
        """
        Leer distancia del sensor HC-SR04
        
        Returns:
            float: Distancia en cm, -1 si error
        """
        if not self.is_initialized:
            return -1
        
        try:
            # Enviar pulso de trigger (10µs)
            GPIO.output(self.trig_pin, GPIO.HIGH)
            time.sleep(0.00001)  # 10µs
            GPIO.output(self.trig_pin, GPIO.LOW)
            
            # Medir tiempo de respuesta con timeout
            timeout_start = time.time()
            timeout_limit = DETECTION_THRESHOLDS['sensor_timeout']
            
            # Esperar inicio del pulso echo
            while GPIO.input(self.echo_pin) == GPIO.LOW:
                pulse_start = time.time()
                if pulse_start - timeout_start > timeout_limit:
                    self.error_count += 1
                    return -1
            
            # Esperar fin del pulso echo
            while GPIO.input(self.echo_pin) == GPIO.HIGH:
                pulse_end = time.time()
                if pulse_end - timeout_start > timeout_limit:
                    self.error_count += 1
                    return -1
            
            # Calcular distancia
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # (343m/s * 100cm/m) / 2
            
            # Validar rango
            if (DETECTION_THRESHOLDS['sensor_valid_min'] <= distance <= 
                DETECTION_THRESHOLDS['sensor_valid_max']):
                self.last_reading = distance
                self.last_update = time.time()
                self.error_count = max(0, self.error_count - 1)  # Reducir errores en lecturas exitosas
                return round(distance, 1)
            else:
                self.error_count += 1
                return -1
                
        except Exception as e:
            self.error_count += 1
            return -1
    
    def get_status(self):
        """Obtener estado del sensor"""
        return {
            'name': self.name,
            'initialized': self.is_initialized,
            'last_reading': self.last_reading,
            'last_update': self.last_update,
            'error_count': self.error_count,
            'pins': {'trig': self.trig_pin, 'echo': self.echo_pin}
        }

class UltrasonicArray:
    """Controlador para array de 4 sensores ultrasónicos"""
    
    def __init__(self):
        self.sensors = {}
        self.readings = {}
        self.lock = threading.Lock()
        self.last_scan_time = 0
        
        # Crear sensores individuales
        self._create_sensors()
        
    def _create_sensors(self):
        """Crear instancias de sensores según configuración"""
        print("🔧 Inicializando array de sensores ultrasónicos...")
        
        for sensor_name, pins in ULTRASONIC_PINS.items():
            sensor = UltrasonicSensor(sensor_name, pins['trig'], pins['echo'])
            self.sensors[sensor_name] = sensor
            self.readings[sensor_name] = -1
        
        initialized_count = sum(1 for s in self.sensors.values() if s.is_initialized)
        print(f"✅ Array inicializado: {initialized_count}/{len(self.sensors)} sensores activos")
    
    def read_all_sensors(self):
        """
        Leer todos los sensores secuencialmente
        
        Returns:
            dict: Diccionario con lecturas {sensor_name: distance}
        """
        readings = {}
        
        # Leer sensores secuencialmente para evitar interferencia
        for sensor_name, sensor in self.sensors.items():
            if sensor.is_initialized:
                distance = sensor.read_distance()
                readings[sensor_name] = distance
                
                # Pausa entre sensores para evitar ecos cruzados
                time.sleep(0.02)  # 20ms
        
        # Actualizar lecturas thread-safe
        with self.lock:
            for sensor_name, distance in readings.items():
                if distance > 0:  # Solo actualizar lecturas válidas
                    self.readings[sensor_name] = distance
            
            self.last_scan_time = time.time()
        
        return readings.copy()
    
    def read_sensor(self, sensor_name):
        """
        Leer un sensor específico
        
        Args:
            sensor_name: Nombre del sensor ('frontal_izq', etc.)
            
        Returns:
            float: Distancia en cm, -1 si error
        """
        if sensor_name in self.sensors:
            return self.sensors[sensor_name].read_distance()
        return -1
    
    def get_readings(self):
        """Obtener últimas lecturas válidas"""
        with self.lock:
            return self.readings.copy()
    
    def get_obstacles_relative(self):
        """
        Obtener obstáculos en coordenadas relativas al robot
        
        Returns:
            list: Lista de obstáculos con posición relativa
        """
        obstacles = []
        
        with self.lock:
            current_readings = self.readings.copy()
        
        for sensor_name, distance in current_readings.items():
            if distance > 0 and sensor_name in SENSOR_POSITIONS:
                # Posición del sensor en el robot
                sensor_pos = SENSOR_POSITIONS[sensor_name]
                sensor_angle = sensor_pos['angle']
                
                # Calcular posición del obstáculo relativa al robot
                obstacle_x = sensor_pos['x'] + distance * cos(radians(sensor_angle))
                obstacle_y = sensor_pos['y'] + distance * sin(radians(sensor_angle))
                
                obstacles.append({
                    'sensor': sensor_name,
                    'distance': distance,
                    'relative_x': obstacle_x,
                    'relative_y': obstacle_y,
                    'angle': sensor_angle
                })
        
        return obstacles
    
    def detect_obstacles_by_direction(self):
        """
        Detectar obstáculos agrupados por dirección
        
        Returns:
            dict: Obstáculos por dirección {front, back, sides}
        """
        readings = self.get_readings()
        
        result = {
            'front': {'min_distance': float('inf'), 'sensors': []},
            'back': {'min_distance': float('inf'), 'sensors': []},
            'sides': {'left': float('inf'), 'right': float('inf')}
        }
        
        # Clasificar por dirección
        for sensor_name, distance in readings.items():
            if distance > 0:
                if 'frontal' in sensor_name:
                    result['front']['sensors'].append({
                        'name': sensor_name,
                        'distance': distance
                    })
                    result['front']['min_distance'] = min(result['front']['min_distance'], distance)
                
                elif 'trasero' in sensor_name:
                    result['back']['sensors'].append({
                        'name': sensor_name, 
                        'distance': distance
                    })
                    result['back']['min_distance'] = min(result['back']['min_distance'], distance)
                    
                    # También clasificar por lado para navegación
                    if 'izq' in sensor_name:
                        result['sides']['left'] = min(result['sides']['left'], distance)
                    elif 'der' in sensor_name:
                        result['sides']['right'] = min(result['sides']['right'], distance)
        
        # Limpiar infinitos
        for direction in ['front', 'back']:
            if result[direction]['min_distance'] == float('inf'):
                result[direction]['min_distance'] = -1
        
        for side in ['left', 'right']:
            if result['sides'][side] == float('inf'):
                result['sides'][side] = -1
        
        return result
    
    def detect_potential_blocks(self):
        """
        Detectar objetos pequeños que podrían ser bloques transportables
        
        Returns:
            list: Lista de posibles bloques detectados
        """
        potential_blocks = []
        readings = self.get_readings()
        
        for sensor_name, distance in readings.items():
            # Bloques son pequeños (30x30x50mm) - aparecen como obstáculos cercanos
            if (DETECTION_THRESHOLDS['block_detect_min'] <= distance <= 
                DETECTION_THRESHOLDS['block_detect_max']):
                
                sensor_pos = SENSOR_POSITIONS[sensor_name]
                
                potential_blocks.append({
                    'sensor': sensor_name,
                    'distance': distance,
                    'confidence': 'high' if 8 <= distance <= 25 else 'medium',
                    'relative_position': {
                        'x': sensor_pos['x'] + distance * cos(radians(sensor_pos['angle'])),
                        'y': sensor_pos['y'] + distance * sin(radians(sensor_pos['angle']))
                    }
                })
        
        return potential_blocks
    
    def get_array_status(self):
        """Obtener estado completo del array"""
        sensor_statuses = {}
        for name, sensor in self.sensors.items():
            sensor_statuses[name] = sensor.get_status()
        
        total_errors = sum(s.error_count for s in self.sensors.values())
        active_sensors = sum(1 for s in self.sensors.values() if s.is_initialized)
        
        return {
            'total_sensors': len(self.sensors),
            'active_sensors': active_sensors,
            'total_errors': total_errors,
            'last_scan': self.last_scan_time,
            'current_readings': self.get_readings(),
            'sensor_details': sensor_statuses
        }
    
    def test_sensors(self, duration=10):
        """
        Rutina de prueba para todos los sensores
        
        Args:
            duration: Duración de la prueba en segundos
        """
        print(f"🔧 Probando sensores por {duration} segundos...")
        print("Coloca obstáculos a diferentes distancias para probar")
        
        start_time = time.time()
        
        try:
            while (time.time() - start_time) < duration:
                readings = self.read_all_sensors()
                
                # Mostrar lecturas formateadas
                print(f"\r", end="")
                display_parts = []
                sensor_icons = {
                    'frontal_izq': '↖️', 'frontal_der': '↗️',
                    'trasero_izq': '↙️', 'trasero_der': '↘️'
                }
                
                for sensor_name, distance in readings.items():
                    icon = sensor_icons.get(sensor_name, '📡')
                    if distance > 0:
                        status = "🟢" if distance > DETECTION_THRESHOLDS['obstacle_min'] else "🔴"
                        display_parts.append(f"{icon}{status}{distance:5.1f}cm")
                    else:
                        display_parts.append(f"{icon}❌ Error")
                
                print(" | ".join(display_parts), end="", flush=True)
                time.sleep(0.5)
        
        except KeyboardInterrupt:
            print(f"\n🛑 Prueba interrumpida")
        
        print(f"\n✅ Prueba completada")
        
        # Mostrar estadísticas finales
        status = self.get_array_status()
        print(f"\nEstadísticas:")
        print(f"  Sensores activos: {status['active_sensors']}/{status['total_sensors']}")
        print(f"  Errores totales: {status['total_errors']}")
        
        for sensor_name, details in status['sensor_details'].items():
            print(f"  {sensor_name}: {details['error_count']} errores")

# ============================================================================
# FUNCIONES DE PRUEBA
# ============================================================================

def test_ultrasonic_array():
    """Función de prueba independiente"""
    print("🎯 Prueba independiente del array de sensores ultrasónicos")
    
    try:
        # Configurar GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Crear array
        sensor_array = UltrasonicArray()
        
        # Verificar inicialización
        status = sensor_array.get_array_status()
        if status['active_sensors'] == 0:
            print("❌ No se inicializó ningún sensor")
            return
        
        print(f"\n📊 Estado inicial: {status['active_sensors']}/{status['total_sensors']} sensores activos")
        
        # Menú de pruebas
        while True:
            print("\nOpciones de prueba:")
            print("1. Lectura única de todos los sensores")
            print("2. Monitoreo continuo")
            print("3. Detectar obstáculos por dirección")
            print("4. Buscar bloques potenciales")
            print("5. Probar sensor individual")
            print("6. Estado detallado")
            print("0. Salir")
            
            choice = input("Elegir opción: ").strip()
            
            if choice == '1':
                readings = sensor_array.read_all_sensors()
                print("\n📊 Lecturas actuales:")
                for sensor, distance in readings.items():
                    status = "✅" if distance > 0 else "❌"
                    print(f"  {sensor:12}: {distance:6.1f} cm {status}")
            
            elif choice == '2':
                duration = int(input("Duración (segundos): ") or "10")
                sensor_array.test_sensors(duration)
            
            elif choice == '3':
                obstacles = sensor_array.detect_obstacles_by_direction()
                print("\n🧭 Obstáculos por dirección:")
                print(f"  Frente: {obstacles['front']['min_distance']:.1f} cm")
                print(f"  Atrás: {obstacles['back']['min_distance']:.1f} cm")
                print(f"  Izquierda: {obstacles['sides']['left']:.1f} cm")
                print(f"  Derecha: {obstacles['sides']['right']:.1f} cm")
            
            elif choice == '4':
                blocks = sensor_array.detect_potential_blocks()
                print(f"\n📦 Bloques potenciales detectados: {len(blocks)}")
                for i, block in enumerate(blocks):
                    print(f"  {i+1}: {block['sensor']} - {block['distance']:.1f}cm ({block['confidence']})")
            
            elif choice == '5':
                sensor_names = list(sensor_array.sensors.keys())
                print("Sensores disponibles:")
                for i, name in enumerate(sensor_names, 1):
                    print(f"  {i}. {name}")
                
                try:
                    idx = int(input("Elegir sensor: ")) - 1
                    if 0 <= idx < len(sensor_names):
                        sensor_name = sensor_names[idx]
                        print(f"Probando {sensor_name} - Ctrl+C para parar")
                        
                        while True:
                            distance = sensor_array.read_sensor(sensor_name)
                            status = "✅" if distance > 0 else "❌"
                            print(f"\r{sensor_name}: {distance:6.1f} cm {status}", end="", flush=True)
                            time.sleep(0.2)
                            
                except (ValueError, KeyboardInterrupt):
                    print("\nPrueba individual terminada")
            
            elif choice == '6':
                status = sensor_array.get_array_status()
                print(f"\n📊 Estado detallado:")
                print(f"Sensores totales: {status['total_sensors']}")
                print(f"Sensores activos: {status['active_sensors']}")
                print(f"Errores totales: {status['total_errors']}")
                print(f"Última lectura: {time.ctime(status['last_scan']) if status['last_scan'] > 0 else 'Nunca'}")
                
                print("\nDetalle por sensor:")
                for name, details in status['sensor_details'].items():
                    init_status = "✅" if details['initialized'] else "❌"
                    print(f"  {name:12} {init_status} | Errores: {details['error_count']:3d} | Último: {details['last_reading']:6.1f}cm")
            
            elif choice == '0':
                break
    
    except KeyboardInterrupt:
        print("\n🛑 Prueba interrumpida")
    
    finally:
        try:
            GPIO.cleanup()
        except:
            pass

if __name__ == "__main__":
    test_ultrasonic_array()