#!/usr/bin/env python3
"""
config.py - Configuraciones y constantes del robot de rescate
Todas las configuraciones centralizadas en un solo lugar
"""

import os
from math import radians

# ============================================================================
# CONFIGURACIÓN DE HARDWARE - PINES GPIO
# ============================================================================

# L298N Motor Driver
MOTOR_PINS = {
    'ENA': 13,      # GPIO 13, Pin 33 - PWM Motor Izquierdo
    'IN1': 19,      # GPIO 19, Pin 35 - Dirección Motor Izquierdo
    'IN2': 16,      # GPIO 16, Pin 36 - Dirección Motor Izquierdo
    'IN3': 26,      # GPIO 26, Pin 37 - Dirección Motor Derecho
    'IN4': 20,      # GPIO 20, Pin 38 - Dirección Motor Derecho
    'ENB': 21       # GPIO 21, Pin 40 - PWM Motor Derecho
}

# Sensores HC-SR04 - NUEVA CONFIGURACIÓN
ULTRASONIC_PINS = {
    'frontal_izq': {'trig': 4, 'echo': 17},   # GPIO 4,17 - Frontal Izquierdo
    'frontal_der': {'trig': 27, 'echo': 22},  # GPIO 27,22 - Frontal Derecho
    'trasero_izq': {'trig': 5, 'echo': 6},    # GPIO 5,6 - Trasero Izquierdo (135°)
    'trasero_der': {'trig': 23, 'echo': 24}   # GPIO 23,24 - Trasero Derecho (225°)
}

# MPU6050 IMU
IMU_CONFIG = {
    'address': 0x68,
    'bus': 1
}

# ============================================================================
# CONFIGURACIÓN FÍSICA DEL ROBOT
# ============================================================================

# Dimensiones del robot (en cm)
ROBOT_DIMENSIONS = {
    'length': 30.0,         # Largo total (frente tiene 30cm)
    'width': 15.0,          # Ancho total
    'height': 10.0,         # Altura aproximada
    'wheel_diameter': 4.0,   # Diámetro de ruedas
    'wheel_base': 15.0       # Distancia entre ruedas
}

# Posiciones de sensores respecto al centro del robot (en cm)
SENSOR_POSITIONS = {
    'frontal_izq': {'x': 15.0, 'y': 7.5, 'angle': 0},     # Esquina frontal izquierda
    'frontal_der': {'x': 15.0, 'y': -7.5, 'angle': 0},    # Esquina frontal derecha
    'trasero_izq': {'x': -15.0, 'y': 7.5, 'angle': 135},  # Esquina trasera izquierda (135°)
    'trasero_der': {'x': -15.0, 'y': -7.5, 'angle': 225}  # Esquina trasera derecha (225°)
}

# ============================================================================
# CONFIGURACIÓN DE SENSOR FUSION
# ============================================================================

# Parámetros de calibración
CALIBRATION = {
    'speed_factor': 0.3,        # Factor PWM -> velocidad lineal (ajustar experimentalmente)
    'gyro_alpha': 0.98,         # Peso del giroscopio vs odometría (filtro complementario)
    'sensor_timeout': 0.1       # Timeout para sensores HC-SR04 (segundos)
}

# Umbrales de detección
DETECTION_THRESHOLDS = {
    'obstacle_min': 20,         # Distancia mínima a obstáculos (cm)
    'block_detect_min': 5,      # Distancia mínima para detectar bloques (cm)
    'block_detect_max': 35,     # Distancia máxima para detectar bloques (cm)
    'sensor_valid_min': 2,      # Lectura mínima válida de ultrasonido (cm)
    'sensor_valid_max': 400     # Lectura máxima válida de ultrasonido (cm)
}

# ============================================================================
# CONFIGURACIÓN DE CONTROL MANUAL
# ============================================================================

# Velocidades para control manual WASD
MANUAL_SPEEDS = {
    'slow': 80,
    'medium': 120,
    'fast': 180
}

# Configuración de PWM
PWM_CONFIG = {
    'frequency': 1000,          # Frecuencia PWM en Hz
    'max_duty_cycle': 100       # Duty cycle máximo (%)
}

# ============================================================================
# CONFIGURACIÓN DEL MAPA DE COMPETENCIA
# ============================================================================

# Dimensiones del tablero (según rulebook - en cm)
FIELD_DIMENSIONS = {
    'width': 117.1,             # 1171mm
    'height': 114.3             # 1143mm
}

# Posición inicial del robot
INITIAL_POSITION = {
    'x': 15.0,                  # 15cm desde borde izquierdo
    'y': 15.0,                  # 15cm desde borde inferior
    'heading': 0.0              # Mirando hacia la derecha (0°)
}

# Zonas de competencia
COMPETITION_ZONES = {
    'hospital': {
        'consultation_room': {
            'x_min': 0, 'x_max': 50, 'y_min': 0, 'y_max': 30, 
            'points_red': 8, 'center': (25, 15)
        },
        'waiting_room': {
            'x_min': 0, 'x_max': 70, 'y_min': 0, 'y_max': 50,
            'points_red': 6, 'center': (35, 25)
        },
        'roof': {
            'x_min': 0, 'x_max': 60, 'y_min': 0, 'y_max': 40,
            'points_red': 2, 'center': (30, 20), 'elevated': True
        },
        'parking': {
            'x_min': 0, 'x_max': 80, 'y_min': 0, 'y_max': 60,
            'points_red': 0, 'center': (40, 30)
        }
    },
    'refuge': {
        'upper_level': {
            'x_min': 90, 'x_max': 117, 'y_min': 0, 'y_max': 25,
            'points_green': 4, 'center': (103.5, 12.5), 'elevated': True
        },
        'lower_level': {
            'x_min': 85, 'x_max': 117, 'y_min': 0, 'y_max': 40,
            'points_green': 2, 'center': (101, 20)
        }
    },
    'start_zone': {
        'area': {
            'x_min': 0, 'x_max': 30, 'y_min': 40, 'y_max': 70,
            'center': (15, 55)
        }
    }
}

# Escombros predefinidos (bloques marrones apilados)
DEBRIS_PILES = [
    {'x': 45, 'y': 45, 'width': 12, 'depth': 2.4, 'height': 10.8},
    {'x': 72, 'y': 45, 'width': 12, 'depth': 2.4, 'height': 10.8},
    {'x': 45, 'y': 69, 'width': 12, 'depth': 2.4, 'height': 10.8},
    {'x': 72, 'y': 69, 'width': 12, 'depth': 2.4, 'height': 10.8}
]

# Bloques móviles iniciales
INITIAL_BLOCKS = {
    'red_debris': [  # 7 heridos en escombros
        {'id': 'R1', 'x': 50, 'y': 50}, {'id': 'R2', 'x': 55, 'y': 52},
        {'id': 'R3', 'x': 60, 'y': 48}, {'id': 'R4', 'x': 62, 'y': 55},
        {'id': 'R5', 'x': 58, 'y': 62}, {'id': 'R6', 'x': 65, 'y': 65},
        {'id': 'R7', 'x': 52, 'y': 58}
    ],
    'green_debris': [  # 2 evacuados en escombros
        {'id': 'G1', 'x': 56, 'y': 60}, {'id': 'G2', 'x': 64, 'y': 50}
    ],
    'red_circuit': [  # 4 heridos en circuito
        {'id': 'R8', 'x': 30, 'y': 35}, {'id': 'R9', 'x': 75, 'y': 25},
        {'id': 'R10', 'x': 85, 'y': 45}, {'id': 'R11', 'x': 40, 'y': 75}
    ],
    'green_circuit': [  # 10 evacuados en circuito
        {'id': 'G3', 'x': 25, 'y': 30}, {'id': 'G4', 'x': 35, 'y': 40},
        {'id': 'G5', 'x': 45, 'y': 30}, {'id': 'G6', 'x': 55, 'y': 35},
        {'id': 'G7', 'x': 70, 'y': 30}, {'id': 'G8', 'x': 80, 'y': 35},
        {'id': 'G9', 'x': 90, 'y': 50}, {'id': 'G10', 'x': 85, 'y': 60},
        {'id': 'G11', 'x': 75, 'y': 70}, {'id': 'G12', 'x': 65, 'y': 75}
    ]
}

# ============================================================================
# CONFIGURACIÓN DE LOGGING Y DATOS
# ============================================================================

# Directorios de datos
DATA_PATHS = {
    'base_dir': 'data',
    'sessions': 'data/sessions',
    'calibration': 'data/calibration',
    'maps': 'data/maps'
}

# Configuración de logging
LOGGING_CONFIG = {
    'sample_rate': 10,          # Hz - Frecuencia de muestreo
    'save_format': 'json',      # Formato de guardado
    'auto_save_interval': 30    # Segundos - Guardado automático
}

# ============================================================================
# FUNCIONES HELPER
# ============================================================================

def create_data_directories():
    """Crear directorios de datos si no existen"""
    for path in DATA_PATHS.values():
        os.makedirs(path, exist_ok=True)

def get_sensor_global_angle(sensor_name, robot_heading):
    """Calcular ángulo global de un sensor"""
    sensor_angle = SENSOR_POSITIONS[sensor_name]['angle']
    return (robot_heading + sensor_angle) % 360

def is_point_in_zone(x, y, zone_name, subzone_name=None):
    """Verificar si un punto está en una zona específica"""
    if zone_name not in COMPETITION_ZONES:
        return False
    
    zone_data = COMPETITION_ZONES[zone_name]
    
    if subzone_name:
        if subzone_name not in zone_data:
            return False
        subzone = zone_data[subzone_name]
        return (subzone['x_min'] <= x <= subzone['x_max'] and 
                subzone['y_min'] <= y <= subzone['y_max'])
    
    # Verificar cualquier subzona
    for subzone in zone_data.values():
        if (subzone['x_min'] <= x <= subzone['x_max'] and 
            subzone['y_min'] <= y <= subzone['y_max']):
            return True
    
    return False

def calculate_block_points(block_id, final_x, final_y):
    """Calcular puntos obtenidos por un bloque según su posición final"""
    block_color = block_id[0].upper()  # 'R' o 'G'
    
    if block_color == 'R':  # Bloques rojos - Hospital
        for zone_name, zone_data in COMPETITION_ZONES['hospital'].items():
            if is_point_in_zone(final_x, final_y, 'hospital', zone_name):
                return zone_data.get('points_red', 0)
    
    elif block_color == 'G':  # Bloques verdes - Refugio
        for zone_name, zone_data in COMPETITION_ZONES['refuge'].items():
            if is_point_in_zone(final_x, final_y, 'refuge', zone_name):
                return zone_data.get('points_green', 0)
        
        # Los verdes también pueden ir al hospital con menos puntos
        if is_point_in_zone(final_x, final_y, 'hospital'):
            return 1
    
    return 0

def get_optimal_target_for_block(block_id):
    """Obtener objetivo óptimo para un bloque"""
    block_color = block_id[0].upper()
    
    if block_color == 'R':
        # Priorizar consultation room del hospital (8 puntos)
        target = COMPETITION_ZONES['hospital']['consultation_room']['center']
        return {'x': target[0], 'y': target[1], 'max_points': 8}
    
    elif block_color == 'G':
        # Priorizar upper level del refugio (4 puntos)
        target = COMPETITION_ZONES['refuge']['upper_level']['center']
        return {'x': target[0], 'y': target[1], 'max_points': 4}
    
    return None

# ============================================================================
# VALIDACIÓN DE CONFIGURACIÓN
# ============================================================================

def validate_config():
    """Validar que la configuración sea consistente"""
    errors = []
    
    # Verificar que todos los sensores tengan posición definida
    for sensor_name in ULTRASONIC_PINS.keys():
        if sensor_name not in SENSOR_POSITIONS:
            errors.append(f"Sensor {sensor_name} no tiene posición definida")
    
    # Verificar que las dimensiones sean positivas
    for key, value in ROBOT_DIMENSIONS.items():
        if value <= 0:
            errors.append(f"Dimensión {key} debe ser positiva")
    
    # Verificar que los thresholds sean lógicos
    if DETECTION_THRESHOLDS['block_detect_min'] >= DETECTION_THRESHOLDS['block_detect_max']:
        errors.append("block_detect_min debe ser menor que block_detect_max")
    
    if errors:
        print("Errores en configuración:")
        for error in errors:
            print(f"  - {error}")
        return False
    
    print("Configuración validada correctamente")
    return True

# ============================================================================
# INICIALIZACIÓN
# ============================================================================

if __name__ == "__main__":
    # Crear directorios de datos
    create_data_directories()
    
    # Validar configuración
    validate_config()
    
    print("Configuración cargada:")
    print(f"  - Sensores: {len(ULTRASONIC_PINS)}")
    print(f"  - Dimensiones robot: {ROBOT_DIMENSIONS['length']}x{ROBOT_DIMENSIONS['width']} cm")
    print(f"  - Campo: {FIELD_DIMENSIONS['width']}x{FIELD_DIMENSIONS['height']} cm")
    print(f"  - Bloques totales: {sum(len(blocks) for blocks in INITIAL_BLOCKS.values())}")
    print(f"  - Zonas de puntuación: Hospital, Refugio")