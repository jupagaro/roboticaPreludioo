#!/usr/bin/env python3
"""
battery_compensation.py
Sistema de compensación dinámica de batería
Monitorea velocidad real vs esperada y ajusta PWM automáticamente
"""

import time
from math import sqrt
from config import CALIBRATION, ROBOT_DIMENSIONS


class BatteryCompensator:
    """
    Monitorea el desempeño del robot y ajusta velocidades para compensar
    caídas de voltaje de batería
    """

    def __init__(self, initial_speed_factor=1.0, enable_logging=True):
        """
        Args:
            initial_speed_factor: Factor inicial de velocidad (1.0 = sin corrección)
            enable_logging: Si se deben imprimir mensajes de compensación
        """
        # Factor de compensación actual (multiplicador de PWM)
        self.speed_factor = initial_speed_factor

        # Historial de correcciones
        self.correction_history = []

        # Umbrales de compensación
        self.min_speed_factor = 0.5   # Mínimo 50% de velocidad base
        self.max_speed_factor = 1.5   # Máximo 150% de velocidad base

        # Tolerancias para detección
        self.speed_tolerance = 0.15   # 15% de tolerancia antes de corregir
        self.min_samples = 3          # Mínimo de mediciones antes de ajustar

        # Buffer de mediciones recientes
        self.recent_measurements = []
        self.max_buffer_size = 5

        # Logging
        self.enable_logging = enable_logging

        # Estadísticas
        self.total_corrections = 0
        self.total_measurements = 0

        if enable_logging:
            print("🔋 Battery Compensator inicializado")
            print(f"   Factor inicial: {self.speed_factor:.2f}")

    def measure_performance(self, commanded_speed, duration, start_pos, end_pos, movement_type='linear'):
        """
        Medir el desempeño de un movimiento y actualizar factor de compensación

        Args:
            commanded_speed: Velocidad PWM comandada (0-255)
            duration: Duración del movimiento (segundos)
            start_pos: Posición inicial {'x', 'y', 'heading'}
            end_pos: Posición final {'x', 'y', 'heading'}
            movement_type: 'linear' para forward/backward, 'angular' para spins

        Returns:
            dict: Información del análisis de desempeño
        """
        self.total_measurements += 1

        # Calcular desempeño esperado vs real
        if movement_type == 'linear':
            expected_distance = self._calculate_expected_linear_distance(commanded_speed, duration)
            actual_distance = self._calculate_actual_linear_distance(start_pos, end_pos)

            performance_ratio = actual_distance / expected_distance if expected_distance > 0 else 0
            metric_name = "distancia"
            metric_expected = expected_distance
            metric_actual = actual_distance

        elif movement_type == 'angular':
            expected_rotation = self._calculate_expected_rotation(commanded_speed, duration)
            actual_rotation = self._calculate_actual_rotation(start_pos, end_pos)

            performance_ratio = actual_rotation / expected_rotation if expected_rotation > 0 else 0
            metric_name = "rotación"
            metric_expected = expected_rotation
            metric_actual = actual_rotation
        else:
            return {'error': 'Invalid movement type'}

        # Almacenar medición
        measurement = {
            'timestamp': time.time(),
            'commanded_speed': commanded_speed,
            'duration': duration,
            'movement_type': movement_type,
            'expected': metric_expected,
            'actual': metric_actual,
            'performance_ratio': performance_ratio,
            'speed_factor_at_time': self.speed_factor
        }

        self.recent_measurements.append(measurement)

        # Mantener buffer limitado
        if len(self.recent_measurements) > self.max_buffer_size:
            self.recent_measurements.pop(0)

        # Decidir si ajustar factor de compensación
        should_adjust, new_factor = self._should_adjust_compensation(performance_ratio)

        if should_adjust:
            old_factor = self.speed_factor
            self.speed_factor = new_factor
            self.total_corrections += 1

            if self.enable_logging:
                print(f"\n🔧 COMPENSACIÓN DE BATERÍA")
                print(f"   {metric_name.capitalize()} esperada: {metric_expected:.1f}")
                print(f"   {metric_name.capitalize()} real: {metric_actual:.1f}")
                print(f"   Desempeño: {performance_ratio*100:.1f}%")
                print(f"   Factor: {old_factor:.2f} → {new_factor:.2f}")

            self.correction_history.append({
                'timestamp': time.time(),
                'old_factor': old_factor,
                'new_factor': new_factor,
                'reason': f"Performance {performance_ratio*100:.1f}%"
            })

        return {
            'performance_ratio': performance_ratio,
            'expected': metric_expected,
            'actual': metric_actual,
            'adjustment_made': should_adjust,
            'current_speed_factor': self.speed_factor
        }

    def _calculate_expected_linear_distance(self, pwm_speed, duration):
        """
        Calcular distancia lineal esperada basada en PWM y duración

        Args:
            pwm_speed: Velocidad PWM (0-255)
            duration: Duración en segundos

        Returns:
            float: Distancia esperada en cm
        """
        # Convertir PWM a velocidad lineal (cm/s)
        linear_velocity = (pwm_speed / 255.0) * CALIBRATION['speed_factor'] * 100

        # Distancia = velocidad * tiempo
        expected_distance = linear_velocity * duration

        return expected_distance

    def _calculate_actual_linear_distance(self, start_pos, end_pos):
        """
        Calcular distancia real recorrida

        Args:
            start_pos: {'x', 'y', 'heading'}
            end_pos: {'x', 'y', 'heading'}

        Returns:
            float: Distancia en cm
        """
        dx = end_pos['x'] - start_pos['x']
        dy = end_pos['y'] - start_pos['y']

        return sqrt(dx**2 + dy**2)

    def _calculate_expected_rotation(self, pwm_speed, duration):
        """
        Calcular rotación angular esperada basada en PWM y duración

        Args:
            pwm_speed: Velocidad PWM (0-255)
            duration: Duración en segundos

        Returns:
            float: Rotación esperada en grados (valor absoluto)
        """
        # Velocidad lineal de las ruedas
        linear_velocity = (pwm_speed / 255.0) * CALIBRATION['speed_factor'] * 100  # cm/s

        # Velocidad angular del robot (rad/s)
        # Para spin: una rueda avanza, otra retrocede
        angular_velocity_rad = (2 * linear_velocity) / ROBOT_DIMENSIONS['wheel_base']

        # Convertir a grados/s
        from math import degrees
        angular_velocity_deg = degrees(angular_velocity_rad)

        # Rotación total
        expected_rotation = abs(angular_velocity_deg * duration)

        return expected_rotation

    def _calculate_actual_rotation(self, start_pos, end_pos):
        """
        Calcular rotación real del robot

        Args:
            start_pos: {'x', 'y', 'heading'}
            end_pos: {'x', 'y', 'heading'}

        Returns:
            float: Rotación en grados (valor absoluto)
        """
        delta_heading = end_pos['heading'] - start_pos['heading']

        # Normalizar a -180, 180
        while delta_heading > 180:
            delta_heading -= 360
        while delta_heading < -180:
            delta_heading += 360

        return abs(delta_heading)

    def _should_adjust_compensation(self, performance_ratio):
        """
        Determinar si se debe ajustar el factor de compensación

        Args:
            performance_ratio: Ratio actual/esperado

        Returns:
            tuple: (should_adjust: bool, new_factor: float)
        """
        # No ajustar si no tenemos suficientes muestras
        if len(self.recent_measurements) < self.min_samples:
            return False, self.speed_factor

        # Calcular promedio de performance reciente
        recent_ratios = [m['performance_ratio'] for m in self.recent_measurements]
        avg_performance = sum(recent_ratios) / len(recent_ratios)

        # Determinar si está fuera de tolerancia
        deviation = 1.0 - avg_performance

        if abs(deviation) < self.speed_tolerance:
            # Desempeño aceptable, no ajustar
            return False, self.speed_factor

        # Calcular nuevo factor
        # Si avg_performance < 1.0: robot va más lento, necesita más potencia
        # Si avg_performance > 1.0: robot va más rápido, necesita menos potencia
        adjustment = 1.0 / avg_performance if avg_performance > 0 else 1.0
        new_factor = self.speed_factor * adjustment

        # Aplicar límites
        new_factor = max(self.min_speed_factor, min(self.max_speed_factor, new_factor))

        # Solo ajustar si el cambio es significativo (>5%)
        if abs(new_factor - self.speed_factor) < 0.05:
            return False, self.speed_factor

        return True, new_factor

    def compensate_speed(self, commanded_speed):
        """
        Aplicar compensación a una velocidad comandada

        Args:
            commanded_speed: Velocidad PWM original (0-255)

        Returns:
            int: Velocidad PWM compensada (0-255)
        """
        compensated = int(commanded_speed * self.speed_factor)

        # Asegurar que está en rango válido
        return max(0, min(255, compensated))

    def get_statistics(self):
        """Obtener estadísticas del compensador"""
        avg_factor = sum(m['speed_factor_at_time'] for m in self.recent_measurements) / len(self.recent_measurements) if self.recent_measurements else self.speed_factor

        avg_performance = sum(m['performance_ratio'] for m in self.recent_measurements) / len(self.recent_measurements) if self.recent_measurements else 0

        return {
            'current_speed_factor': self.speed_factor,
            'total_measurements': self.total_measurements,
            'total_corrections': self.total_corrections,
            'recent_measurements': len(self.recent_measurements),
            'average_speed_factor': avg_factor,
            'average_performance': avg_performance * 100,  # Como porcentaje
            'correction_history_count': len(self.correction_history)
        }

    def print_statistics(self):
        """Imprimir estadísticas de compensación"""
        stats = self.get_statistics()

        print("\n" + "=" * 60)
        print("ESTADÍSTICAS DE COMPENSACIÓN DE BATERÍA")
        print("=" * 60)
        print(f"Factor de velocidad actual: {stats['current_speed_factor']:.2f}")
        print(f"Total de mediciones: {stats['total_measurements']}")
        print(f"Correcciones aplicadas: {stats['total_corrections']}")
        print(f"Desempeño promedio reciente: {stats['average_performance']:.1f}%")

        if self.correction_history:
            print(f"\nÚltimas {min(3, len(self.correction_history))} correcciones:")
            for correction in self.correction_history[-3:]:
                print(f"  {correction['old_factor']:.2f} → {correction['new_factor']:.2f} "
                      f"({correction['reason']})")

        print("=" * 60)

    def reset(self):
        """Resetear compensador a estado inicial"""
        self.speed_factor = 1.0
        self.recent_measurements = []
        self.correction_history = []
        self.total_corrections = 0
        self.total_measurements = 0

        if self.enable_logging:
            print("🔄 Battery Compensator reseteado")


# ============================================================================
# FUNCIONES DE UTILIDAD
# ============================================================================

def test_compensator():
    """Prueba del compensador con datos simulados"""
    print("🧪 Prueba del Battery Compensator\n")

    compensator = BatteryCompensator(enable_logging=True)

    # Simular movimientos con batería degradándose
    print("Simulando movimientos con batería bajando...\n")

    # Movimiento 1: Batería al 100%
    start1 = {'x': 0, 'y': 0, 'heading': 0}
    end1 = {'x': 27, 'y': 0, 'heading': 0}  # Movimiento normal
    compensator.measure_performance(255, 1.0, start1, end1, 'linear')

    # Movimiento 2: Batería al 80%
    start2 = {'x': 0, 'y': 0, 'heading': 0}
    end2 = {'x': 22, 'y': 0, 'heading': 0}  # 20% más lento
    compensator.measure_performance(255, 1.0, start2, end2, 'linear')

    # Movimiento 3: Batería al 70%
    start3 = {'x': 0, 'y': 0, 'heading': 0}
    end3 = {'x': 19, 'y': 0, 'heading': 0}  # 30% más lento
    result = compensator.measure_performance(255, 1.0, start3, end3, 'linear')

    print(f"\nPrueba de velocidad compensada:")
    original_speed = 255
    compensated_speed = compensator.compensate_speed(original_speed)
    print(f"Velocidad original: {original_speed}")
    print(f"Velocidad compensada: {compensated_speed}")
    print(f"Incremento: +{compensated_speed - original_speed} PWM")

    compensator.print_statistics()

    print("\n✅ Prueba completada")


if __name__ == "__main__":
    test_compensator()
