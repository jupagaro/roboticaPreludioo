#!/usr/bin/env python3
"""
hardware/motors.py
Control de motores DC usando driver L298N
"""

import RPi.GPIO as GPIO
import time
import threading
from config import MOTOR_PINS, PWM_CONFIG, ROBOT_DIMENSIONS

class MotorController:
    """Controlador para motores DC con driver L298N"""
    
    def __init__(self):
        # Estado de los motores
        self.left_speed = 0     # Velocidad actual motor izquierdo (-255 a 255)
        self.right_speed = 0    # Velocidad actual motor derecho (-255 a 255)
        self.is_initialized = False
        self.lock = threading.Lock()
        
        # Configurar GPIO
        self._setup_gpio()
        
    def _setup_gpio(self):
        """Configurar pines GPIO para L298N"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Configurar pines como OUTPUT
            pins_to_setup = [
                MOTOR_PINS['ENA'], MOTOR_PINS['IN1'], MOTOR_PINS['IN2'],
                MOTOR_PINS['IN3'], MOTOR_PINS['IN4'], MOTOR_PINS['ENB']
            ]
            
            for pin in pins_to_setup:
                GPIO.setup(pin, GPIO.OUT)
            
            # Crear objetos PWM
            self.pwm_left = GPIO.PWM(MOTOR_PINS['ENA'], PWM_CONFIG['frequency'])
            self.pwm_right = GPIO.PWM(MOTOR_PINS['ENB'], PWM_CONFIG['frequency'])
            
            # Iniciar PWM en 0%
            self.pwm_left.start(0)
            self.pwm_right.start(0)
            
            # Asegurar que motores estén detenidos
            self._set_motor_direction('left', 'stop')
            self._set_motor_direction('right', 'stop')
            
            self.is_initialized = True
            print("✅ Motores L298N inicializados correctamente")
            
        except Exception as e:
            print(f"❌ Error inicializando motores: {e}")
            self.is_initialized = False
    
    def _set_motor_direction(self, motor, direction):
        """Configurar dirección de un motor"""
        if motor == 'left':
            if direction == 'forward':
                GPIO.output(MOTOR_PINS['IN1'], GPIO.HIGH)
                GPIO.output(MOTOR_PINS['IN2'], GPIO.LOW)
            elif direction == 'backward':
                GPIO.output(MOTOR_PINS['IN1'], GPIO.LOW)
                GPIO.output(MOTOR_PINS['IN2'], GPIO.HIGH)
            else:  # stop
                GPIO.output(MOTOR_PINS['IN1'], GPIO.LOW)
                GPIO.output(MOTOR_PINS['IN2'], GPIO.LOW)
                
        elif motor == 'right':
            if direction == 'forward':
                GPIO.output(MOTOR_PINS['IN3'], GPIO.HIGH)
                GPIO.output(MOTOR_PINS['IN4'], GPIO.LOW)
            elif direction == 'backward':
                GPIO.output(MOTOR_PINS['IN3'], GPIO.LOW)
                GPIO.output(MOTOR_PINS['IN4'], GPIO.HIGH)
            else:  # stop
                GPIO.output(MOTOR_PINS['IN3'], GPIO.LOW)
                GPIO.output(MOTOR_PINS['IN4'], GPIO.LOW)
    
    def set_speeds(self, left_speed, right_speed):
        """
        Configurar velocidades de ambos motores
        
        Args:
            left_speed: Velocidad motor izquierdo (-255 a 255)
            right_speed: Velocidad motor derecho (-255 a 255)
        """
        if not self.is_initialized:
            print("⚠️ Motores no inicializados")
            return False
        
        with self.lock:
            # Limitar velocidades
            left_speed = max(-255, min(255, int(left_speed)))
            right_speed = max(-255, min(255, int(right_speed)))
            
            # Guardar velocidades actuales
            self.left_speed = left_speed
            self.right_speed = right_speed
            
            # Configurar motor izquierdo
            if left_speed > 0:
                self._set_motor_direction('left', 'forward')
                duty_cycle = (left_speed / 255.0) * PWM_CONFIG['max_duty_cycle']
            elif left_speed < 0:
                self._set_motor_direction('left', 'backward')
                duty_cycle = (abs(left_speed) / 255.0) * PWM_CONFIG['max_duty_cycle']
            else:
                self._set_motor_direction('left', 'stop')
                duty_cycle = 0
            
            self.pwm_left.ChangeDutyCycle(duty_cycle)
            
            # Configurar motor derecho
            if right_speed > 0:
                self._set_motor_direction('right', 'forward')
                duty_cycle = (right_speed / 255.0) * PWM_CONFIG['max_duty_cycle']
            elif right_speed < 0:
                self._set_motor_direction('right', 'backward')
                duty_cycle = (abs(right_speed) / 255.0) * PWM_CONFIG['max_duty_cycle']
            else:
                self._set_motor_direction('right', 'stop')
                duty_cycle = 0
            
            self.pwm_right.ChangeDutyCycle(duty_cycle)
        
        return True
    
    def move_forward(self, speed=120):
        """Mover hacia adelante"""
        return self.set_speeds(speed, speed)
    
    def move_backward(self, speed=120):
        """Mover hacia atrás"""
        return self.set_speeds(-speed, -speed)
    
    def turn_left(self, speed=100):
        """Girar a la izquierda (motor izquierdo más lento)"""
        return self.set_speeds(speed // 2, speed)
    
    def turn_right(self, speed=100):
        """Girar a la derecha (motor derecho más lento)"""
        return self.set_speeds(speed, speed // 2)
    
    def spin_left(self, speed=100):
        """Girar en el lugar hacia la izquierda"""
        return self.set_speeds(-speed, speed)
    
    def spin_right(self, speed=100):
        """Girar en el lugar hacia la derecha"""
        return self.set_speeds(speed, -speed)
    
    def stop(self):
        """Detener ambos motores"""
        return self.set_speeds(0, 0)
    
    def get_speeds(self):
        """Obtener velocidades actuales"""
        with self.lock:
            return self.left_speed, self.right_speed
    
    def get_status(self):
        """Obtener estado detallado de los motores"""
        left_speed, right_speed = self.get_speeds()
        
        return {
            'initialized': self.is_initialized,
            'left_speed': left_speed,
            'right_speed': right_speed,
            'moving': left_speed != 0 or right_speed != 0,
            'direction': self._get_movement_direction(left_speed, right_speed)
        }
    
    def _get_movement_direction(self, left_speed, right_speed):
        """Determinar dirección del movimiento"""
        if left_speed == 0 and right_speed == 0:
            return 'stopped'
        elif left_speed > 0 and right_speed > 0:
            if left_speed == right_speed:
                return 'forward'
            elif left_speed > right_speed:
                return 'forward_right'
            else:
                return 'forward_left'
        elif left_speed < 0 and right_speed < 0:
            if left_speed == right_speed:
                return 'backward'
            elif left_speed < right_speed:
                return 'backward_right'
            else:
                return 'backward_left'
        elif left_speed < 0 and right_speed > 0:
            return 'spin_left'
        elif left_speed > 0 and right_speed < 0:
            return 'spin_right'
        else:
            return 'mixed'
    
    def test_motors(self, duration=2):
        """
        Rutina de prueba de motores
        
        Args:
            duration: Duración de cada movimiento en segundos
        """
        if not self.is_initialized:
            print("❌ No se pueden probar motores - no inicializados")
            return False
        
        print("🔧 Iniciando prueba de motores...")
        
        test_sequence = [
            ("Adelante lento", 80, 80),
            ("Adelante rápido", 150, 150),
            ("Giro derecha", 100, 50),
            ("Giro izquierda", 50, 100),
            ("Spin derecha", 100, -100),
            ("Spin izquierda", -100, 100),
            ("Atrás", -120, -120)
        ]
        
        try:
            for description, left, right in test_sequence:
                print(f"  {description}...")
                self.set_speeds(left, right)
                time.sleep(duration)
                
            print("  Deteniendo...")
            self.stop()
            print("✅ Prueba de motores completada")
            return True
            
        except KeyboardInterrupt:
            print("\n🛑 Prueba interrumpida")
            self.stop()
            return False
        except Exception as e:
            print(f"❌ Error en prueba: {e}")
            self.stop()
            return False
    
    def calibrate_straight_movement(self, duration=5, target_speed=150):
        """
        Calibración para movimiento recto
        Ayuda a determinar si los motores necesitan compensación
        
        Args:
            duration: Duración del test en segundos
            target_speed: Velocidad objetivo para ambos motores
        """
        print(f"🔧 Calibración de movimiento recto ({duration}s a velocidad {target_speed})")
        print("Coloca el robot en línea recta y observa si se desvía")
        
        input("Presiona Enter para comenzar...")
        
        print("¡Moviendo recto!")
        self.set_speeds(target_speed, target_speed)
        time.sleep(duration)
        self.stop()
        
        print("\nResultados de la calibración:")
        print("- Si el robot se desvía a la DERECHA: motor izquierdo es más lento")
        print("- Si el robot se desvía a la IZQUIERDA: motor derecho es más lento")
        print("- Si va recto: motores bien balanceados")
        
        deviation = input("¿Se desvió? (izquierda/derecha/recto): ").strip().lower()
        
        if deviation == 'derecha':
            print("💡 Sugerencia: En el código, usar factor de compensación para motor izquierdo")
            print("   Ejemplo: left_speed = int(left_speed * 1.05)")
        elif deviation == 'izquierda':
            print("💡 Sugerencia: En el código, usar factor de compensación para motor derecho")
            print("   Ejemplo: right_speed = int(right_speed * 1.05)")
        else:
            print("✅ Motores bien balanceados")
    
    def emergency_stop(self):
        """Parada de emergencia - detiene inmediatamente"""
        try:
            self.pwm_left.ChangeDutyCycle(0)
            self.pwm_right.ChangeDutyCycle(0)
            self._set_motor_direction('left', 'stop')
            self._set_motor_direction('right', 'stop')
            self.left_speed = 0
            self.right_speed = 0
            print("🛑 Parada de emergencia activada")
        except Exception as e:
            print(f"❌ Error en parada de emergencia: {e}")
    
    def cleanup(self):
        """Limpiar recursos GPIO"""
        if self.is_initialized:
            try:
                self.stop()
                self.pwm_left.stop()
                self.pwm_right.stop()
                print("🧹 Recursos de motores liberados")
            except Exception as e:
                print(f"⚠️ Error en cleanup de motores: {e}")

# ============================================================================
# FUNCIONES DE PRUEBA
# ============================================================================

def test_motor_controller():
    """Función de prueba independiente"""
    print("🎯 Prueba independiente del controlador de motores")
    
    try:
        motor = MotorController()
        
        if not motor.is_initialized:
            print("❌ No se pudieron inicializar motores")
            return
        
        print("\nEstado inicial:")
        print(f"Estado: {motor.get_status()}")
        
        # Menú de pruebas
        while True:
            print("\nOpciones de prueba:")
            print("1. Adelante")
            print("2. Atrás") 
            print("3. Girar izquierda")
            print("4. Girar derecha")
            print("5. Prueba automática")
            print("6. Calibrar movimiento recto")
            print("0. Salir")
            
            choice = input("Elegir opción: ").strip()
            
            if choice == '1':
                speed = int(input("Velocidad (1-255): ") or "120")
                motor.move_forward(speed)
                input("Presiona Enter para parar...")
                motor.stop()
                
            elif choice == '2':
                speed = int(input("Velocidad (1-255): ") or "120")
                motor.move_backward(speed)
                input("Presiona Enter para parar...")
                motor.stop()
                
            elif choice == '3':
                speed = int(input("Velocidad (1-255): ") or "100")
                motor.spin_left(speed)
                input("Presiona Enter para parar...")
                motor.stop()
                
            elif choice == '4':
                speed = int(input("Velocidad (1-255): ") or "100")
                motor.spin_right(speed)
                input("Presiona Enter para parar...")
                motor.stop()
                
            elif choice == '5':
                motor.test_motors()
                
            elif choice == '6':
                motor.calibrate_straight_movement()
                
            elif choice == '0':
                break
            
            print(f"Estado actual: {motor.get_status()}")
    
    except KeyboardInterrupt:
        print("\n🛑 Prueba interrumpida")
    
    finally:
        if 'motor' in locals():
            motor.cleanup()
        try:
            GPIO.cleanup()
        except:
            pass

if __name__ == "__main__":
    test_motor_controller()