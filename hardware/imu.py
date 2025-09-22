#!/usr/bin/env python3
"""
hardware/imu.py
Controlador para acelerómetro/giroscopio MPU6050
Proporciona datos de orientación para sensor fusion
"""

import smbus
import time
import threading
from math import degrees, radians
from config import IMU_CONFIG, CALIBRATION

class IMUController:
    """Controlador para MPU6050 (acelerómetro + giroscopio)"""
    
    def __init__(self):
        self.bus = None
        self.is_connected = False
        self.lock = threading.Lock()
        
        # Datos de calibración
        self.gyro_z_offset = 0
        self.accel_offsets = {'x': 0, 'y': 0, 'z': 0}
        
        # Últimas lecturas
        self.last_gyro = {'x': 0, 'y': 0, 'z': 0}
        self.last_accel = {'x': 0, 'y': 0, 'z': 0}
        self.last_temp = 0
        self.last_update = 0
        
        # Ángulo integrado
        self.integrated_heading = 0
        self.last_integration_time = time.time()
        
        self._initialize_mpu6050()
    
    def _initialize_mpu6050(self):
        """Inicializar conexión I2C con MPU6050"""
        try:
            self.bus = smbus.SMBus(IMU_CONFIG['bus'])
            
            # Despertar MPU6050 (registro 0x6B = Power Management)
            self.bus.write_byte_data(IMU_CONFIG['address'], 0x6B, 0)
            
            # Configurar giroscopio (±250°/s)
            self.bus.write_byte_data(IMU_CONFIG['address'], 0x1B, 0)
            
            # Configurar acelerómetro (±2g)
            self.bus.write_byte_data(IMU_CONFIG['address'], 0x1C, 0)
            
            time.sleep(0.1)  # Esperar estabilización
            
            # Verificar conexión
            who_am_i = self.bus.read_byte_data(IMU_CONFIG['address'], 0x75)
            if who_am_i == 0x68:  # MPU6050 responde con 0x68
                self.is_connected = True
                print("✅ MPU6050 inicializado correctamente")
            else:
                print(f"⚠️ MPU6050 respuesta inesperada: 0x{who_am_i:02X}")
                self.is_connected = False
                
        except Exception as e:
            print(f"❌ Error conectando MPU6050: {e}")
            self.is_connected = False
    
    def _read_raw_data(self, register_high):
        """
        Leer datos raw de 16 bits desde registros consecutivos
        
        Args:
            register_high: Registro alto (el bajo será register_high + 1)
            
        Returns:
            int: Valor con signo de 16 bits
        """
        if not self.is_connected:
            return 0
        
        try:
            high = self.bus.read_byte_data(IMU_CONFIG['address'], register_high)
            low = self.bus.read_byte_data(IMU_CONFIG['address'], register_high + 1)
            
            # Combinar bytes y convertir a signed
            value = (high << 8) | low
            if value > 32768:
                value = value - 65536
                
            return value
            
        except Exception as e:
            return 0
    
    def read_gyro_raw(self):
        """
        Leer datos raw del giroscopio
        
        Returns:
            dict: {'x': raw_x, 'y': raw_y, 'z': raw_z}
        """
        return {
            'x': self._read_raw_data(0x43),  # GYRO_XOUT_H
            'y': self._read_raw_data(0x45),  # GYRO_YOUT_H
            'z': self._read_raw_data(0x47)   # GYRO_ZOUT_H
        }
    
    def read_accel_raw(self):
        """
        Leer datos raw del acelerómetro
        
        Returns:
            dict: {'x': raw_x, 'y': raw_y, 'z': raw_z}
        """
        return {
            'x': self._read_raw_data(0x3B),  # ACCEL_XOUT_H
            'y': self._read_raw_data(0x3D),  # ACCEL_YOUT_H
            'z': self._read_raw_data(0x3F)   # ACCEL_ZOUT_H
        }
    
    def read_temperature_raw(self):
        """
        Leer temperatura raw
        
        Returns:
            int: Temperatura raw
        """
        return self._read_raw_data(0x41)  # TEMP_OUT_H
    
    def read_gyro_scaled(self):
        """
        Leer giroscopio en grados/segundo
        
        Returns:
            dict: {'x': deg/s, 'y': deg/s, 'z': deg/s}
        """
        if not self.is_connected:
            return {'x': 0, 'y': 0, 'z': 0}
        
        raw = self.read_gyro_raw()
        
        # Convertir a grados/segundo (sensibilidad ±250°/s = 131 LSB/°/s)
        scaled = {
            'x': raw['x'] / 131.0,
            'y': raw['y'] / 131.0,
            'z': (raw['z'] / 131.0) - self.gyro_z_offset  # Aplicar offset de calibración
        }
        
        with self.lock:
            self.last_gyro = scaled.copy()
            self.last_update = time.time()
        
        return scaled
    
    def read_accel_scaled(self):
        """
        Leer acelerómetro en g (gravedad terrestre)
        
        Returns:
            dict: {'x': g, 'y': g, 'z': g}
        """
        if not self.is_connected:
            return {'x': 0, 'y': 0, 'z': 1}  # Simular gravedad en Z
        
        raw = self.read_accel_raw()
        
        # Convertir a g (sensibilidad ±2g = 16384 LSB/g)
        scaled = {
            'x': (raw['x'] / 16384.0) - self.accel_offsets['x'],
            'y': (raw['y'] / 16384.0) - self.accel_offsets['y'],
            'z': (raw['z'] / 16384.0) - self.accel_offsets['z']
        }
        
        with self.lock:
            self.last_accel = scaled.copy()
        
        return scaled
    
    def read_temperature_celsius(self):
        """
        Leer temperatura en grados Celsius
        
        Returns:
            float: Temperatura en °C
        """
        if not self.is_connected:
            return 25.0  # Temperatura ambiente por defecto
        
        raw = self.read_temperature_raw()
        temp_c = (raw / 340.0) + 36.53  # Fórmula del datasheet
        
        with self.lock:
            self.last_temp = temp_c
        
        return temp_c
    
    def get_heading_rate(self):
        """
        Obtener velocidad de cambio de heading (grados/segundo)
        Para navegación - solo eje Z (yaw)
        
        Returns:
            float: Velocidad angular en grados/segundo
        """
        gyro = self.read_gyro_scaled()
        return gyro['z']
    
    def update_integrated_heading(self, dt=None):
        """
        Actualizar heading integrado desde giroscopio
        
        Args:
            dt: Delta tiempo. Si None, se calcula automáticamente
            
        Returns:
            float: Heading integrado en grados
        """
        current_time = time.time()
        
        if dt is None:
            dt = current_time - self.last_integration_time
        
        gyro_rate = self.get_heading_rate()
        
        # Integrar velocidad angular
        self.integrated_heading += gyro_rate * dt
        
        # Normalizar a [-180, 180]
        while self.integrated_heading > 180:
            self.integrated_heading -= 360
        while self.integrated_heading < -180:
            self.integrated_heading += 360
        
        self.last_integration_time = current_time
        
        return self.integrated_heading
    
    def reset_integrated_heading(self, new_heading=0):
        """Resetear heading integrado a un valor específico"""
        self.integrated_heading = new_heading
        self.last_integration_time = time.time()
        print(f"🧭 Heading integrado reseteado a {new_heading}°")
    
    def calibrate_gyro_z(self, samples=100, duration=None):
        """
        Calibrar offset del giroscopio Z (para eliminar deriva)
        
        Args:
            samples: Número de muestras para promedio
            duration: Duración máxima en segundos (None = usar samples)
        """
        if not self.is_connected:
            print("❌ No se puede calibrar - MPU6050 no conectado")
            return False
        
        print(f"🔧 Calibrando giroscopio Z... (robot debe estar QUIETO)")
        print("Obteniendo muestras...")
        
        readings = []
        start_time = time.time()
        
        for i in range(samples):
            if duration and (time.time() - start_time) > duration:
                break
                
            raw = self.read_gyro_raw()
            readings.append(raw['z'] / 131.0)
            
            if i % 10 == 0:
                print(f"Muestra {i+1}/{samples}")
            
            time.sleep(0.1)
        
        # Calcular offset
        if readings:
            self.gyro_z_offset = sum(readings) / len(readings)
            print(f"✅ Calibración completada")
            print(f"Offset giroscopio Z: {self.gyro_z_offset:.3f} °/s")
            print(f"Muestras utilizadas: {len(readings)}")
            return True
        else:
            print("❌ No se pudieron obtener muestras")
            return False
    
    def calibrate_accelerometer(self, samples=50):
        """
        Calibración básica del acelerómetro
        Asume que el robot está en superficie plana
        """
        if not self.is_connected:
            print("❌ No se puede calibrar - MPU6050 no conectado")
            return False
        
        print(f"🔧 Calibrando acelerómetro... (robot en superficie PLANA)")
        
        readings = {'x': [], 'y': [], 'z': []}
        
        for i in range(samples):
            accel = self.read_accel_scaled()
            
            # No aplicar offsets durante calibración
            raw = self.read_accel_raw()
            readings['x'].append(raw['x'] / 16384.0)
            readings['y'].append(raw['y'] / 16384.0) 
            readings['z'].append(raw['z'] / 16384.0)
            
            if i % 10 == 0:
                print(f"Muestra {i+1}/{samples}")
            
            time.sleep(0.05)
        
        # Calcular offsets (X e Y deberían ser ~0, Z debería ser ~1g)
        self.accel_offsets['x'] = sum(readings['x']) / len(readings['x'])
        self.accel_offsets['y'] = sum(readings['y']) / len(readings['y'])
        self.accel_offsets['z'] = (sum(readings['z']) / len(readings['z'])) - 1.0
        
        print("✅ Acelerómetro calibrado")
        print(f"Offsets: X={self.accel_offsets['x']:.3f}, Y={self.accel_offsets['y']:.3f}, Z={self.accel_offsets['z']:.3f}")
        
        return True
    
    def get_sensor_status(self):
        """Obtener estado completo del sensor"""
        with self.lock:
            return {
                'connected': self.is_connected,
                'last_gyro': self.last_gyro.copy(),
                'last_accel': self.last_accel.copy(),
                'last_temp': self.last_temp,
                'last_update': self.last_update,
                'integrated_heading': self.integrated_heading,
                'gyro_z_offset': self.gyro_z_offset,
                'accel_offsets': self.accel_offsets.copy()
            }
    
    def test_sensor(self, duration=10):
        """
        Rutina de prueba del sensor
        
        Args:
            duration: Duración de la prueba en segundos
        """
        if not self.is_connected:
            print("❌ MPU6050 no conectado - no se puede probar")
            return
        
        print(f"🔧 Probando MPU6050 por {duration} segundos...")
        print("Mueve/gira el robot para ver cambios en los valores")
        
        start_time = time.time()
        last_heading_update = start_time
        
        try:
            while (time.time() - start_time) < duration:
                current_time = time.time()
                
                # Leer sensores
                gyro = self.read_gyro_scaled()
                accel = self.read_accel_scaled()
                temp = self.read_temperature_celsius()
                
                # Actualizar heading integrado
                if current_time - last_heading_update >= 0.1:  # 10Hz
                    self.update_integrated_heading()
                    last_heading_update = current_time
                
                # Mostrar datos
                print(f"\r", end="")
                print(f"Gyro: X={gyro['x']:6.2f} Y={gyro['y']:6.2f} Z={gyro['z']:6.2f} °/s | ", end="")
                print(f"Accel: X={accel['x']:5.2f} Y={accel['y']:5.2f} Z={accel['z']:5.2f} g | ", end="")
                print(f"Temp: {temp:4.1f}°C | ", end="")
                print(f"Heading: {self.integrated_heading:6.1f}°", end="", flush=True)
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print(f"\n🛑 Prueba interrumpida")
        
        print(f"\n✅ Prueba completada")

# ============================================================================
# FUNCIONES DE PRUEBA
# ============================================================================

def test_imu_controller():
    """Función de prueba independiente"""
    print("🎯 Prueba independiente del controlador IMU")
    
    try:
        # Crear controlador
        imu = IMUController()
        
        if not imu.is_connected:
            print("❌ MPU6050 no disponible")
            return
        
        # Menú de pruebas
        while True:
            print("\nOpciones de prueba:")
            print("1. Lectura única de sensores")
            print("2. Monitoreo continuo") 
            print("3. Calibrar giroscopio Z")
            print("4. Calibrar acelerómetro")
            print("5. Probar integración de heading")
            print("6. Estado del sensor")
            print("7. Solo giroscopio Z (para navegación)")
            print("0. Salir")
            
            choice = input("Elegir opción: ").strip()
            
            if choice == '1':
                gyro = imu.read_gyro_scaled()
                accel = imu.read_accel_scaled()
                temp = imu.read_temperature_celsius()
                
                print(f"\n📊 Lecturas actuales:")
                print(f"Giroscopio (°/s): X={gyro['x']:6.2f}, Y={gyro['y']:6.2f}, Z={gyro['z']:6.2f}")
                print(f"Acelerómetro (g): X={accel['x']:6.2f}, Y={accel['y']:6.2f}, Z={accel['z']:6.2f}")
                print(f"Temperatura: {temp:.1f}°C")
            
            elif choice == '2':
                duration = int(input("Duración (segundos): ") or "10")
                imu.test_sensor(duration)
            
            elif choice == '3':
                samples = int(input("Número de muestras (default 100): ") or "100")
                imu.calibrate_gyro_z(samples)
            
            elif choice == '4':
                samples = int(input("Número de muestras (default 50): ") or "50")
                imu.calibrate_accelerometer(samples)
            
            elif choice == '5':
                print("Prueba de integración de heading - gira el robot")
                print("Ctrl+C para parar")
                
                imu.reset_integrated_heading(0)
                
                try:
                    while True:
                        heading = imu.update_integrated_heading()
                        gyro_rate = imu.get_heading_rate()
                        
                        print(f"\rGyro Z: {gyro_rate:7.2f}°/s | Heading integrado: {heading:7.1f}°", 
                              end="", flush=True)
                        time.sleep(0.1)
                        
                except KeyboardInterrupt:
                    print("\nPrueba terminada")
            
            elif choice == '6':
                status = imu.get_sensor_status()
                print(f"\n📊 Estado del sensor:")
                print(f"Conectado: {'✅' if status['connected'] else '❌'}")
                print(f"Último gyro (°/s): X={status['last_gyro']['x']:.2f}, Y={status['last_gyro']['y']:.2f}, Z={status['last_gyro']['z']:.2f}")
                print(f"Último accel (g): X={status['last_accel']['x']:.2f}, Y={status['last_accel']['y']:.2f}, Z={status['last_accel']['z']:.2f}")
                print(f"Temperatura: {status['last_temp']:.1f}°C")
                print(f"Heading integrado: {status['integrated_heading']:.1f}°")
                print(f"Offset gyro Z: {status['gyro_z_offset']:.3f}")
                
                if status['last_update'] > 0:
                    age = time.time() - status['last_update']
                    print(f"Última actualización: hace {age:.1f}s")
            
            elif choice == '7':
                print("Solo giroscopio Z para navegación - Ctrl+C para parar")
                
                try:
                    while True:
                        gyro_z = imu.get_heading_rate()
                        status = "Girando" if abs(gyro_z) > 5 else "Quieto"
                        print(f"\rGyro Z: {gyro_z:7.2f}°/s ({status})", end="", flush=True)
                        time.sleep(0.1)
                        
                except KeyboardInterrupt:
                    print("\nMonitoreo terminado")
            
            elif choice == '0':
                break
    
    except KeyboardInterrupt:
        print("\n🛑 Prueba interrumpida")
    
    except Exception as e:
        print(f"❌ Error en prueba: {e}")

if __name__ == "__main__":
    test_imu_controller()