#!/usr/bin/env python3
"""
movement_recorder.py
Sistema de grabación y reproducción de secuencias de movimiento
Permite entrenar al robot sin necesidad de sensores
"""

import time
import json
import os
from datetime import datetime
from hardware import MotorController
from config import DATA_PATHS, create_data_directories
from sensor_fusion import SensorFusion
from utils.logging import DataLogger

class MovementRecorder:
    """Grabador y reproductor de secuencias de movimiento"""

    # Acciones disponibles
    AVAILABLE_ACTIONS = {
        'forward': 'Avanzar hacia adelante',
        'backward': 'Retroceder',
        'spin_left': 'Girar en el lugar (izquierda)',
        'spin_right': 'Girar en el lugar (derecha)',
        'turn_left': 'Curva suave (izquierda)',
        'turn_right': 'Curva suave (derecha)',
        'stop': 'Detener (pausa)'
    }

    def __init__(self, enable_sensors=False):
        """
        Inicializar Movement Recorder

        Args:
            enable_sensors (bool): Si True, inicializa sensor fusion para grabaciones enriquecidas
        """
        if enable_sensors:
            print("Inicializando Movement Recorder (con sensores)...")
        else:
            print("Inicializando Movement Recorder (solo motores)...")

        self.motors = MotorController()
        self.enable_sensors = enable_sensors
        self.sensor_fusion = None
        self.data_logger = None

        if not self.motors.is_initialized:
            raise Exception("Error: No se pudieron inicializar los motores")

        # Inicializar sensor fusion si está habilitado
        if enable_sensors:
            try:
                self.sensor_fusion = SensorFusion(motors=self.motors)
                print("✓ Sensor fusion inicializado")
            except Exception as e:
                print(f"⚠ Advertencia: No se pudo inicializar sensor fusion: {e}")
                print("  Continuando en modo solo-motores")
                self.enable_sensors = False

        # Secuencia actual en memoria
        self.current_sequence = {
            'name': None,
            'created_at': None,
            'steps': [],
            'total_duration': 0
        }

        # Crear directorio de movimientos
        create_data_directories()

        print("Movement Recorder inicializado correctamente")

    # ========================================================================
    # GRABACIÓN DE SECUENCIAS
    # ========================================================================

    def record_sequence(self):
        """Grabar nueva secuencia paso a paso (modo interactivo)"""
        print("\n" + "=" * 60)
        print("GRABACIÓN DE SECUENCIA DE MOVIMIENTO")
        print("=" * 60)

        # Solicitar nombre de secuencia
        sequence_name = input("Nombre de la secuencia: ").strip()
        if not sequence_name:
            print("Error: Debes proporcionar un nombre")
            return False

        # Inicializar nueva secuencia
        self.current_sequence = {
            'name': sequence_name,
            'created_at': datetime.now().isoformat(),
            'steps': [],
            'total_duration': 0
        }

        print(f"\nGrabando secuencia: '{sequence_name}'")
        print("Añade pasos uno por uno. Presiona Enter sin valor para terminar.\n")

        step_number = 1

        while True:
            print(f"\n--- Paso {step_number} ---")

            # Seleccionar acción
            action = self._select_action()
            if action is None:
                break  # Usuario terminó de grabar

            # Solicitar velocidad
            speed = self._input_speed(action)
            if speed is None:
                break

            # Solicitar duración
            duration = self._input_duration()
            if duration is None:
                break

            # Crear paso
            step = {
                'action': action,
                'speed': speed,
                'duration': duration
            }

            # Opción de probar el paso
            if input("¿Probar este paso? [y/N]: ").lower() == 'y':
                print("Ejecutando paso...")
                self._execute_step(step)
                print("Paso completado")

            # Añadir paso a la secuencia
            self.current_sequence['steps'].append(step)
            self.current_sequence['total_duration'] += duration

            print(f"✓ Paso {step_number} añadido: {action} @ {speed}% por {duration}s")
            step_number += 1

        # Resumen de la secuencia grabada
        if self.current_sequence['steps']:
            print(f"\n" + "=" * 60)
            print(f"Secuencia '{sequence_name}' completada")
            print(f"Total de pasos: {len(self.current_sequence['steps'])}")
            print(f"Duración total: {self.current_sequence['total_duration']:.1f}s")
            print("=" * 60)
            return True
        else:
            print("No se grabó ningún paso")
            return False

    def _select_action(self):
        """Seleccionar tipo de acción"""
        print("\nAcciones disponibles:")
        actions_list = list(self.AVAILABLE_ACTIONS.keys())

        for i, (action, description) in enumerate(self.AVAILABLE_ACTIONS.items(), 1):
            print(f"  {i}. {action:12} - {description}")

        choice = input("Elegir acción [1-7, Enter para terminar]: ").strip()

        if not choice:
            return None

        try:
            idx = int(choice) - 1
            if 0 <= idx < len(actions_list):
                return actions_list[idx]
            else:
                print("Opción inválida")
                return self._select_action()
        except ValueError:
            print("Entrada inválida")
            return self._select_action()

    def _input_speed(self, action):
        """Solicitar velocidad (0-255, convertido a porcentaje para display)"""
        if action == 'stop':
            return 0  # Paradas no necesitan velocidad

        while True:
            speed_input = input(f"Velocidad [0-255, Enter=150]: ").strip()

            if not speed_input:
                return 150  # Valor por defecto

            if speed_input.lower() == 'q':
                return None  # Cancelar

            try:
                speed = int(speed_input)
                if 0 <= speed <= 255:
                    return speed
                else:
                    print("Error: Velocidad debe estar entre 0 y 255")
            except ValueError:
                print("Error: Debes ingresar un número")

    def _input_duration(self):
        """Solicitar duración en segundos"""
        while True:
            duration_input = input("Duración (segundos) [Enter=1.0]: ").strip()

            if not duration_input:
                return 1.0  # Valor por defecto

            if duration_input.lower() == 'q':
                return None  # Cancelar

            try:
                duration = float(duration_input)
                if duration > 0:
                    return duration
                else:
                    print("Error: Duración debe ser positiva")
            except ValueError:
                print("Error: Debes ingresar un número")

    def record_with_sensors(self):
        """
        Grabar secuencia enriquecida con datos de sensor fusion
        Registra tanto los comandos de motor como la trayectoria real del robot
        """
        if not self.enable_sensors or self.sensor_fusion is None:
            print("❌ Error: Sensor fusion no está habilitado")
            print("   Inicializa MovementRecorder con enable_sensors=True")
            return False

        print("\n" + "=" * 60)
        print("GRABACIÓN ENRIQUECIDA CON SENSOR FUSION")
        print("=" * 60)

        # Solicitar nombre de secuencia
        sequence_name = input("Nombre de la secuencia: ").strip()
        if not sequence_name:
            print("Error: Debes proporcionar un nombre")
            return False

        # Inicializar nueva secuencia con estructura enriquecida
        self.current_sequence = {
            'name': sequence_name,
            'created_at': datetime.now().isoformat(),
            'recording_mode': 'sensor_fusion_enabled',
            'steps': [],
            'total_duration': 0
        }

        # Inicializar data logger para esta sesión
        session_name = f"recording_{sequence_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.data_logger = DataLogger(session_name)

        # Iniciar sensor fusion
        print("\nIniciando sensor fusion...")
        self.sensor_fusion.start(enable_logging=True, data_logger=self.data_logger)
        time.sleep(0.5)  # Esperar estabilización

        print(f"\nGrabando secuencia enriquecida: '{sequence_name}'")
        print("Los sensores están capturando datos en tiempo real")
        print("Añade pasos uno por uno. Presiona Enter sin valor para terminar.\n")

        step_number = 1
        recording_start_time = time.time()

        try:
            while True:
                print(f"\n--- Paso {step_number} ---")

                # Seleccionar acción
                action = self._select_action()
                if action is None:
                    break

                # Solicitar velocidad
                speed = self._input_speed(action)
                if speed is None:
                    break

                # Solicitar duración
                duration = self._input_duration()
                if duration is None:
                    break

                # Capturar posición inicial
                step_start_time = time.time() - recording_start_time
                start_pos = self.sensor_fusion.get_position()

                # Ejecutar paso
                print(f"Ejecutando y registrando: {action}...")
                self._execute_step({'action': action, 'speed': speed, 'duration': duration})

                # Capturar posición final
                step_end_time = time.time() - recording_start_time
                end_pos = self.sensor_fusion.get_position()

                # Obtener resumen de sensores para este paso
                sensor_summary = self._get_sensor_summary_for_step(step_start_time, step_end_time)

                # Crear paso enriquecido
                step = {
                    'action': action,
                    'speed': speed,
                    'duration': duration,
                    'start_time': step_start_time,
                    'end_time': step_end_time,
                    'sensor_fusion': {
                        'start_position': start_pos,
                        'end_position': end_pos,
                        'sensor_summary': sensor_summary
                    }
                }

                # Añadir paso a la secuencia
                self.current_sequence['steps'].append(step)
                self.current_sequence['total_duration'] += duration

                # Mostrar info del paso
                print(f"✓ Paso {step_number} registrado:")
                print(f"  Posición: ({start_pos['x']:.1f}, {start_pos['y']:.1f}) → "
                      f"({end_pos['x']:.1f}, {end_pos['y']:.1f})")
                print(f"  Heading: {start_pos['heading']:.1f}° → {end_pos['heading']:.1f}°")

                step_number += 1

        except KeyboardInterrupt:
            print("\n\nGrabación interrumpida por el usuario")

        finally:
            # Detener sensor fusion y guardar sesión
            print("\nFinalizando grabación...")
            self.sensor_fusion.stop()

            if self.data_logger:
                session_file = self.data_logger.save_session()
                print(f"✓ Sesión de sensor fusion guardada: {session_file}")

                # Añadir referencia a la sesión en la secuencia
                self.current_sequence['sensor_fusion_session'] = session_file

        # Resumen final
        if self.current_sequence['steps']:
            print(f"\n" + "=" * 60)
            print(f"Secuencia enriquecida '{sequence_name}' completada")
            print(f"Total de pasos: {len(self.current_sequence['steps'])}")
            print(f"Duración total: {self.current_sequence['total_duration']:.1f}s")

            # Mostrar trayectoria total
            first_step = self.current_sequence['steps'][0]
            last_step = self.current_sequence['steps'][-1]
            start_pos = first_step['sensor_fusion']['start_position']
            end_pos = last_step['sensor_fusion']['end_position']

            dx = end_pos['x'] - start_pos['x']
            dy = end_pos['y'] - start_pos['y']
            distance = (dx**2 + dy**2)**0.5

            print(f"Distancia recorrida: {distance:.1f} cm")
            print(f"Posición final: ({end_pos['x']:.1f}, {end_pos['y']:.1f})")
            print(f"Heading final: {end_pos['heading']:.1f}°")
            print("=" * 60)
            return True
        else:
            print("No se grabó ningún paso")
            return False

    def _get_sensor_summary_for_step(self, start_time, end_time):
        """Obtener resumen de lecturas de sensores para un paso"""
        if not self.data_logger:
            return {}

        # Obtener lecturas del data logger entre start_time y end_time
        # Por ahora, obtenemos el estado actual como resumen
        status = self.sensor_fusion.get_system_status()

        return {
            'navigation': status.get('navigation', {}),
            'sensors_ok': status.get('system_health', {}).get('ultrasonics_ok', False),
            'obstacles_detected': len(status.get('navigation', {}).get('potential_blocks', []))
        }

    # ========================================================================
    # REPRODUCCIÓN DE SECUENCIAS
    # ========================================================================

    def replay_sequence(self):
        """Reproducir la secuencia actual"""
        if not self.current_sequence['steps']:
            print("Error: No hay secuencia cargada")
            return False

        print("\n" + "=" * 60)
        print(f"REPRODUCIENDO: {self.current_sequence['name']}")
        print(f"Total de pasos: {len(self.current_sequence['steps'])}")
        print(f"Duración total: {self.current_sequence['total_duration']:.1f}s")
        print("=" * 60)
        print("\nPresiona Ctrl+C para detener en cualquier momento\n")

        input("Presiona Enter para comenzar...")

        try:
            for i, step in enumerate(self.current_sequence['steps'], 1):
                print(f"\n[Paso {i}/{len(self.current_sequence['steps'])}] "
                      f"{step['action']} @ velocidad {step['speed']} "
                      f"por {step['duration']:.1f}s")

                self._execute_step(step)

            # Detener al final
            self.motors.stop()

            print("\n" + "=" * 60)
            print("✓ Secuencia completada exitosamente")
            print("=" * 60)
            return True

        except KeyboardInterrupt:
            print("\n\n⚠ Reproducción interrumpida por el usuario")
            self.motors.stop()
            return False

        except Exception as e:
            print(f"\n\n❌ Error durante reproducción: {e}")
            self.motors.stop()
            return False

    def play_with_sensors(self):
        """
        Reproducir secuencia con sensor fusion activo
        Captura la trayectoria REAL durante la ejecución para comparación posterior
        """
        if not self.current_sequence['steps']:
            print("Error: No hay secuencia cargada")
            return False

        if not self.enable_sensors or self.sensor_fusion is None:
            print("❌ Error: Sensor fusion no está habilitado")
            print("   Inicializa MovementRecorder con enable_sensors=True")
            return False

        print("\n" + "=" * 60)
        print(f"REPRODUCCIÓN CON SENSORES: {self.current_sequence['name']}")
        print("=" * 60)
        print(f"Total de pasos: {len(self.current_sequence['steps'])}")
        print(f"Duración total: {self.current_sequence['total_duration']:.1f}s")
        print("\nSensor fusion capturará la trayectoria real durante la ejecución")
        print("=" * 60)
        print("\nPresiona Ctrl+C para detener en cualquier momento\n")

        input("Presiona Enter para comenzar...")

        # Inicializar data logger para esta sesión de playback
        session_name = f"playback_{self.current_sequence['name']}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.data_logger = DataLogger(session_name)

        # Iniciar sensor fusion
        print("\nIniciando sensor fusion...")
        self.sensor_fusion.start(enable_logging=True, data_logger=self.data_logger)
        time.sleep(0.5)  # Esperar estabilización

        playback_start_time = time.time()
        playback_data = {
            'sequence_name': self.current_sequence['name'],
            'playback_timestamp': datetime.now().isoformat(),
            'original_recording_mode': self.current_sequence.get('recording_mode', 'simple'),
            'steps_executed': []
        }

        try:
            for i, step in enumerate(self.current_sequence['steps'], 1):
                print(f"\n[Paso {i}/{len(self.current_sequence['steps'])}] "
                      f"{step['action']} @ velocidad {step['speed']} "
                      f"por {step['duration']:.1f}s")

                # Capturar posición antes del paso
                step_start_time = time.time() - playback_start_time
                start_pos = self.sensor_fusion.get_position()
                start_status = self.sensor_fusion.get_system_status()

                # Ejecutar paso
                self._execute_step(step)

                # Capturar posición después del paso
                step_end_time = time.time() - playback_start_time
                end_pos = self.sensor_fusion.get_position()
                end_status = self.sensor_fusion.get_system_status()

                # Calcular distancia real recorrida
                dx = end_pos['x'] - start_pos['x']
                dy = end_pos['y'] - start_pos['y']
                distance = (dx**2 + dy**2)**0.5

                # Registrar datos del paso ejecutado
                step_data = {
                    'step_number': i,
                    'action': step['action'],
                    'speed': step['speed'],
                    'duration': step['duration'],
                    'start_time': step_start_time,
                    'end_time': step_end_time,
                    'actual_execution': {
                        'start_position': start_pos,
                        'end_position': end_pos,
                        'distance_traveled': distance,
                        'start_navigation': start_status.get('navigation', {}),
                        'end_navigation': end_status.get('navigation', {})
                    }
                }

                # Si la secuencia original tenía datos de sensor fusion, comparar
                if 'sensor_fusion' in step:
                    expected_pos = step['sensor_fusion']['end_position']
                    pos_error = {
                        'x': end_pos['x'] - expected_pos['x'],
                        'y': end_pos['y'] - expected_pos['y'],
                        'heading': end_pos['heading'] - expected_pos['heading']
                    }
                    step_data['position_error'] = pos_error

                    print(f"  Posición esperada: ({expected_pos['x']:.1f}, {expected_pos['y']:.1f})")
                    print(f"  Posición real:     ({end_pos['x']:.1f}, {end_pos['y']:.1f})")
                    print(f"  Error: Δx={pos_error['x']:.1f}cm, Δy={pos_error['y']:.1f}cm, "
                          f"Δheading={pos_error['heading']:.1f}°")
                else:
                    print(f"  Posición: ({start_pos['x']:.1f}, {start_pos['y']:.1f}) → "
                          f"({end_pos['x']:.1f}, {end_pos['y']:.1f})")
                    print(f"  Distancia: {distance:.1f} cm")

                playback_data['steps_executed'].append(step_data)

        except KeyboardInterrupt:
            print("\n\n⚠ Reproducción interrumpida por el usuario")
            self.motors.stop()
            playback_data['interrupted'] = True

        except Exception as e:
            print(f"\n\n❌ Error durante reproducción: {e}")
            self.motors.stop()
            playback_data['error'] = str(e)

        finally:
            # Detener motores y sensor fusion
            self.motors.stop()
            print("\n\nFinalizando sensor fusion...")
            self.sensor_fusion.stop()

            # Guardar sesión de sensor fusion
            if self.data_logger:
                session_file = self.data_logger.save_session()
                print(f"✓ Sesión de sensor fusion guardada: {session_file}")
                playback_data['sensor_fusion_session'] = session_file

            # Guardar datos de playback
            playback_file = os.path.join(
                DATA_PATHS['sessions'],
                f"playback_{self.current_sequence['name']}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )

            try:
                with open(playback_file, 'w') as f:
                    json.dump(playback_data, f, indent=2)
                print(f"✓ Datos de playback guardados: {playback_file}")
            except Exception as e:
                print(f"⚠ Error guardando datos de playback: {e}")

        # Resumen final
        if playback_data['steps_executed']:
            print(f"\n" + "=" * 60)
            print("RESUMEN DE REPRODUCCIÓN")
            print("=" * 60)

            first_step = playback_data['steps_executed'][0]
            last_step = playback_data['steps_executed'][-1]

            start_pos = first_step['actual_execution']['start_position']
            end_pos = last_step['actual_execution']['end_position']

            total_distance = sum(s['actual_execution']['distance_traveled']
                               for s in playback_data['steps_executed'])

            print(f"Pasos ejecutados: {len(playback_data['steps_executed'])}")
            print(f"Posición inicial: ({start_pos['x']:.1f}, {start_pos['y']:.1f}, {start_pos['heading']:.1f}°)")
            print(f"Posición final:   ({end_pos['x']:.1f}, {end_pos['y']:.1f}, {end_pos['heading']:.1f}°)")
            print(f"Distancia total recorrida: {total_distance:.1f} cm")

            # Si había datos esperados, mostrar error acumulado
            if 'position_error' in playback_data['steps_executed'][-1]:
                final_error = last_step['position_error']
                error_magnitude = (final_error['x']**2 + final_error['y']**2)**0.5
                print(f"\nError acumulado:")
                print(f"  Posición: {error_magnitude:.1f} cm")
                print(f"  Heading:  {abs(final_error['heading']):.1f}°")

            print("=" * 60)
            print("\n✓ Secuencia completada exitosamente")
            print("  Usa trajectory_analyzer.py para análisis detallado")
            return True

        return False

    def _execute_step(self, step):
        """Ejecutar un solo paso de movimiento"""
        action = step['action']
        speed = step['speed']
        duration = step['duration']

        # Ejecutar acción correspondiente
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

        # Esperar duración especificada
        time.sleep(duration)

        # Detener motores después de cada paso
        self.motors.stop()
        time.sleep(0.1)  # Pequeña pausa entre pasos

    # ========================================================================
    # GESTIÓN DE ARCHIVOS
    # ========================================================================

    def save_sequence(self, filename=None):
        """Guardar secuencia actual a archivo JSON"""
        if not self.current_sequence['steps']:
            print("Error: No hay secuencia para guardar")
            return None

        if filename is None:
            # Usar nombre de la secuencia
            safe_name = self.current_sequence['name'].replace(' ', '_').lower()
            filename = f"{safe_name}.json"

        # Asegurar extensión .json
        if not filename.endswith('.json'):
            filename += '.json'

        filepath = os.path.join(DATA_PATHS['movements'], filename)

        try:
            with open(filepath, 'w') as f:
                json.dump(self.current_sequence, f, indent=2)

            print(f"\n✓ Secuencia guardada: {filepath}")
            return filepath

        except Exception as e:
            print(f"❌ Error guardando secuencia: {e}")
            return None

    def load_sequence(self, filename):
        """Cargar secuencia desde archivo JSON"""
        # Añadir extensión si falta
        if not filename.endswith('.json'):
            filename += '.json'

        filepath = os.path.join(DATA_PATHS['movements'], filename)

        if not os.path.exists(filepath):
            print(f"Error: Archivo no encontrado: {filepath}")
            return False

        try:
            with open(filepath, 'r') as f:
                self.current_sequence = json.load(f)

            print(f"\n✓ Secuencia cargada: {self.current_sequence['name']}")
            print(f"  Pasos: {len(self.current_sequence['steps'])}")
            print(f"  Duración: {self.current_sequence['total_duration']:.1f}s")
            return True

        except Exception as e:
            print(f"❌ Error cargando secuencia: {e}")
            return False

    def list_sequences(self):
        """Listar secuencias guardadas"""
        movements_dir = DATA_PATHS['movements']

        if not os.path.exists(movements_dir):
            print("No hay secuencias guardadas")
            return []

        files = [f for f in os.listdir(movements_dir) if f.endswith('.json')]

        if not files:
            print("No hay secuencias guardadas")
            return []

        print(f"\nSecuencias disponibles ({len(files)}):")
        print("-" * 60)

        sequences = []

        for i, filename in enumerate(sorted(files), 1):
            filepath = os.path.join(movements_dir, filename)

            try:
                with open(filepath, 'r') as f:
                    data = json.load(f)

                sequences.append({
                    'filename': filename,
                    'name': data.get('name', 'Sin nombre'),
                    'steps': len(data.get('steps', [])),
                    'duration': data.get('total_duration', 0)
                })

                print(f"{i:2d}. {data.get('name', 'Sin nombre'):20} | "
                      f"{len(data.get('steps', [])):2d} pasos | "
                      f"{data.get('total_duration', 0):5.1f}s | "
                      f"{filename}")

            except Exception as e:
                print(f"{i:2d}. {filename:20} | ERROR: {e}")

        print("-" * 60)
        return sequences

    def delete_sequence(self, filename):
        """Eliminar secuencia guardada"""
        if not filename.endswith('.json'):
            filename += '.json'

        filepath = os.path.join(DATA_PATHS['movements'], filename)

        try:
            os.remove(filepath)
            print(f"✓ Secuencia eliminada: {filename}")
            return True
        except Exception as e:
            print(f"❌ Error eliminando secuencia: {e}")
            return False

    def print_sequence_details(self):
        """Mostrar detalles de la secuencia actual"""
        if not self.current_sequence['steps']:
            print("No hay secuencia cargada")
            return

        print("\n" + "=" * 60)
        print(f"SECUENCIA: {self.current_sequence['name']}")
        print("=" * 60)
        print(f"Creada: {self.current_sequence['created_at']}")
        print(f"Duración total: {self.current_sequence['total_duration']:.1f}s")
        print(f"\nPasos ({len(self.current_sequence['steps'])}):")
        print("-" * 60)

        for i, step in enumerate(self.current_sequence['steps'], 1):
            print(f"{i:2d}. {step['action']:12} | "
                  f"Velocidad: {step['speed']:3d} | "
                  f"Duración: {step['duration']:5.1f}s")

        print("=" * 60)

    # ========================================================================
    # LIMPIEZA
    # ========================================================================

    def cleanup(self):
        """Limpiar recursos"""
        self.motors.stop()
        self.motors.cleanup()

        # Limpiar sensor fusion si está activo
        if self.sensor_fusion is not None:
            try:
                self.sensor_fusion.stop()
                self.sensor_fusion.cleanup()
            except Exception as e:
                print(f"⚠ Error limpiando sensor fusion: {e}")

        print("Recursos del Movement Recorder liberados")

# ============================================================================
# MENÚ INTERACTIVO
# ============================================================================

def movement_recorder_menu():
    """Menú interactivo del grabador de movimientos"""
    print("\n" + "=" * 70)
    print("MOVEMENT RECORDER - Entrenamiento de Robot")
    print("=" * 70)
    print("\n¿Habilitar sensor fusion para grabaciones enriquecidas?")
    print("  [y] - Sí, usar sensores (recomendado para análisis)")
    print("  [n] - No, solo motores (más rápido, menos datos)")

    enable_sensors_input = input("\nOpción [y/N]: ").strip().lower()
    enable_sensors = enable_sensors_input == 'y'

    try:
        recorder = MovementRecorder(enable_sensors=enable_sensors)
    except Exception as e:
        print(f"❌ Error inicializando Movement Recorder: {e}")
        print("Verifica las conexiones de hardware")
        return

    # Mostrar modo activo REAL (después de posible fallback)
    mode = "CON SENSOR FUSION" if recorder.enable_sensors else "SOLO MOTORES"

    # Advertir si hubo fallback
    if enable_sensors and not recorder.enable_sensors:
        print(f"\n⚠️  ADVERTENCIA: Sensor fusion no se pudo inicializar")
        print(f"    Sistema operando en modo: {mode}")
    else:
        print(f"\n✓ Movement Recorder inicializado en modo: {mode}")

    try:
        while True:
            print("\n" + "=" * 70)
            print(f"MENÚ MOVEMENT RECORDER - Modo: {mode}")
            print("=" * 70)

            # Opciones básicas (siempre disponibles)
            print("\n--- GRABACIÓN Y REPRODUCCIÓN ---")
            print("1. 📝 Grabar nueva secuencia (simple)")
            print("2. ▶️  Reproducir secuencia actual (simple)")

            # Opciones avanzadas con sensores
            if recorder.enable_sensors:
                print("\n--- CON SENSOR FUSION ---")
                print("3. 🎯 Grabar con sensores (enriquecida)")
                print("4. 📊 Reproducir con sensores (análisis)")

            # Gestión de archivos
            print("\n--- GESTIÓN DE ARCHIVOS ---")
            print("5. 💾 Guardar secuencia")
            print("6. 📂 Cargar secuencia")
            print("7. 📋 Listar secuencias guardadas")
            print("8. 🗑️  Eliminar secuencia")
            print("9. ℹ️  Ver detalles de secuencia actual")

            # Análisis
            if recorder.enable_sensors:
                print("\n--- ANÁLISIS ---")
                print("a. 📈 Analizar última reproducción (trajectory_analyzer)")

            print("\n0. 🚪 Salir")
            print("=" * 70)

            choice = input("Elegir opción: ").strip().lower()

            if choice == '1':
                recorder.record_sequence()

            elif choice == '2':
                recorder.replay_sequence()

            elif choice == '3' and recorder.enable_sensors:
                recorder.record_with_sensors()

            elif choice == '4' and recorder.enable_sensors:
                recorder.play_with_sensors()

            elif choice == '5':
                recorder.save_sequence()

            elif choice == '6':
                sequences = recorder.list_sequences()
                if sequences:
                    try:
                        idx = int(input("\nElegir secuencia [número]: ")) - 1
                        if 0 <= idx < len(sequences):
                            recorder.load_sequence(sequences[idx]['filename'])
                    except ValueError:
                        print("Entrada inválida")

            elif choice == '7':
                recorder.list_sequences()

            elif choice == '8':
                sequences = recorder.list_sequences()
                if sequences:
                    try:
                        idx = int(input("\nElegir secuencia a eliminar [número]: ")) - 1
                        if 0 <= idx < len(sequences):
                            confirm = input(f"¿Eliminar '{sequences[idx]['name']}'? [y/N]: ")
                            if confirm.lower() == 'y':
                                recorder.delete_sequence(sequences[idx]['filename'])
                    except ValueError:
                        print("Entrada inválida")

            elif choice == '9':
                recorder.print_sequence_details()

            elif choice == 'a' and recorder.enable_sensors:
                # Importar y ejecutar trajectory analyzer
                try:
                    from trajectory_analyzer import interactive_analyzer
                    interactive_analyzer()
                except Exception as e:
                    print(f"❌ Error ejecutando trajectory analyzer: {e}")

            elif choice == '0':
                print("Saliendo...")
                break

            else:
                print("Opción inválida")

    except KeyboardInterrupt:
        print("\n\nMovement Recorder interrumpido")

    finally:
        recorder.cleanup()

# ============================================================================
# PUNTO DE ENTRADA
# ============================================================================

if __name__ == "__main__":
    movement_recorder_menu()
