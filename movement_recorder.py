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

    def __init__(self):
        # Inicializar SOLO motores (sin sensores)
        print("Inicializando Movement Recorder (solo motores)...")
        self.motors = MotorController()

        if not self.motors.is_initialized:
            raise Exception("Error: No se pudieron inicializar los motores")

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
        print("Recursos del Movement Recorder liberados")

# ============================================================================
# MENÚ INTERACTIVO
# ============================================================================

def movement_recorder_menu():
    """Menú interactivo del grabador de movimientos"""
    print("\n" + "=" * 60)
    print("MOVEMENT RECORDER - Entrenamiento de Robot")
    print("=" * 60)

    try:
        recorder = MovementRecorder()
    except Exception as e:
        print(f"❌ Error inicializando Movement Recorder: {e}")
        print("Verifica las conexiones de los motores")
        return

    try:
        while True:
            print("\n" + "=" * 60)
            print("MENÚ MOVEMENT RECORDER")
            print("=" * 60)
            print("1. 📝 Grabar nueva secuencia")
            print("2. ▶️  Reproducir secuencia actual")
            print("3. 💾 Guardar secuencia")
            print("4. 📂 Cargar secuencia")
            print("5. 📋 Listar secuencias guardadas")
            print("6. 🗑️  Eliminar secuencia")
            print("7. ℹ️  Ver detalles de secuencia actual")
            print("0. 🚪 Salir")
            print("=" * 60)

            choice = input("Elegir opción [0-7]: ").strip()

            if choice == '1':
                recorder.record_sequence()

            elif choice == '2':
                recorder.replay_sequence()

            elif choice == '3':
                recorder.save_sequence()

            elif choice == '4':
                sequences = recorder.list_sequences()
                if sequences:
                    try:
                        idx = int(input("\nElegir secuencia [número]: ")) - 1
                        if 0 <= idx < len(sequences):
                            recorder.load_sequence(sequences[idx]['filename'])
                    except ValueError:
                        print("Entrada inválida")

            elif choice == '5':
                recorder.list_sequences()

            elif choice == '6':
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

            elif choice == '7':
                recorder.print_sequence_details()

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
