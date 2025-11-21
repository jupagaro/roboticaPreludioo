#!/usr/bin/env python3
"""
adaptive_replay_example.py
Ejemplo de cómo usar Battery Compensation con Movement Recorder

Este script muestra cómo integrar compensación adaptativa de batería
durante la reproducción de secuencias
"""

import time
from datetime import datetime
from movement_recorder import MovementRecorder
from battery_compensation import BatteryCompensator
from utils.logging import DataLogger
from config import DATA_PATHS
import json
import os


def replay_with_battery_compensation(sequence_filename, heading_threshold=10.0):
    """
    Reproducir secuencia con corrección de heading Y compensación de batería (Nivel 2)

    Args:
        sequence_filename: Nombre del archivo de secuencia (ej: "1.json")
        heading_threshold: Umbral de error de heading para corrección (grados)
    """
    print("\n" + "=" * 70)
    print("REPRODUCCIÓN CON COMPENSACIÓN ADAPTATIVA (NIVEL 2)")
    print("=" * 70)
    print("✅ Corrección de heading automática")
    print("🔋 Compensación dinámica de batería")
    print("=" * 70)

    # Inicializar recorder con sensores
    try:
        recorder = MovementRecorder(enable_sensors=True)
    except Exception as e:
        print(f"❌ Error inicializando Movement Recorder: {e}")
        return

    # Cargar secuencia
    if not recorder.load_sequence(sequence_filename):
        print(f"❌ No se pudo cargar la secuencia '{sequence_filename}'")
        recorder.cleanup()
        return

    # Mostrar info de secuencia
    recorder.print_sequence_details()

    # Inicializar compensador de batería
    compensator = BatteryCompensator(enable_logging=True)

    # Configurar logging
    session_name = f"adaptive_{recorder.current_sequence['name']}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    data_logger = DataLogger(session_name)

    # Activar sensor fusion
    recorder.sensor_fusion.enable_logging = True
    recorder.sensor_fusion.logger = data_logger

    print("\nIniciando sensor fusion...")
    recorder.sensor_fusion.start_sensor_loop(update_rate=10)
    time.sleep(0.5)

    input("\nPresiona Enter para comenzar reproducción adaptativa...")

    playback_start_time = time.time()
    playback_data = {
        'sequence_name': recorder.current_sequence['name'],
        'playback_timestamp': datetime.now().isoformat(),
        'correction_mode': 'adaptive_level2',
        'heading_threshold': heading_threshold,
        'steps_executed': [],
        'heading_corrections': [],
        'speed_compensations': []
    }

    total_heading_corrections = 0

    try:
        for i, step in enumerate(recorder.current_sequence['steps'], 1):
            print(f"\n[Paso {i}/{len(recorder.current_sequence['steps'])}] "
                  f"{step['action']} @ velocidad {step['speed']} "
                  f"por {step['duration']:.1f}s")

            # Capturar posición antes del paso
            step_start_time = time.time() - playback_start_time
            start_pos = recorder.sensor_fusion.get_position()

            # COMPENSAR VELOCIDAD BASADO EN BATERÍA
            original_speed = step['speed']
            compensated_speed = compensator.compensate_speed(original_speed)

            if compensated_speed != original_speed:
                print(f"  🔋 Velocidad ajustada: {original_speed} → {compensated_speed} PWM")

            # Ejecutar paso con velocidad compensada
            compensated_step = step.copy()
            compensated_step['speed'] = compensated_speed
            recorder._execute_step(compensated_step)

            # Capturar posición después del paso
            step_end_time = time.time() - playback_start_time
            end_pos = recorder.sensor_fusion.get_position()

            print(f"  Posición: ({start_pos['x']:.1f}, {start_pos['y']:.1f}, {start_pos['heading']:.1f}°) → "
                  f"({end_pos['x']:.1f}, {end_pos['y']:.1f}, {end_pos['heading']:.1f}°)")

            # ANALIZAR DESEMPEÑO Y ACTUALIZAR COMPENSADOR
            movement_type = 'linear' if step['action'] in ['forward', 'backward'] else 'angular'

            performance_data = compensator.measure_performance(
                commanded_speed=original_speed,
                duration=step['duration'],
                start_pos=start_pos,
                end_pos=end_pos,
                movement_type=movement_type
            )

            if performance_data['adjustment_made']:
                playback_data['speed_compensations'].append({
                    'step': i,
                    'performance_ratio': performance_data['performance_ratio'],
                    'new_speed_factor': performance_data['current_speed_factor']
                })

            # CORRECCIÓN DE HEADING (si tenemos referencia)
            has_reference = 'sensor_fusion' in step
            heading_error = 0

            if has_reference:
                expected_heading = step['sensor_fusion']['end_position']['heading']
                heading_error = expected_heading - end_pos['heading']

                # Normalizar
                while heading_error > 180:
                    heading_error -= 360
                while heading_error < -180:
                    heading_error += 360

                print(f"  Heading esperado: {expected_heading:.1f}°")
                print(f"  Error de heading: {heading_error:.1f}°")

                if abs(heading_error) > heading_threshold:
                    print(f"  🔧 CORRIGIENDO heading ({heading_error:.1f}°)...")

                    correction_duration = abs(heading_error) / 60.0

                    # Aplicar compensación de batería también a la corrección
                    correction_speed = compensator.compensate_speed(100)

                    if heading_error > 0:
                        recorder.motors.spin_left(correction_speed)
                    else:
                        recorder.motors.spin_right(correction_speed)

                    time.sleep(correction_duration)
                    recorder.motors.stop()
                    time.sleep(0.1)

                    corrected_pos = recorder.sensor_fusion.get_position()
                    final_error = expected_heading - corrected_pos['heading']

                    while final_error > 180:
                        final_error -= 360
                    while final_error < -180:
                        final_error += 360

                    print(f"  ✅ Heading corregido: {corrected_pos['heading']:.1f}° "
                          f"(error residual: {final_error:.1f}°)")

                    total_heading_corrections += 1

                    playback_data['heading_corrections'].append({
                        'step': i,
                        'error_before': heading_error,
                        'error_after': final_error,
                        'compensated_speed_used': correction_speed
                    })

            # Registrar datos del paso
            step_data = {
                'step_number': i,
                'action': step['action'],
                'original_speed': original_speed,
                'compensated_speed': compensated_speed,
                'duration': step['duration'],
                'start_time': step_start_time,
                'end_time': step_end_time,
                'actual_execution': {
                    'start_position': start_pos,
                    'end_position': end_pos
                },
                'performance_ratio': performance_data['performance_ratio'],
                'heading_error': heading_error if has_reference else None
            }

            playback_data['steps_executed'].append(step_data)

    except KeyboardInterrupt:
        print("\n\n⚠ Reproducción interrumpida por el usuario")
        recorder.motors.stop()
        playback_data['interrupted'] = True

    except Exception as e:
        print(f"\n\n❌ Error durante reproducción: {e}")
        import traceback
        traceback.print_exc()
        recorder.motors.stop()
        playback_data['error'] = str(e)

    finally:
        # Detener todo
        recorder.motors.stop()
        print("\n\nFinalizando sensor fusion...")
        recorder.sensor_fusion.stop()

        # Guardar sesión
        session_file = data_logger.save_session()
        print(f"✓ Sesión de sensor fusion guardada: {session_file}")
        playback_data['sensor_fusion_session'] = session_file

        # Guardar datos de playback
        playback_file = os.path.join(
            DATA_PATHS['sessions'],
            f"adaptive_{recorder.current_sequence['name']}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )

        try:
            with open(playback_file, 'w') as f:
                json.dump(playback_data, f, indent=2)
            print(f"✓ Datos de playback guardados: {playback_file}")
        except Exception as e:
            print(f"⚠ Error guardando datos: {e}")

    # Resumen final
    if playback_data['steps_executed']:
        print(f"\n" + "=" * 70)
        print("RESUMEN DE REPRODUCCIÓN ADAPTATIVA")
        print("=" * 70)

        first_step = playback_data['steps_executed'][0]
        last_step = playback_data['steps_executed'][-1]

        start_pos = first_step['actual_execution']['start_position']
        end_pos = last_step['actual_execution']['end_position']

        print(f"Pasos ejecutados: {len(playback_data['steps_executed'])}")
        print(f"Correcciones de heading: {total_heading_corrections}")
        print(f"Ajustes de velocidad: {len(playback_data['speed_compensations'])}")
        print(f"Posición final: ({end_pos['x']:.1f}, {end_pos['y']:.1f}, {end_pos['heading']:.1f}°)")

        # Mostrar estadísticas de compensación
        print("\n")
        compensator.print_statistics()

        print("=" * 70)
        print("\n✅ Reproducción completada")

    # Cleanup
    recorder.cleanup()


def main():
    """Menú principal para reproducción adaptativa"""
    print("\n🤖 REPRODUCCIÓN ADAPTATIVA CON COMPENSACIÓN DE BATERÍA")
    print("=" * 70)
    print("\nEste modo combina:")
    print("  1. Corrección automática de heading (Nivel 1)")
    print("  2. Compensación dinámica de velocidad por batería (Nivel 2)")
    print("\nEl robot ajustará automáticamente su velocidad para compensar")
    print("la caída de voltaje de la batería durante la ejecución.")
    print("=" * 70)

    # Seleccionar secuencia
    sequence = input("\nNombre del archivo de secuencia [1.json]: ").strip()
    if not sequence:
        sequence = "1.json"

    # Umbral de heading
    threshold_input = input("Umbral de corrección de heading en grados [10]: ").strip()
    threshold = float(threshold_input) if threshold_input else 10.0

    # Ejecutar
    replay_with_battery_compensation(sequence, threshold)


if __name__ == "__main__":
    main()
