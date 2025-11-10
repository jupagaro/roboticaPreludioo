#!/usr/bin/env python3
"""
trajectory_analyzer.py
Herramienta de análisis de trayectorias para comparar ejecuciones grabadas vs reales
Analiza drift, precisión de sensores, y genera reportes de desempeño
"""

import json
import os
import math
from datetime import datetime
from config import DATA_PATHS

class TrajectoryAnalyzer:
    """Analizador de trayectorias del robot"""

    def __init__(self):
        """Inicializar analizador"""
        self.playback_data = None
        self.session_data = None
        self.analysis_results = {}

    def load_playback_data(self, playback_file):
        """
        Cargar datos de reproducción con sensores

        Args:
            playback_file: Nombre del archivo de playback (puede ser ruta completa o solo nombre)

        Returns:
            bool: True si se cargó exitosamente
        """
        # Si no es ruta completa, buscar en directorio de sesiones
        if not os.path.isabs(playback_file):
            playback_file = os.path.join(DATA_PATHS['sessions'], playback_file)

        if not os.path.exists(playback_file):
            print(f"❌ Error: Archivo no encontrado: {playback_file}")
            return False

        try:
            with open(playback_file, 'r') as f:
                self.playback_data = json.load(f)

            print(f"✓ Datos de playback cargados: {playback_file}")
            print(f"  Secuencia: {self.playback_data.get('sequence_name', 'N/A')}")
            print(f"  Pasos ejecutados: {len(self.playback_data.get('steps_executed', []))}")

            # Intentar cargar sesión de sensor fusion asociada
            if 'sensor_fusion_session' in self.playback_data:
                session_file = self.playback_data['sensor_fusion_session']
                if os.path.exists(session_file):
                    with open(session_file, 'r') as f:
                        self.session_data = json.load(f)
                    print(f"✓ Sesión de sensor fusion cargada")

            return True

        except Exception as e:
            print(f"❌ Error cargando archivo: {e}")
            return False

    def analyze_trajectory(self):
        """
        Analizar trayectoria completa

        Returns:
            dict: Resultados del análisis
        """
        if not self.playback_data:
            print("❌ Error: No hay datos de playback cargados")
            return None

        print("\n" + "=" * 70)
        print("ANÁLISIS DE TRAYECTORIA")
        print("=" * 70)

        self.analysis_results = {
            'sequence_name': self.playback_data.get('sequence_name', 'N/A'),
            'timestamp': datetime.now().isoformat(),
            'position_drift': self._analyze_position_drift(),
            'heading_drift': self._analyze_heading_drift(),
            'step_accuracy': self._analyze_step_accuracy(),
            'sensor_reliability': self._analyze_sensor_reliability(),
            'distance_analysis': self._analyze_distance(),
        }

        return self.analysis_results

    def _analyze_position_drift(self):
        """Analizar deriva de posición acumulada"""
        steps = self.playback_data.get('steps_executed', [])

        if not steps:
            return {'error': 'No steps data'}

        # Calcular deriva paso a paso
        drift_data = []
        cumulative_error = {'x': 0, 'y': 0}

        for step in steps:
            if 'position_error' in step:
                error = step['position_error']
                cumulative_error['x'] += error['x']
                cumulative_error['y'] += error['y']

                magnitude = math.sqrt(error['x']**2 + error['y']**2)
                cumulative_magnitude = math.sqrt(cumulative_error['x']**2 + cumulative_error['y']**2)

                drift_data.append({
                    'step': step['step_number'],
                    'error_x': error['x'],
                    'error_y': error['y'],
                    'magnitude': magnitude,
                    'cumulative_magnitude': cumulative_magnitude
                })

        if not drift_data:
            # Sin datos esperados, solo analizar trayectoria real
            first_step = steps[0]
            last_step = steps[-1]

            start_pos = first_step['actual_execution']['start_position']
            end_pos = last_step['actual_execution']['end_position']

            return {
                'has_reference': False,
                'start_position': start_pos,
                'end_position': end_pos,
                'total_displacement': {
                    'x': end_pos['x'] - start_pos['x'],
                    'y': end_pos['y'] - start_pos['y']
                }
            }

        # Con datos de referencia
        final_drift = drift_data[-1]

        return {
            'has_reference': True,
            'drift_per_step': drift_data,
            'final_drift': {
                'x': final_drift['error_x'],
                'y': final_drift['error_y'],
                'magnitude': final_drift['cumulative_magnitude']
            },
            'max_step_error': max(d['magnitude'] for d in drift_data),
            'avg_step_error': sum(d['magnitude'] for d in drift_data) / len(drift_data),
        }

    def _analyze_heading_drift(self):
        """Analizar deriva de orientación"""
        steps = self.playback_data.get('steps_executed', [])

        if not steps:
            return {'error': 'No steps data'}

        heading_data = []
        cumulative_heading_error = 0

        for step in steps:
            actual = step['actual_execution']
            start_heading = actual['start_position']['heading']
            end_heading = actual['end_position']['heading']
            heading_change = end_heading - start_heading

            # Normalizar a -180 a 180
            heading_change = ((heading_change + 180) % 360) - 180

            if 'position_error' in step:
                heading_error = step['position_error']['heading']
                cumulative_heading_error += heading_error

                heading_data.append({
                    'step': step['step_number'],
                    'heading_change': heading_change,
                    'heading_error': heading_error,
                    'cumulative_error': cumulative_heading_error
                })

        if not heading_data:
            # Sin referencia, solo reportar cambios de heading
            first_step = steps[0]
            last_step = steps[-1]

            start_heading = first_step['actual_execution']['start_position']['heading']
            end_heading = last_step['actual_execution']['end_position']['heading']
            total_change = end_heading - start_heading

            return {
                'has_reference': False,
                'start_heading': start_heading,
                'end_heading': end_heading,
                'total_change': total_change
            }

        return {
            'has_reference': True,
            'heading_per_step': heading_data,
            'final_heading_error': cumulative_heading_error,
            'max_step_error': max(abs(d['heading_error']) for d in heading_data),
            'avg_step_error': sum(abs(d['heading_error']) for d in heading_data) / len(heading_data),
        }

    def _analyze_step_accuracy(self):
        """Analizar precisión de cada tipo de movimiento"""
        steps = self.playback_data.get('steps_executed', [])

        if not steps:
            return {'error': 'No steps data'}

        # Agrupar por tipo de acción
        action_stats = {}

        for step in steps:
            action = step['action']
            actual = step['actual_execution']
            distance = actual['distance_traveled']

            if action not in action_stats:
                action_stats[action] = {
                    'count': 0,
                    'distances': [],
                    'errors': []
                }

            action_stats[action]['count'] += 1
            action_stats[action]['distances'].append(distance)

            if 'position_error' in step:
                error_mag = math.sqrt(step['position_error']['x']**2 +
                                    step['position_error']['y']**2)
                action_stats[action]['errors'].append(error_mag)

        # Calcular estadísticas por acción
        for action, stats in action_stats.items():
            if stats['distances']:
                stats['avg_distance'] = sum(stats['distances']) / len(stats['distances'])
                stats['max_distance'] = max(stats['distances'])
                stats['min_distance'] = min(stats['distances'])

            if stats['errors']:
                stats['avg_error'] = sum(stats['errors']) / len(stats['errors'])
                stats['max_error'] = max(stats['errors'])
            else:
                stats['avg_error'] = None
                stats['max_error'] = None

            # Limpiar listas brutas para resumen
            del stats['distances']
            del stats['errors']

        return action_stats

    def _analyze_sensor_reliability(self):
        """Analizar confiabilidad de los sensores durante la ejecución"""
        if not self.session_data:
            return {'error': 'No session data available'}

        sensor_readings = self.session_data.get('sensor_readings', [])

        if not sensor_readings:
            return {'error': 'No sensor readings in session'}

        # Contar lecturas válidas e inválidas por sensor
        sensor_stats = {}
        sensor_names = ['frontal_izq', 'frontal_der', 'trasero_izq', 'trasero_der']

        for sensor_name in sensor_names:
            sensor_stats[sensor_name] = {
                'total_readings': 0,
                'valid_readings': 0,
                'blocked_readings': 0,
                'min_distance': float('inf'),
                'max_distance': 0,
                'avg_distance': 0
            }

        # Analizar cada lectura
        for reading in sensor_readings:
            readings_dict = reading.get('readings', {})

            for sensor_name, distance in readings_dict.items():
                if sensor_name in sensor_stats:
                    stats = sensor_stats[sensor_name]
                    stats['total_readings'] += 1

                    if distance > 0:
                        stats['valid_readings'] += 1
                        stats['min_distance'] = min(stats['min_distance'], distance)
                        stats['max_distance'] = max(stats['max_distance'], distance)
                        stats['avg_distance'] += distance
                    else:
                        stats['blocked_readings'] += 1

        # Calcular promedios
        for sensor_name, stats in sensor_stats.items():
            if stats['valid_readings'] > 0:
                stats['avg_distance'] = stats['avg_distance'] / stats['valid_readings']
                stats['success_rate'] = stats['valid_readings'] / stats['total_readings']
            else:
                stats['avg_distance'] = 0
                stats['success_rate'] = 0
                stats['min_distance'] = 0

        return sensor_stats

    def _analyze_distance(self):
        """Analizar distancias recorridas"""
        steps = self.playback_data.get('steps_executed', [])

        if not steps:
            return {'error': 'No steps data'}

        total_distance = sum(s['actual_execution']['distance_traveled']
                           for s in steps)

        first_step = steps[0]
        last_step = steps[-1]

        start_pos = first_step['actual_execution']['start_position']
        end_pos = last_step['actual_execution']['end_position']

        # Distancia directa inicio-fin
        straight_line_distance = math.sqrt(
            (end_pos['x'] - start_pos['x'])**2 +
            (end_pos['y'] - start_pos['y'])**2
        )

        return {
            'total_distance_traveled': total_distance,
            'straight_line_distance': straight_line_distance,
            'path_efficiency': straight_line_distance / total_distance if total_distance > 0 else 0,
            'start_position': start_pos,
            'end_position': end_pos
        }

    def print_report(self):
        """Imprimir reporte de análisis legible"""
        if not self.analysis_results:
            print("❌ Error: No hay resultados de análisis. Ejecuta analyze_trajectory() primero")
            return

        print("\n" + "=" * 70)
        print(f"REPORTE DE ANÁLISIS - {self.analysis_results['sequence_name']}")
        print("=" * 70)

        # 1. Análisis de distancia
        print("\n📏 ANÁLISIS DE DISTANCIA")
        print("-" * 70)
        dist = self.analysis_results['distance_analysis']
        if 'error' not in dist:
            print(f"Distancia total recorrida:  {dist['total_distance_traveled']:.1f} cm")
            print(f"Distancia en línea recta:   {dist['straight_line_distance']:.1f} cm")
            print(f"Eficiencia del camino:      {dist['path_efficiency']*100:.1f}%")
            print(f"Posición inicial: ({dist['start_position']['x']:.1f}, "
                  f"{dist['start_position']['y']:.1f}, {dist['start_position']['heading']:.1f}°)")
            print(f"Posición final:   ({dist['end_position']['x']:.1f}, "
                  f"{dist['end_position']['y']:.1f}, {dist['end_position']['heading']:.1f}°)")

        # 2. Deriva de posición
        print("\n📍 DERIVA DE POSICIÓN")
        print("-" * 70)
        pos_drift = self.analysis_results['position_drift']

        if pos_drift.get('has_reference'):
            print("✓ Datos de referencia disponibles (comparación con grabación original)")
            final = pos_drift['final_drift']
            print(f"Deriva final acumulada: {final['magnitude']:.1f} cm")
            print(f"  Δx = {final['x']:.1f} cm")
            print(f"  Δy = {final['y']:.1f} cm")
            print(f"Error máximo por paso:  {pos_drift['max_step_error']:.1f} cm")
            print(f"Error promedio por paso: {pos_drift['avg_step_error']:.1f} cm")

            # Advertencias
            if final['magnitude'] > 50:
                print(f"\n⚠ ADVERTENCIA: Deriva significativa detectada ({final['magnitude']:.1f} cm)")
                print("  Considera calibrar odometría o verificar superficies de trabajo")
        else:
            print("ℹ No hay datos de referencia (movimiento simple sin sensor fusion original)")
            print(f"Desplazamiento total: Δx={pos_drift['total_displacement']['x']:.1f} cm, "
                  f"Δy={pos_drift['total_displacement']['y']:.1f} cm")

        # 3. Deriva de orientación
        print("\n🧭 DERIVA DE ORIENTACIÓN")
        print("-" * 70)
        heading_drift = self.analysis_results['heading_drift']

        if heading_drift.get('has_reference'):
            print("✓ Datos de referencia disponibles")
            print(f"Error de heading final: {heading_drift['final_heading_error']:.1f}°")
            print(f"Error máximo por paso:  {heading_drift['max_step_error']:.1f}°")
            print(f"Error promedio por paso: {heading_drift['avg_step_error']:.1f}°")

            if abs(heading_drift['final_heading_error']) > 15:
                print(f"\n⚠ ADVERTENCIA: Deriva de orientación significativa "
                      f"({heading_drift['final_heading_error']:.1f}°)")
                print("  Verifica calibración del giroscopio y diferencial de motores")
        else:
            print("ℹ No hay datos de referencia")
            print(f"Cambio total de heading: {heading_drift['total_change']:.1f}°")

        # 4. Precisión por tipo de movimiento
        print("\n🎯 PRECISIÓN POR TIPO DE MOVIMIENTO")
        print("-" * 70)
        step_accuracy = self.analysis_results['step_accuracy']

        for action, stats in step_accuracy.items():
            print(f"\n{action.upper()}:")
            print(f"  Ejecutado {stats['count']} veces")
            print(f"  Distancia promedio: {stats['avg_distance']:.1f} cm "
                  f"(rango: {stats['min_distance']:.1f} - {stats['max_distance']:.1f} cm)")

            if stats['avg_error'] is not None:
                print(f"  Error promedio: {stats['avg_error']:.1f} cm")
                print(f"  Error máximo:   {stats['max_error']:.1f} cm")

        # 5. Confiabilidad de sensores
        print("\n🔍 CONFIABILIDAD DE SENSORES")
        print("-" * 70)
        sensor_reliability = self.analysis_results['sensor_reliability']

        if 'error' in sensor_reliability:
            print(f"ℹ {sensor_reliability['error']}")
        else:
            for sensor_name, stats in sensor_reliability.items():
                success_rate = stats['success_rate'] * 100

                # Clasificar sensores
                if 'frontal' in sensor_name:
                    sensor_type = "🔴 FRONTAL"
                else:
                    sensor_type = "🔵 TRASERO"

                print(f"\n{sensor_type} - {sensor_name}:")
                print(f"  Lecturas totales: {stats['total_readings']}")
                print(f"  Lecturas válidas: {stats['valid_readings']} ({success_rate:.1f}%)")
                print(f"  Lecturas bloqueadas/error: {stats['blocked_readings']}")

                if stats['valid_readings'] > 0:
                    print(f"  Rango de distancias: {stats['min_distance']:.1f} - "
                          f"{stats['max_distance']:.1f} cm (promedio: {stats['avg_distance']:.1f} cm)")

                # Advertencias
                if success_rate < 80:
                    print(f"  ⚠ Tasa de éxito baja - posible bloqueo u obstrucción")

        # Resumen final y recomendaciones
        print("\n" + "=" * 70)
        print("💡 RECOMENDACIONES")
        print("=" * 70)

        self._print_recommendations()

        print("\n" + "=" * 70)

    def _print_recommendations(self):
        """Generar recomendaciones basadas en el análisis"""
        recommendations = []

        # Análisis de deriva
        pos_drift = self.analysis_results['position_drift']
        if pos_drift.get('has_reference'):
            final_drift = pos_drift['final_drift']['magnitude']

            if final_drift > 50:
                recommendations.append(
                    "🔧 Alta deriva de posición detectada. Considera:\n"
                    "   - Verificar calibración de speed_factor en config.py\n"
                    "   - Revisar que las ruedas no resbalen en la superficie\n"
                    "   - Calibrar odometría con movimientos en línea recta conocidos"
                )

        # Análisis de heading
        heading_drift = self.analysis_results['heading_drift']
        if heading_drift.get('has_reference'):
            heading_error = abs(heading_drift['final_heading_error'])

            if heading_error > 15:
                recommendations.append(
                    "🧭 Deriva de orientación significativa. Considera:\n"
                    "   - Recalibrar giroscopio (calibration.py)\n"
                    "   - Verificar que el diferencial de motores esté balanceado\n"
                    "   - Aumentar peso del giroscopio en sensor fusion (gyro_alpha)"
                )

        # Análisis de sensores
        sensor_reliability = self.analysis_results['sensor_reliability']
        if 'error' not in sensor_reliability:
            blocked_sensors = []

            for sensor_name, stats in sensor_reliability.items():
                if stats['success_rate'] < 0.8:
                    blocked_sensors.append(sensor_name)

            if blocked_sensors:
                recommendations.append(
                    f"🔍 Sensores con baja confiabilidad: {', '.join(blocked_sensors)}\n"
                    "   - Verifica que no haya obstrucciones físicas\n"
                    "   - Si llevas objetos, considera usar solo sensores traseros\n"
                    "   - Revisa conexiones y cableado"
                )

        # Si no hay recomendaciones críticas
        if not recommendations:
            print("✅ El desempeño general es bueno. El robot está bien calibrado.")
        else:
            for i, rec in enumerate(recommendations, 1):
                print(f"\n{i}. {rec}")

    def save_report(self, output_file=None):
        """Guardar reporte de análisis en JSON"""
        if not self.analysis_results:
            print("❌ Error: No hay resultados para guardar")
            return None

        if output_file is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            seq_name = self.analysis_results['sequence_name'].replace(' ', '_')
            output_file = f"analysis_{seq_name}_{timestamp}.json"

        # Guardar en directorio de sesiones
        output_path = os.path.join(DATA_PATHS['sessions'], output_file)

        try:
            with open(output_path, 'w') as f:
                json.dump(self.analysis_results, f, indent=2)

            print(f"\n✓ Reporte guardado: {output_path}")
            return output_path

        except Exception as e:
            print(f"❌ Error guardando reporte: {e}")
            return None


def list_playback_files():
    """Listar archivos de playback disponibles"""
    sessions_dir = DATA_PATHS['sessions']

    if not os.path.exists(sessions_dir):
        print("No hay sesiones disponibles")
        return []

    files = [f for f in os.listdir(sessions_dir) if f.startswith('playback_') and f.endswith('.json')]

    if not files:
        print("No hay archivos de playback disponibles")
        return []

    print(f"\nArchivos de playback disponibles ({len(files)}):")
    print("-" * 70)

    playback_files = []

    for i, filename in enumerate(sorted(files), 1):
        filepath = os.path.join(sessions_dir, filename)

        try:
            with open(filepath, 'r') as f:
                data = json.load(f)

            playback_files.append({
                'filename': filename,
                'sequence_name': data.get('sequence_name', 'N/A'),
                'timestamp': data.get('playback_timestamp', 'N/A'),
                'steps': len(data.get('steps_executed', []))
            })

            print(f"{i:2d}. {data.get('sequence_name', 'N/A'):25} | "
                  f"{len(data.get('steps_executed', [])):2d} pasos | "
                  f"{filename}")

        except Exception as e:
            print(f"{i:2d}. {filename:25} | ERROR: {e}")

    print("-" * 70)
    return playback_files


def interactive_analyzer():
    """Modo interactivo del analizador"""
    print("\n" + "=" * 70)
    print("TRAJECTORY ANALYZER - Análisis de Trayectorias")
    print("=" * 70)

    analyzer = TrajectoryAnalyzer()

    # Listar archivos disponibles
    playback_files = list_playback_files()

    if not playback_files:
        print("\n❌ No hay archivos de playback para analizar")
        print("   Primero ejecuta una secuencia con play_with_sensors() en movement_recorder.py")
        return

    # Seleccionar archivo
    try:
        choice = int(input("\nElegir archivo a analizar [número]: "))

        if 1 <= choice <= len(playback_files):
            selected_file = playback_files[choice - 1]['filename']

            # Cargar datos
            if analyzer.load_playback_data(selected_file):
                # Ejecutar análisis
                print("\nEjecutando análisis...")
                analyzer.analyze_trajectory()

                # Mostrar reporte
                analyzer.print_report()

                # Opción de guardar
                save_option = input("\n¿Guardar reporte en JSON? [y/N]: ")
                if save_option.lower() == 'y':
                    analyzer.save_report()

        else:
            print("Opción inválida")

    except ValueError:
        print("Entrada inválida")
    except KeyboardInterrupt:
        print("\n\nAnálisis cancelado")


if __name__ == "__main__":
    interactive_analyzer()
