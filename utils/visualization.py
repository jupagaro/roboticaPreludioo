#!/usr/bin/env python3
"""
utils/visualization.py
Visualización simple de datos del robot
ASCII art y consola para análisis básico de sesiones
"""

import json
import time
from math import sqrt, floor, ceil
from datetime import datetime
from config import FIELD_DIMENSIONS, COMPETITION_ZONES

class SimpleVisualizer:
    """Visualizador simple usando ASCII para análisis de sesiones"""
    
    def __init__(self):
        self.field_width = FIELD_DIMENSIONS['width']
        self.field_height = FIELD_DIMENSIONS['height']
    
    def print_session_summary(self, session_data):
        """Mostrar resumen de una sesión"""
        info = session_data.get('session_info', {})
        stats = session_data.get('statistics', {})
        
        print("\n" + "="*60)
        print(f"RESUMEN DE SESIÓN: {info.get('name', 'Sin nombre')}")
        print("="*60)
        
        # Información básica
        duration = info.get('duration', 0)
        print(f"Duración: {duration:.1f}s ({duration//60:.0f}m {duration%60:.0f}s)")
        print(f"Creada: {info.get('created_at', 'Desconocida')}")
        print(f"Entradas de log: {info.get('total_entries', 0)}")
        
        # Estadísticas de trayectoria
        traj_stats = stats.get('trajectory_stats', {})
        if traj_stats:
            distance = traj_stats.get('distance_traveled', 0)
            positions = traj_stats.get('total_positions', 0)
            x_range = traj_stats.get('x_range', [0, 0])
            y_range = traj_stats.get('y_range', [0, 0])
            
            print(f"\nTrayectoria:")
            print(f"  Distancia recorrida: {distance:.1f} cm")
            print(f"  Posiciones registradas: {positions}")
            print(f"  Rango X: {x_range[0]:.1f} a {x_range[1]:.1f} cm")
            print(f"  Rango Y: {y_range[0]:.1f} a {y_range[1]:.1f} cm")
        
        # Estadísticas de obstáculos
        obstacles = stats.get('total_obstacles_detected', 0)
        if obstacles > 0:
            print(f"\nObstáculos detectados: {obstacles}")
        
        # Eventos de misión
        events = stats.get('total_mission_events', 0)
        if events > 0:
            print(f"Eventos de misión: {events}")
            
            # Mostrar eventos si están disponibles
            mission_events = session_data.get('mission_events', [])
            if mission_events:
                print("  Eventos principales:")
                for event in mission_events[-5:]:  # Últimos 5 eventos
                    time_str = datetime.fromtimestamp(event['timestamp']).strftime('%H:%M:%S')
                    print(f"    {time_str}: {event['description']}")
        
        # Estadísticas de sensores
        sensor_stats = stats.get('sensor_stats', {})
        if sensor_stats:
            total_readings = sensor_stats.get('total_sensor_readings', 0)
            readings_per_sensor = sensor_stats.get('readings_per_sensor', {})
            
            print(f"\nSensores:")
            print(f"  Lecturas totales: {total_readings}")
            if readings_per_sensor:
                print("  Por sensor:")
                for sensor, count in readings_per_sensor.items():
                    print(f"    {sensor}: {count}")
        
        print("="*60)
    
    def draw_trajectory_map(self, session_data, width=50, height=25):
        """Dibujar mapa ASCII de la trayectoria"""
        trajectory = session_data.get('trajectory', [])
        obstacles = session_data.get('obstacles_detected', [])
        
        if not trajectory:
            print("No hay datos de trayectoria para mostrar")
            return
        
        print(f"\nMAPA DE TRAYECTORIA ({width}x{height} chars)")
        print("R = Robot path, X = Obstacles, + = Start/End")
        print("-" * (width + 2))
        
        # Calcular límites de los datos
        x_coords = [p['x'] for p in trajectory]
        y_coords = [p['y'] for p in trajectory]
        
        if obstacles:
            x_coords.extend([o['x'] for o in obstacles])
            y_coords.extend([o['y'] for o in obstacles])
        
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        # Añadir margen
        x_range = max_x - min_x
        y_range = max_y - min_y
        margin_x = x_range * 0.1
        margin_y = y_range * 0.1
        
        min_x -= margin_x
        max_x += margin_x
        min_y -= margin_y
        max_y += margin_y
        
        # Crear grid
        grid = [['.' for _ in range(width)] for _ in range(height)]
        
        # Función para mapear coordenadas reales a grid
        def to_grid(x, y):
            if max_x == min_x or max_y == min_y:
                return width//2, height//2
            grid_x = int((x - min_x) / (max_x - min_x) * (width - 1))
            grid_y = int((y - min_y) / (max_y - min_y) * (height - 1))
            return max(0, min(grid_x, width-1)), max(0, min(grid_y, height-1))
        
        # Marcar obstáculos
        obstacle_positions = set()
        for obs in obstacles[::max(1, len(obstacles)//100)]:  # Máximo 100 obstáculos
            gx, gy = to_grid(obs['x'], obs['y'])
            obstacle_positions.add((gx, gy))
            grid[gy][gx] = 'X'
        
        # Marcar trayectoria (cada N puntos para no saturar)
        step = max(1, len(trajectory) // 200)  # Máximo 200 puntos
        for i, point in enumerate(trajectory[::step]):
            gx, gy = to_grid(point['x'], point['y'])
            
            # No sobreescribir obstáculos
            if (gx, gy) not in obstacle_positions:
                if i == 0:
                    grid[gy][gx] = '+'  # Inicio
                elif i == len(trajectory[::step]) - 1:
                    grid[gy][gx] = '+'  # Final
                else:
                    grid[gy][gx] = 'R'  # Trayectoria
        
        # Imprimir grid (Y invertida para que arriba sea positivo)
        for row in reversed(grid):
            print('|' + ''.join(row) + '|')
        
        print("-" * (width + 2))
        
        # Información del mapa
        print(f"Rango real: X[{min_x:.1f}, {max_x:.1f}] Y[{min_y:.1f}, {max_y:.1f}] cm")
        print(f"Puntos mostrados: {len(trajectory[::step])}/{len(trajectory)}")
        print(f"Obstáculos mostrados: {min(len(obstacles), 100)}/{len(obstacles)}")
    
    def show_sensor_analysis(self, session_data):
        """Análisis de datos de sensores"""
        sensor_readings = session_data.get('sensor_readings', [])
        
        if not sensor_readings:
            print("No hay datos de sensores para analizar")
            return
        
        print(f"\nANÁLISIS DE SENSORES")
        print("="*40)
        
        # Recopilar estadísticas por sensor
        sensor_stats = {}
        
        for reading in sensor_readings:
            for sensor_name, distance in reading['readings'].items():
                if distance > 0:  # Solo lecturas válidas
                    if sensor_name not in sensor_stats:
                        sensor_stats[sensor_name] = []
                    sensor_stats[sensor_name].append(distance)
        
        # Mostrar estadísticas
        sensor_icons = {
            'frontal_izq': 'FL', 'frontal_der': 'FR',
            'trasero_izq': 'BL', 'trasero_der': 'BR'
        }
        
        for sensor_name, distances in sensor_stats.items():
            if distances:
                icon = sensor_icons.get(sensor_name, sensor_name[:2].upper())
                min_dist = min(distances)
                max_dist = max(distances)
                avg_dist = sum(distances) / len(distances)
                
                print(f"{icon} {sensor_name:12}: "
                      f"Min={min_dist:5.1f} Max={max_dist:5.1f} "
                      f"Avg={avg_dist:5.1f} ({len(distances)} lecturas)")
                
                # Histograma simple
                self._print_distance_histogram(distances, sensor_name)
    
    def _print_distance_histogram(self, distances, sensor_name, bins=10, width=30):
        """Imprimir histograma simple de distancias"""
        if len(distances) < 5:
            return
        
        min_dist = min(distances)
        max_dist = max(distances)
        
        if max_dist - min_dist < 1:
            return
        
        # Crear bins
        bin_size = (max_dist - min_dist) / bins
        bin_counts = [0] * bins
        
        for dist in distances:
            bin_idx = min(int((dist - min_dist) / bin_size), bins - 1)
            bin_counts[bin_idx] += 1
        
        max_count = max(bin_counts) if bin_counts else 1
        
        print(f"  Distribución de distancias:")
        for i, count in enumerate(bin_counts):
            bin_start = min_dist + i * bin_size
            bin_end = min_dist + (i + 1) * bin_size
            
            # Crear barra ASCII
            bar_length = int((count / max_count) * width) if max_count > 0 else 0
            bar = '#' * bar_length + ' ' * (width - bar_length)
            
            print(f"    {bin_start:4.0f}-{bin_end:4.0f}cm |{bar}| {count}")
    
    def compare_sessions_visual(self, sessions_data):
        """Comparación visual de múltiples sesiones"""
        if len(sessions_data) < 2:
            print("Se necesitan al menos 2 sesiones para comparar")
            return
        
        print(f"\nCOMPARACIÓN DE {len(sessions_data)} SESIONES")
        print("="*60)
        
        # Tabla comparativa
        headers = ['Sesión', 'Duración', 'Distancia', 'Obstáculos', 'Eventos']
        col_widths = [15, 10, 12, 12, 10]
        
        # Imprimir header
        header_line = "|".join(h.center(w) for h, w in zip(headers, col_widths))
        print(header_line)
        print("-" * len(header_line))
        
        # Datos de cada sesión
        for session in sessions_data:
            info = session.get('session_info', {})
            stats = session.get('statistics', {})
            
            name = info.get('name', 'Unknown')[:14]
            duration = f"{info.get('duration', 0):.0f}s"
            
            traj_stats = stats.get('trajectory_stats', {})
            distance = f"{traj_stats.get('distance_traveled', 0):.0f}cm"
            
            obstacles = str(stats.get('total_obstacles_detected', 0))
            events = str(stats.get('total_mission_events', 0))
            
            values = [name, duration, distance, obstacles, events]
            row = "|".join(v.center(w) for v, w in zip(values, col_widths))
            print(row)
        
        print("-" * len(header_line))
        
        # Análisis comparativo
        distances = []
        durations = []
        
        for session in sessions_data:
            info = session.get('session_info', {})
            stats = session.get('statistics', {})
            
            durations.append(info.get('duration', 0))
            traj_stats = stats.get('trajectory_stats', {})
            distances.append(traj_stats.get('distance_traveled', 0))
        
        if distances and max(distances) > 0:
            print(f"\nAnálisis:")
            print(f"Sesión más larga: {max(durations):.0f}s")
            print(f"Mayor distancia: {max(distances):.0f}cm")
            print(f"Promedio distancia: {sum(distances)/len(distances):.0f}cm")
            
            # Eficiencia (distancia/tiempo)
            efficiencies = [d/t if t > 0 else 0 for d, t in zip(distances, durations)]
            if efficiencies:
                best_efficiency = max(efficiencies)
                best_idx = efficiencies.index(best_efficiency)
                best_session = sessions_data[best_idx].get('session_info', {}).get('name', 'Unknown')
                print(f"Más eficiente: {best_session} ({best_efficiency:.1f} cm/s)")
    
    def show_competition_analysis(self, session_data):
        """Análisis específico para competencia"""
        mission_events = session_data.get('mission_events', [])
        trajectory = session_data.get('trajectory', [])
        
        print(f"\nANÁLISIS DE COMPETENCIA")
        print("="*40)
        
        # Análisis de eventos de misión
        if mission_events:
            blocks_detected = [e for e in mission_events if 'block_detected' in e['type']]
            blocks_collected = [e for e in mission_events if 'block_collected' in e['type']]
            
            print(f"Bloques detectados: {len(blocks_detected)}")
            print(f"Bloques recogidos: {len(blocks_collected)}")
            
            # Calcular puntuación estimada
            total_points = 0
            for event in blocks_collected:
                points = event.get('data', {}).get('points', 0)
                total_points += points
            
            if total_points > 0:
                print(f"Puntuación estimada: {total_points} puntos")
        
        # Análisis de zonas visitadas
        if trajectory:
            zones_visited = self._analyze_zones_visited(trajectory)
            if zones_visited:
                print(f"\nZonas visitadas:")
                for zone, time_spent in zones_visited.items():
                    print(f"  {zone}: {time_spent:.1f}s")
    
    def _analyze_zones_visited(self, trajectory):
        """Analizar qué zonas de competencia se visitaron"""
        zones_time = {}
        
        for i, point in enumerate(trajectory):
            x, y = point['x'], point['y']
            
            # Verificar cada zona
            for zone_name, zone_data in COMPETITION_ZONES.items():
                for subzone_name, subzone in zone_data.items():
                    if ('x_min' in subzone and 'x_max' in subzone and 
                        'y_min' in subzone and 'y_max' in subzone):
                        
                        if (subzone['x_min'] <= x <= subzone['x_max'] and 
                            subzone['y_min'] <= y <= subzone['y_max']):
                            
                            zone_key = f"{zone_name}_{subzone_name}"
                            
                            # Estimar tiempo en zona (diferencia entre puntos)
                            if i > 0:
                                time_diff = point['timestamp'] - trajectory[i-1]['timestamp']
                                zones_time[zone_key] = zones_time.get(zone_key, 0) + time_diff
        
        return zones_time

# ============================================================================
# FUNCIONES DE UTILIDAD
# ============================================================================

def visualize_session_file(session_file):
    """Visualizar una sesión desde archivo"""
    try:
        with open(session_file, 'r') as f:
            session_data = json.load(f)
        
        visualizer = SimpleVisualizer()
        
        # Mostrar resumen
        visualizer.print_session_summary(session_data)
        
        # Mostrar mapa si hay trayectoria
        if session_data.get('trajectory'):
            visualizer.draw_trajectory_map(session_data)
        
        # Análisis de sensores
        visualizer.show_sensor_analysis(session_data)
        
        # Análisis de competencia
        visualizer.show_competition_analysis(session_data)
        
    except Exception as e:
        print(f"Error visualizando sesión: {e}")

def quick_session_overview(sessions_dir="data/sessions"):
    """Vista rápida de todas las sesiones"""
    import os
    
    try:
        files = [f for f in os.listdir(sessions_dir) if f.endswith('.json')]
        
        if not files:
            print("No hay sesiones guardadas")
            return
        
        print(f"\nVISTA RÁPIDA - {len(files)} SESIONES")
        print("="*50)
        
        for filename in sorted(files)[-10:]:  # Últimas 10 sesiones
            filepath = os.path.join(sessions_dir, filename)
            
            try:
                with open(filepath, 'r') as f:
                    data = json.load(f)
                
                info = data.get('session_info', {})
                stats = data.get('statistics', {})
                
                name = info.get('name', filename[:-5])[:20]
                duration = info.get('duration', 0)
                entries = info.get('total_entries', 0)
                
                traj_stats = stats.get('trajectory_stats', {})
                distance = traj_stats.get('distance_traveled', 0)
                
                print(f"{name:20} | {duration:6.0f}s | {entries:6} logs | {distance:6.0f}cm")
                
            except:
                print(f"{filename:20} | ERROR LEYENDO ARCHIVO")
        
        print("="*50)
        
    except Exception as e:
        print(f"Error: {e}")

# ============================================================================
# PROGRAMA DE PRUEBA
# ============================================================================

def test_visualization():
    """Función de prueba del visualizador"""
    print("Prueba del sistema de visualización")
    
    # Crear datos de prueba
    test_session = {
        'session_info': {
            'name': 'test_visualization',
            'duration': 120.5,
            'total_entries': 1205,
            'created_at': datetime.now().isoformat()
        },
        'statistics': {
            'total_obstacles_detected': 45,
            'total_mission_events': 8,
            'trajectory_stats': {
                'distance_traveled': 250.3,
                'total_positions': 600,
                'x_range': [10, 90],
                'y_range': [5, 75]
            },
            'sensor_stats': {
                'total_sensor_readings': 2400,
                'readings_per_sensor': {
                    'frontal_izq': 600,
                    'frontal_der': 600,
                    'trasero_izq': 600,
                    'trasero_der': 600
                }
            }
        },
        'trajectory': [
            {'timestamp': time.time(), 'x': 10, 'y': 10, 'heading': 0},
            {'timestamp': time.time() + 10, 'x': 30, 'y': 20, 'heading': 45},
            {'timestamp': time.time() + 20, 'x': 50, 'y': 40, 'heading': 90},
            {'timestamp': time.time() + 30, 'x': 70, 'y': 60, 'heading': 135},
            {'timestamp': time.time() + 40, 'x': 80, 'y': 70, 'heading': 180}
        ],
        'obstacles_detected': [
            {'x': 25, 'y': 15, 'distance': 20, 'sensor': 'frontal_izq'},
            {'x': 55, 'y': 35, 'distance': 15, 'sensor': 'frontal_der'},
            {'x': 75, 'y': 55, 'distance': 25, 'sensor': 'trasero_izq'}
        ],
        'sensor_readings': [
            {'timestamp': time.time(), 'readings': {'frontal_izq': 50, 'frontal_der': 45, 'trasero_izq': 100, 'trasero_der': 95}},
            {'timestamp': time.time() + 1, 'readings': {'frontal_izq': 45, 'frontal_der': 40, 'trasero_izq': 105, 'trasero_der': 90}}
        ],
        'mission_events': [
            {'timestamp': time.time(), 'type': 'block_detected', 'description': 'Bloque R1 detectado', 'data': {'block_id': 'R1'}},
            {'timestamp': time.time() + 30, 'type': 'block_collected', 'description': 'Bloque recogido', 'data': {'block_id': 'R1', 'points': 8}}
        ]
    }
    
    # Probar visualizador
    visualizer = SimpleVisualizer()
    
    print("\n1. Resumen de sesión:")
    visualizer.print_session_summary(test_session)
    
    print("\n2. Mapa de trayectoria:")
    visualizer.draw_trajectory_map(test_session)
    
    print("\n3. Análisis de sensores:")
    visualizer.show_sensor_analysis(test_session)
    
    print("\n4. Análisis de competencia:")
    visualizer.show_competition_analysis(test_session)
    
    print("\nPrueba completada")

if __name__ == "__main__":
    test_visualization()