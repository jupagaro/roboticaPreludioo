#!/usr/bin/env python3
"""
utils/logging.py
Sistema de logging para datos del robot
Graba datos de sensores, posición y estado durante las sesiones
"""

import json
import os
import time
import threading
from datetime import datetime
from math import sqrt
from config import DATA_PATHS, LOGGING_CONFIG, create_data_directories

class DataLogger:
    """Logger principal para datos del robot durante sesiones"""
    
    def __init__(self, session_name=None):
        # Información de sesión
        self.session_name = session_name or f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.start_time = time.time()
        
        # Datos de logging
        self.log_entries = []           # Log completo con timestamps
        self.trajectory = []            # Posiciones del robot
        self.obstacles_detected = []   # Obstáculos encontrados
        self.sensor_readings = []      # Lecturas de sensores
        self.mission_events = []       # Eventos de misión (bloques recogidos, etc.)
        
        # Control de threading
        self.lock = threading.Lock()
        self.auto_save_active = False
        self.last_auto_save = time.time()
        
        # Crear directorios si no existen
        create_data_directories()
        
        print(f"📊 DataLogger iniciado - Sesión: {self.session_name}")
    
    def log_robot_state(self, robot_data):
        """
        Registrar estado completo del robot
        
        Args:
            robot_data: Dict con datos del robot {position, sensors, motors, etc.}
        """
        current_time = time.time()
        
        # Crear entrada de log completa
        entry = {
            'timestamp': current_time,
            'time_str': datetime.fromtimestamp(current_time).strftime('%H:%M:%S.%f')[:-3],
            'session_time': current_time - self.start_time,
            **robot_data  # Expandir todos los datos del robot
        }
        
        with self.lock:
            self.log_entries.append(entry)
            
            # Registrar posición en trayectoria si está disponible
            if 'position' in robot_data:
                pos = robot_data['position']
                self.trajectory.append({
                    'timestamp': current_time,
                    'x': pos.get('x', 0),
                    'y': pos.get('y', 0),
                    'heading': pos.get('heading', 0)
                })
            
            # Registrar lecturas de sensores
            if 'sensors' in robot_data:
                self.sensor_readings.append({
                    'timestamp': current_time,
                    'readings': robot_data['sensors'].copy()
                })
            
            # Registrar obstáculos detectados
            if 'obstacles' in robot_data:
                for obstacle in robot_data['obstacles']:
                    obstacle_entry = obstacle.copy()
                    obstacle_entry['detection_time'] = current_time
                    self.obstacles_detected.append(obstacle_entry)
        
        # Auto-save periódico
        if (current_time - self.last_auto_save) > LOGGING_CONFIG['auto_save_interval']:
            self._auto_save()
    
    def log_mission_event(self, event_type, description, data=None):
        """
        Registrar evento importante de la misión
        
        Args:
            event_type: Tipo de evento ('block_detected', 'block_collected', etc.)
            description: Descripción del evento
            data: Datos adicionales del evento
        """
        event = {
            'timestamp': time.time(),
            'session_time': time.time() - self.start_time,
            'type': event_type,
            'description': description,
            'data': data or {}
        }
        
        with self.lock:
            self.mission_events.append(event)
        
        print(f"📝 Evento: {description}")
    
    def get_session_statistics(self):
        """Obtener estadísticas de la sesión actual"""
        with self.lock:
            total_entries = len(self.log_entries)
            total_obstacles = len(self.obstacles_detected)
            total_events = len(self.mission_events)
            session_duration = time.time() - self.start_time
            
            # Estadísticas de trayectoria
            trajectory_stats = {}
            if self.trajectory:
                positions = [(p['x'], p['y']) for p in self.trajectory]
                
                if len(positions) > 1:
                    # Calcular distancia recorrida aproximada
                    total_distance = sum(
                        sqrt((positions[i][0] - positions[i-1][0])**2 + 
                             (positions[i][1] - positions[i-1][1])**2)
                        for i in range(1, len(positions))
                    )
                    
                    # Rango de posiciones
                    x_coords = [p[0] for p in positions]
                    y_coords = [p[1] for p in positions]
                    
                    trajectory_stats = {
                        'distance_traveled': total_distance,
                        'x_range': [min(x_coords), max(x_coords)],
                        'y_range': [min(y_coords), max(y_coords)],
                        'total_positions': len(positions)
                    }
            
            # Estadísticas de sensores
            sensor_stats = {}
            if self.sensor_readings:
                # Contar lecturas por sensor
                sensor_counts = {}
                for reading in self.sensor_readings:
                    for sensor_name in reading['readings'].keys():
                        sensor_counts[sensor_name] = sensor_counts.get(sensor_name, 0) + 1
                
                sensor_stats = {
                    'total_sensor_readings': len(self.sensor_readings),
                    'readings_per_sensor': sensor_counts
                }
        
        return {
            'session_name': self.session_name,
            'duration': session_duration,
            'total_log_entries': total_entries,
            'total_obstacles_detected': total_obstacles,
            'total_mission_events': total_events,
            'trajectory_stats': trajectory_stats,
            'sensor_stats': sensor_stats
        }
    
    def _auto_save(self):
        """Guardado automático periódico"""
        if not self.auto_save_active:
            self.auto_save_active = True
            threading.Thread(target=self._perform_auto_save, daemon=True).start()
    
    def _perform_auto_save(self):
        """Realizar guardado automático en thread separado"""
        try:
            filename = f"{self.session_name}_autosave.json"
            self.save_session(filename, auto_save=True)
            self.last_auto_save = time.time()
        except Exception as e:
            print(f"Error en auto-save: {e}")
        finally:
            self.auto_save_active = False
    
    def save_session(self, filename=None, auto_save=False):
        """
        Guardar sesión completa a archivo JSON
        
        Args:
            filename: Nombre del archivo (None = generar automático)
            auto_save: Si es guardado automático (no mostrar mensajes)
        """
        if filename is None:
            filename = f"{self.session_name}.json"
        
        filepath = os.path.join(DATA_PATHS['sessions'], filename)
        
        # Preparar datos para guardado
        session_data = {
            'session_info': {
                'name': self.session_name,
                'start_time': self.start_time,
                'end_time': time.time(),
                'duration': time.time() - self.start_time,
                'created_at': datetime.fromtimestamp(self.start_time).isoformat(),
                'total_entries': len(self.log_entries)
            },
            'statistics': self.get_session_statistics(),
            'trajectory': self.trajectory.copy(),
            'obstacles_detected': self.obstacles_detected.copy(),
            'sensor_readings': self.sensor_readings.copy(),
            'mission_events': self.mission_events.copy(),
            'full_log': self.log_entries.copy()
        }
        
        try:
            with self.lock:
                with open(filepath, 'w') as f:
                    json.dump(session_data, f, indent=2)
            
            if not auto_save:
                print(f"💾 Sesión guardada: {filepath}")
                print(f"📊 Entradas: {len(self.log_entries)}, Obstáculos: {len(self.obstacles_detected)}")
            
            return filepath
            
        except Exception as e:
            print(f"Error guardando sesión: {e}")
            return None

class SessionManager:
    """Manager para múltiples sesiones y análisis histórico"""
    
    def __init__(self):
        self.sessions_dir = DATA_PATHS['sessions']
        create_data_directories()
    
    def list_sessions(self):
        """Listar todas las sesiones guardadas"""
        try:
            files = [f for f in os.listdir(self.sessions_dir) if f.endswith('.json')]
            sessions = []
            
            for filename in files:
                filepath = os.path.join(self.sessions_dir, filename)
                try:
                    # Obtener información básica del archivo
                    stat = os.stat(filepath)
                    size_kb = stat.st_size / 1024
                    modified = datetime.fromtimestamp(stat.st_mtime)
                    
                    # Intentar leer información de la sesión
                    with open(filepath, 'r') as f:
                        data = json.load(f)
                        session_info = data.get('session_info', {})
                    
                    sessions.append({
                        'filename': filename,
                        'session_name': session_info.get('name', filename[:-5]),
                        'created': session_info.get('created_at', modified.isoformat()),
                        'duration': session_info.get('duration', 0),
                        'entries': session_info.get('total_entries', 0),
                        'size_kb': size_kb,
                        'filepath': filepath
                    })
                    
                except Exception as e:
                    # Si no se puede leer el archivo, incluir info básica
                    sessions.append({
                        'filename': filename,
                        'session_name': filename[:-5],
                        'created': modified.isoformat(),
                        'duration': 0,
                        'entries': 0,
                        'size_kb': size_kb,
                        'filepath': filepath,
                        'error': str(e)
                    })
            
            # Ordenar por fecha de creación (más recientes primero)
            sessions.sort(key=lambda x: x['created'], reverse=True)
            return sessions
            
        except Exception as e:
            print(f"Error listando sesiones: {e}")
            return []
    
    def load_session(self, session_name_or_file):
        """
        Cargar una sesión específica
        
        Args:
            session_name_or_file: Nombre de sesión o archivo completo
            
        Returns:
            dict: Datos de la sesión cargada
        """
        # Determinar filepath
        if session_name_or_file.endswith('.json'):
            filepath = os.path.join(self.sessions_dir, session_name_or_file)
        else:
            filepath = os.path.join(self.sessions_dir, f"{session_name_or_file}.json")
        
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error cargando sesión: {e}")
            return None
    
    def compare_sessions(self, session_names):
        """
        Comparar múltiples sesiones
        
        Args:
            session_names: Lista de nombres de sesiones a comparar
            
        Returns:
            dict: Comparación de las sesiones
        """
        sessions_data = []
        
        for name in session_names:
            data = self.load_session(name)
            if data:
                sessions_data.append(data)
        
        if not sessions_data:
            return {"error": "No se pudieron cargar sesiones"}
        
        # Crear comparación
        comparison = {
            'sessions_compared': len(sessions_data),
            'comparison': []
        }
        
        for session in sessions_data:
            info = session.get('session_info', {})
            stats = session.get('statistics', {})
            
            comparison['comparison'].append({
                'name': info.get('name', 'Unknown'),
                'duration': info.get('duration', 0),
                'total_entries': info.get('total_entries', 0),
                'obstacles_detected': stats.get('total_obstacles_detected', 0),
                'distance_traveled': stats.get('trajectory_stats', {}).get('distance_traveled', 0),
                'mission_events': stats.get('total_mission_events', 0)
            })
        
        return comparison
    
    def cleanup_old_sessions(self, days_old=30, keep_minimum=5):
        """
        Limpiar sesiones antiguas
        
        Args:
            days_old: Eliminar sesiones más antiguas que N días
            keep_minimum: Mantener mínimo N sesiones recientes
        """
        sessions = self.list_sessions()
        
        if len(sessions) <= keep_minimum:
            print(f"Solo hay {len(sessions)} sesiones, manteniendo todas")
            return []
        
        current_time = time.time()
        cutoff_time = current_time - (days_old * 24 * 3600)
        
        # Identificar sesiones a eliminar
        sessions_to_delete = []
        sessions_to_keep = sessions[:keep_minimum]  # Mantener las más recientes
        
        for session in sessions[keep_minimum:]:
            try:
                created_timestamp = datetime.fromisoformat(session['created']).timestamp()
                if created_timestamp < cutoff_time:
                    sessions_to_delete.append(session)
            except:
                # Si no se puede parsear la fecha, mantener la sesión
                continue
        
        # Eliminar sesiones
        deleted_sessions = []
        for session in sessions_to_delete:
            try:
                os.remove(session['filepath'])
                deleted_sessions.append(session['filename'])
                print(f"🗑️ Eliminada: {session['filename']}")
            except Exception as e:
                print(f"Error eliminando {session['filename']}: {e}")
        
        if deleted_sessions:
            print(f"🧹 Limpieza completada: {len(deleted_sessions)} sesiones eliminadas")
        else:
            print("🧹 No hay sesiones que eliminar")
        
        return deleted_sessions

# ============================================================================
# FUNCIONES DE PRUEBA
# ============================================================================

def test_logging_system():
    """Función de prueba del sistema de logging"""
    print("🎯 Prueba del sistema de logging")
    
    # Crear logger de prueba
    logger = DataLogger("test_session")
    
    # Simular datos del robot
    print("📊 Simulando datos del robot...")
    
    for i in range(10):
        # Simular estado del robot
        robot_data = {
            'position': {
                'x': i * 10,
                'y': i * 5,
                'heading': i * 15
            },
            'sensors': {
                'frontal_izq': 50 + i * 5,
                'frontal_der': 45 + i * 3,
                'trasero_izq': 100 - i * 2,
                'trasero_der': 95 - i * 4
            },
            'motors': {
                'left_speed': 120,
                'right_speed': 115
            },
            'obstacles': [
                {'x': i * 10 + 20, 'y': i * 5 + 10, 'distance': 30, 'sensor': 'frontal_izq'}
            ] if i % 3 == 0 else []
        }
        
        logger.log_robot_state(robot_data)
        
        # Simular algunos eventos de misión
        if i == 3:
            logger.log_mission_event('block_detected', 'Bloque rojo detectado', {'block_id': 'R1'})
        elif i == 7:
            logger.log_mission_event('block_collected', 'Bloque recogido', {'block_id': 'R1', 'points': 8})
        
        time.sleep(0.1)
    
    # Mostrar estadísticas
    stats = logger.get_session_statistics()
    print(f"\n📊 Estadísticas de la sesión:")
    print(f"Duración: {stats['duration']:.1f}s")
    print(f"Entradas de log: {stats['total_log_entries']}")
    print(f"Obstáculos detectados: {stats['total_obstacles_detected']}")
    print(f"Eventos de misión: {stats['total_mission_events']}")
    
    if stats['trajectory_stats']:
        traj = stats['trajectory_stats']
        print(f"Distancia recorrida: {traj.get('distance_traveled', 0):.1f} cm")
        print(f"Rango X: {traj.get('x_range', [0,0])}")
        print(f"Rango Y: {traj.get('y_range', [0,0])}")
    
    # Guardar sesión
    saved_file = logger.save_session()
    if saved_file:
        print(f"✅ Sesión guardada correctamente")
        
        # Probar SessionManager
        print("\n🗂️ Probando SessionManager...")
        manager = SessionManager()
        sessions = manager.list_sessions()
        
        print(f"📁 Sesiones encontradas: {len(sessions)}")
        for session in sessions[:3]:  # Mostrar primeras 3
            print(f"  - {session['session_name']} ({session['entries']} entradas, {session['size_kb']:.1f}KB)")
        
        # Cargar la sesión que acabamos de crear
        loaded = manager.load_session("test_session")
        if loaded:
            print("✅ Sesión cargada correctamente")
        else:
            print("❌ Error cargando sesión")
    
    print("🎯 Prueba completada")

if __name__ == "__main__":
    test_logging_system()