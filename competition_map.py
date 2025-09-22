#!/usr/bin/env python3
"""
competition_map.py
Mapa de la competencia y lógica específica para el desafío de rescate
Maneja zonas, bloques, puntuación y estrategia
"""

import time
from math import sqrt, atan2, degrees
from config import (
    FIELD_DIMENSIONS, COMPETITION_ZONES, INITIAL_BLOCKS, 
    DEBRIS_PILES, INITIAL_POSITION, get_optimal_target_for_block,
    calculate_block_points, is_point_in_zone
)

class CompetitionMap:
    """Mapa de la competencia con estado dinámico"""
    
    def __init__(self):
        # Dimensiones del campo
        self.field_width = FIELD_DIMENSIONS['width']
        self.field_height = FIELD_DIMENSIONS['height']
        
        # Zonas estáticas
        self.zones = COMPETITION_ZONES.copy()
        self.debris_piles = DEBRIS_PILES.copy()
        
        # Estado dinámico de bloques
        self.blocks = self._initialize_blocks()
        
        # Mapa de obstáculos descubiertos
        self.discovered_obstacles = []
        self.static_obstacles = []
        
        # Información de misión
        self.mission_start_time = time.time()
        self.mission_duration = 120  # 2 minutos
        
        print(f"Mapa de competencia inicializado:")
        print(f"  Campo: {self.field_width} x {self.field_height} cm")
        print(f"  Bloques totales: {self._count_blocks()}")
        print(f"  Zonas de puntuación: Hospital, Refugio")
    
    def _initialize_blocks(self):
        """Inicializar estado de todos los bloques"""
        blocks = {}
        
        for category, block_list in INITIAL_BLOCKS.items():
            for block in block_list:
                block_id = block['id']
                blocks[block_id] = {
                    'id': block_id,
                    'type': 'red' if block_id.startswith('R') else 'green',
                    'original_x': block['x'],
                    'original_y': block['y'],
                    'current_x': block['x'],
                    'current_y': block['y'],
                    'status': 'free',  # free, detected, collected, delivered
                    'points_earned': 0,
                    'detection_time': None,
                    'collection_time': None,
                    'delivery_time': None,
                    'category': category  # red_debris, green_circuit, etc.
                }
        
        return blocks
    
    def _count_blocks(self):
        """Contar bloques por tipo"""
        red_count = len([b for b in self.blocks.values() if b['type'] == 'red'])
        green_count = len([b for b in self.blocks.values() if b['type'] == 'green'])
        return {'red': red_count, 'green': green_count, 'total': red_count + green_count}
    
    # ========================================================================
    # GESTIÓN DE BLOQUES
    # ========================================================================
    
    def find_nearest_block(self, robot_x, robot_y, block_type=None, status='free'):
        """
        Encontrar el bloque más cercano
        
        Args:
            robot_x, robot_y: Posición actual del robot
            block_type: 'red', 'green' o None para cualquiera
            status: Estado del bloque ('free', 'detected', etc.)
        """
        candidates = []
        
        for block in self.blocks.values():
            # Filtros
            if status and block['status'] != status:
                continue
            if block_type and block['type'] != block_type:
                continue
            
            # Calcular distancia
            distance = sqrt((block['current_x'] - robot_x)**2 + 
                          (block['current_y'] - robot_y)**2)
            
            candidates.append((distance, block))
        
        if candidates:
            candidates.sort(key=lambda x: x[0])
            return candidates[0][1]  # Retornar el bloque más cercano
        
        return None
    
    def detect_block(self, block_id, robot_x, robot_y):
        """Marcar un bloque como detectado"""
        if block_id in self.blocks:
            block = self.blocks[block_id]
            if block['status'] == 'free':
                block['status'] = 'detected'
                block['detection_time'] = time.time()
                
                print(f"Bloque detectado: {block_id} ({block['type']})")
                return True
        return False
    
    def collect_block(self, block_id, robot_x, robot_y):
        """Marcar un bloque como recogido"""
        if block_id in self.blocks:
            block = self.blocks[block_id]
            if block['status'] in ['free', 'detected']:
                block['status'] = 'collected'
                block['collection_time'] = time.time()
                block['current_x'] = robot_x
                block['current_y'] = robot_y
                
                print(f"Bloque recogido: {block_id}")
                return True
        return False
    
    def deliver_block(self, block_id, delivery_x, delivery_y):
        """Marcar un bloque como entregado y calcular puntos"""
        if block_id in self.blocks:
            block = self.blocks[block_id]
            if block['status'] == 'collected':
                # Calcular puntos según posición de entrega
                points = calculate_block_points(block_id, delivery_x, delivery_y)
                
                block['status'] = 'delivered'
                block['delivery_time'] = time.time()
                block['current_x'] = delivery_x
                block['current_y'] = delivery_y
                block['points_earned'] = points
                
                print(f"Bloque entregado: {block_id} -> {points} puntos")
                return points
        return 0
    
    def get_block_by_position(self, x, y, tolerance=10):
        """Encontrar bloque en una posición específica"""
        for block in self.blocks.values():
            if block['status'] == 'free':
                distance = sqrt((block['current_x'] - x)**2 + (block['current_y'] - y)**2)
                if distance <= tolerance:
                    return block
        return None
    
    # ========================================================================
    # NAVEGACIÓN Y ESTRATEGIA
    # ========================================================================
    
    def get_optimal_target_zone(self, block_type):
        """Obtener zona objetivo óptima para un tipo de bloque"""
        target = get_optimal_target_for_block(f"{block_type[0].upper()}1")  # Ejemplo con ID
        return target
    
    def plan_collection_route(self, robot_x, robot_y, max_blocks=5):
        """Planificar ruta de recolección óptima"""
        free_blocks = [b for b in self.blocks.values() if b['status'] == 'free']
        
        if not free_blocks:
            return []
        
        # Ordenar por distancia al robot
        blocks_with_distance = []
        for block in free_blocks:
            distance = sqrt((block['current_x'] - robot_x)**2 + 
                          (block['current_y'] - robot_y)**2)
            blocks_with_distance.append((distance, block))
        
        blocks_with_distance.sort(key=lambda x: x[0])
        
        # Seleccionar los más cercanos, priorizando bloques rojos (más puntos)
        route = []
        red_blocks = [b for d, b in blocks_with_distance if b['type'] == 'red']
        green_blocks = [b for d, b in blocks_with_distance if b['type'] == 'green']
        
        # Priorizar rojos, luego verdes
        route.extend(red_blocks[:max_blocks//2])
        route.extend(green_blocks[:max_blocks - len(route)])
        
        return route[:max_blocks]
    
    def calculate_mission_priority(self, robot_x, robot_y):
        """Calcular prioridad de misión según tiempo restante y oportunidades"""
        time_remaining = self.get_time_remaining()
        
        # Fase de la misión según tiempo restante
        if time_remaining > 90:
            phase = 'exploration'    # Explorar y mapear
        elif time_remaining > 30:
            phase = 'collection'     # Recoger bloques
        else:
            phase = 'final_rush'     # Entregas finales
        
        # Bloques disponibles
        free_blocks = len([b for b in self.blocks.values() if b['status'] == 'free'])
        collected_blocks = len([b for b in self.blocks.values() if b['status'] == 'collected'])
        
        # Determinar acción recomendada
        if collected_blocks > 0:
            action = 'deliver'
        elif free_blocks > 0 and phase != 'final_rush':
            action = 'collect'
        else:
            action = 'explore'
        
        return {
            'phase': phase,
            'time_remaining': time_remaining,
            'recommended_action': action,
            'free_blocks': free_blocks,
            'collected_blocks': collected_blocks
        }
    
    # ========================================================================
    # ANÁLISIS DEL MAPA
    # ========================================================================
    
    def add_discovered_obstacle(self, x, y, confidence='medium', source='ultrasonic'):
        """Añadir obstáculo descubierto al mapa"""
        # Verificar si ya existe un obstáculo cercano
        for obs in self.discovered_obstacles:
            distance = sqrt((obs['x'] - x)**2 + (obs['y'] - y)**2)
            if distance < 15:  # 15cm de tolerancia para duplicados
                obs['confidence'] = max(obs['confidence'], confidence, key=['low', 'medium', 'high'].index)
                obs['last_seen'] = time.time()
                return obs
        
        # Nuevo obstáculo
        obstacle = {
            'x': x,
            'y': y,
            'confidence': confidence,
            'source': source,
            'first_seen': time.time(),
            'last_seen': time.time(),
            'type': 'unknown'
        }
        
        self.discovered_obstacles.append(obstacle)
        return obstacle
    
    def is_path_clear(self, start_x, start_y, end_x, end_y, robot_width=None):
        """Verificar si un path está libre de obstáculos"""
        if robot_width is None:
            robot_width = 20  # cm, margen de seguridad
        
        # Verificar obstáculos conocidos
        for obs in self.discovered_obstacles + self.static_obstacles:
            # Distancia punto-línea aproximada
            line_length = sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
            if line_length == 0:
                continue
            
            # Proyección del obstáculo sobre la línea
            t = ((obs['x'] - start_x) * (end_x - start_x) + 
                 (obs['y'] - start_y) * (end_y - start_y)) / (line_length**2)
            
            if 0 <= t <= 1:  # Proyección dentro del segmento
                proj_x = start_x + t * (end_x - start_x)
                proj_y = start_y + t * (end_y - start_y)
                
                distance = sqrt((obs['x'] - proj_x)**2 + (obs['y'] - proj_y)**2)
                if distance < robot_width:
                    return False
        
        # Verificar pilas de escombros
        for pile in self.debris_piles:
            pile_radius = max(pile['width'], pile['depth']) / 2 + robot_width
            
            # Similar análisis punto-línea para cada pila
            line_length = sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
            if line_length == 0:
                continue
                
            t = ((pile['x'] - start_x) * (end_x - start_x) + 
                 (pile['y'] - start_y) * (end_y - start_y)) / (line_length**2)
            
            if 0 <= t <= 1:
                proj_x = start_x + t * (end_x - start_x)
                proj_y = start_y + t * (end_y - start_y)
                
                distance = sqrt((pile['x'] - proj_x)**2 + (pile['y'] - proj_y)**2)
                if distance < pile_radius:
                    return False
        
        return True
    
    def get_zone_info(self, x, y):
        """Obtener información de la zona en una posición"""
        for zone_name, zone_data in self.zones.items():
            for subzone_name, subzone in zone_data.items():
                if is_point_in_zone(x, y, zone_name, subzone_name):
                    return {
                        'zone': zone_name,
                        'subzone': subzone_name,
                        'points_red': subzone.get('points_red', 0),
                        'points_green': subzone.get('points_green', 0),
                        'elevated': subzone.get('elevated', False)
                    }
        
        return {'zone': 'field', 'subzone': None, 'points_red': 0, 'points_green': 0}
    
    # ========================================================================
    # ESTADÍSTICAS Y ANÁLISIS
    # ========================================================================
    
    def get_mission_statistics(self):
        """Obtener estadísticas de la misión actual"""
        current_time = time.time()
        elapsed_time = current_time - self.mission_start_time
        
        # Contadores por estado
        status_counts = {}
        for block in self.blocks.values():
            status = block['status']
            status_counts[status] = status_counts.get(status, 0) + 1
        
        # Puntuación total
        total_score = sum(block['points_earned'] for block in self.blocks.values())
        
        # Análisis por tipo
        red_stats = self._analyze_blocks_by_type('red')
        green_stats = self._analyze_blocks_by_type('green')
        
        return {
            'mission_time': elapsed_time,
            'time_remaining': self.get_time_remaining(),
            'total_score': total_score,
            'status_counts': status_counts,
            'red_blocks': red_stats,
            'green_blocks': green_stats,
            'discovered_obstacles': len(self.discovered_obstacles),
            'completion_percentage': (status_counts.get('delivered', 0) / len(self.blocks)) * 100
        }
    
    def _analyze_blocks_by_type(self, block_type):
        """Análisis detallado por tipo de bloque"""
        type_blocks = [b for b in self.blocks.values() if b['type'] == block_type]
        
        if not type_blocks:
            return {}
        
        delivered = [b for b in type_blocks if b['status'] == 'delivered']
        total_points = sum(b['points_earned'] for b in delivered)
        
        return {
            'total': len(type_blocks),
            'delivered': len(delivered),
            'points_earned': total_points,
            'average_points': total_points / len(delivered) if delivered else 0
        }
    
    def get_time_remaining(self):
        """Obtener tiempo restante de misión en segundos"""
        elapsed = time.time() - self.mission_start_time
        return max(0, self.mission_duration - elapsed)
    
    def get_efficiency_metrics(self):
        """Calcular métricas de eficiencia"""
        stats = self.get_mission_statistics()
        elapsed_time = stats['mission_time']
        
        if elapsed_time <= 0:
            return {}
        
        return {
            'points_per_minute': (stats['total_score'] / elapsed_time) * 60,
            'blocks_per_minute': (stats['status_counts'].get('delivered', 0) / elapsed_time) * 60,
            'exploration_efficiency': len(self.discovered_obstacles) / elapsed_time,
            'success_rate': stats['completion_percentage']
        }
    
    # ========================================================================
    # FUNCIONES DE ESTADO
    # ========================================================================
    
    def reset_mission(self):
        """Reiniciar misión para nueva sesión"""
        self.blocks = self._initialize_blocks()
        self.discovered_obstacles = []
        self.mission_start_time = time.time()
        print("Misión reiniciada")
    
    def export_map_state(self):
        """Exportar estado completo del mapa"""
        return {
            'field_dimensions': {'width': self.field_width, 'height': self.field_height},
            'mission_info': {
                'start_time': self.mission_start_time,
                'duration': self.mission_duration,
                'elapsed': time.time() - self.mission_start_time
            },
            'blocks': self.blocks.copy(),
            'discovered_obstacles': self.discovered_obstacles.copy(),
            'statistics': self.get_mission_statistics(),
            'efficiency': self.get_efficiency_metrics()
        }
    
    def print_mission_summary(self):
        """Mostrar resumen de la misión actual"""
        stats = self.get_mission_statistics()
        efficiency = self.get_efficiency_metrics()
        
        print(f"\n=== RESUMEN DE MISIÓN ===")
        print(f"Tiempo transcurrido: {stats['mission_time']:.1f}s")
        print(f"Tiempo restante: {stats['time_remaining']:.1f}s")
        print(f"Puntuación total: {stats['total_score']} puntos")
        print(f"Progreso: {stats['completion_percentage']:.1f}% completado")
        
        print(f"\nBloques:")
        for status, count in stats['status_counts'].items():
            print(f"  {status}: {count}")
        
        if efficiency:
            print(f"\nEficiencia:")
            print(f"  {efficiency['points_per_minute']:.1f} puntos/min")
            print(f"  {efficiency['blocks_per_minute']:.1f} bloques/min")
        
        print(f"\nObstáculos descubiertos: {stats['discovered_obstacles']}")

# ============================================================================
# FUNCIONES DE PRUEBA
# ============================================================================

def test_competition_map():
    """Función de prueba del mapa de competencia"""
    print("Prueba del mapa de competencia")
    
    # Crear mapa
    comp_map = CompetitionMap()
    
    # Simular robot en posición inicial
    robot_x, robot_y = INITIAL_POSITION['x'], INITIAL_POSITION['y']
    print(f"\nRobot en posición inicial: ({robot_x}, {robot_y})")
    
    # Buscar bloque más cercano
    nearest = comp_map.find_nearest_block(robot_x, robot_y)
    if nearest:
        print(f"Bloque más cercano: {nearest['id']} en ({nearest['current_x']}, {nearest['current_y']})")
        
        # Simular detección
        comp_map.detect_block(nearest['id'], robot_x, robot_y)
        
        # Simular recolección
        comp_map.collect_block(nearest['id'], robot_x, robot_y)
        
        # Simular entrega
        target = comp_map.get_optimal_target_zone(nearest['type'])
        if target:
            points = comp_map.deliver_block(nearest['id'], target['x'], target['y'])
            print(f"Puntos obtenidos: {points}")
    
    # Añadir algunos obstáculos
    comp_map.add_discovered_obstacle(30, 30, 'high', 'ultrasonic')
    comp_map.add_discovered_obstacle(60, 45, 'medium', 'ultrasonic')
    
    # Verificar path
    path_clear = comp_map.is_path_clear(robot_x, robot_y, 50, 50)
    print(f"Path libre a (50,50): {path_clear}")
    
    # Estadísticas
    print("\n=== ESTADÍSTICAS ===")
    comp_map.print_mission_summary()
    
    # Planificar ruta
    route = comp_map.plan_collection_route(robot_x, robot_y, max_blocks=3)
    print(f"\nRuta planificada ({len(route)} bloques):")
    for i, block in enumerate(route, 1):
        print(f"  {i}. {block['id']} en ({block['current_x']}, {block['current_y']})")
    
    print("\nPrueba completada")

if __name__ == "__main__":
    test_competition_map()