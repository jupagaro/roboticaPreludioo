#!/usr/bin/env python3
"""
robot_controller.py
Controlador principal que integra todos los sistemas
Combina sensor fusion + mapa de competencia + control manual/automático
"""

import time
import threading
import sys
import select
import termios
import tty
from sensor_fusion import SensorFusion
from competition_map import CompetitionMap
from utils import DataLogger
from config import INITIAL_POSITION, MANUAL_SPEEDS

class RobotController:
    """Controlador principal del robot de rescate"""
    
    def __init__(self, session_name=None, enable_competition_mode=True):
        # Inicializar componentes principales
        self.sensor_fusion = SensorFusion(session_name, enable_logging=True)
        
        if enable_competition_mode:
            self.competition_map = CompetitionMap()
        else:
            self.competition_map = None
        
        # Estado del robot
        self.carrying_block = None
        self.current_mission = 'exploration'  # exploration, collection, delivery
        self.mission_active = False
        
        # Control manual
        self.manual_control_active = False
        self.current_speed_level = 'medium'
        
        # Threading
        self.running = False
        self.control_lock = threading.Lock()
        
        # Establecer posición inicial
        if self.competition_map:
            self.sensor_fusion.position.update(INITIAL_POSITION)
        
        print(f"Robot Controller inicializado")
        print(f"Sesión: {self.sensor_fusion.logger.session_name if self.sensor_fusion.enable_logging else 'Sin logging'}")
        print(f"Modo competencia: {'Activado' if self.competition_map else 'Desactivado'}")
    
    # ========================================================================
    # CONTROL MANUAL
    # ========================================================================
    
    def get_key_input(self):
        """Capturar tecla sin bloquear"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def start_manual_control(self):
        """Iniciar modo de control manual con teclas WASD"""
        print("\nMODO CONTROL MANUAL ACTIVADO")
        print("=" * 40)
        print("Controles:")
        print("  W/S: Adelante/Atrás")
        print("  A/D: Girar izquierda/derecha") 
        print("  Q/E: Spin izquierda/derecha")
        print("  1/2/3: Velocidad lenta/media/rápida")
        print("  SPACE: Parar")
        print("  M: Mostrar estado")
        print("  B: Buscar bloques")
        print("  X: Salir")
        print("=" * 40)
        
        self.manual_control_active = True
        
        # Configurar terminal para captura de teclas
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.cbreak(sys.stdin.fileno())
            
            while self.manual_control_active and self.running:
                key = self.get_key_input()
                
                if key:
                    self._process_manual_command(key.lower())
                
                time.sleep(0.05)  # 20Hz para respuesta suave
                
        except KeyboardInterrupt:
            print("\nControl manual interrumpido")
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.sensor_fusion.motors.stop()
            self.manual_control_active = False
    
    def _process_manual_command(self, key):
        """Procesar comando manual"""
        speed = MANUAL_SPEEDS[self.current_speed_level]
        
        # Comandos de movimiento
        if key == 'w':      # Adelante
            self.sensor_fusion.motors.move_forward(speed)
        elif key == 's':    # Atrás
            self.sensor_fusion.motors.move_backward(speed)
        elif key == 'a':    # Girar izquierda
            self.sensor_fusion.motors.spin_left(speed)
        elif key == 'd':    # Girar derecha
            self.sensor_fusion.motors.spin_right(speed)
        elif key == 'q':    # Curva izquierda
            self.sensor_fusion.motors.turn_left(speed)
        elif key == 'e':    # Curva derecha
            self.sensor_fusion.motors.turn_right(speed)
        elif key == ' ':    # Parar
            self.sensor_fusion.motors.stop()
        
        # Cambios de velocidad
        elif key == '1':
            self.current_speed_level = 'slow'
            print(f"Velocidad: {self.current_speed_level}")
        elif key == '2':
            self.current_speed_level = 'medium'
            print(f"Velocidad: {self.current_speed_level}")
        elif key == '3':
            self.current_speed_level = 'fast'
            print(f"Velocidad: {self.current_speed_level}")
        
        # Comandos especiales
        elif key == 'm':
            self._print_status()
        elif key == 'b':
            self._search_for_blocks()
        elif key == 'x':
            print("Saliendo del control manual...")
            self.manual_control_active = False
    
    # ========================================================================
    # MISIONES AUTOMÁTICAS
    # ========================================================================
    
    def start_autonomous_mission(self):
        """Iniciar misión autónoma de competencia"""
        if not self.competition_map:
            print("Error: Modo competencia no activado")
            return False
        
        print("Iniciando misión autónoma de rescate")
        self.mission_active = True
        
        # Iniciar thread de misión
        self.mission_thread = threading.Thread(target=self._autonomous_mission_loop, daemon=True)
        self.mission_thread.start()
        
        return True
    
    def _autonomous_mission_loop(self):
        """Loop principal de misión autónoma"""
        while self.mission_active and self.running:
            try:
                # Obtener prioridades de misión
                priority = self.competition_map.calculate_mission_priority(
                    self.sensor_fusion.position['x'], 
                    self.sensor_fusion.position['y']
                )
                
                # Ejecutar acción según prioridad
                if priority['recommended_action'] == 'deliver' and self.carrying_block:
                    self._execute_delivery_mission()
                elif priority['recommended_action'] == 'collect':
                    self._execute_collection_mission()
                else:
                    self._execute_exploration_mission()
                
                time.sleep(0.5)  # 2Hz para decisiones de alto nivel
                
            except Exception as e:
                print(f"Error en misión autónoma: {e}")
                time.sleep(1)
    
    def _execute_exploration_mission(self):
        """Misión de exploración - navegación reactiva"""
        action = self.sensor_fusion.reactive_navigation()
        
        # Buscar bloques mientras explora
        potential_blocks = self.sensor_fusion.get_navigation_data()['potential_blocks']
        
        for potential_block in potential_blocks:
            # Registrar bloque detectado en el mapa
            robot_pos = self.sensor_fusion.position
            
            # Estimar posición del bloque
            block_distance = potential_block['distance']
            sensor_name = potential_block['sensor']
            
            # Aquí podrías añadir lógica más sofisticada para identificar el bloque
            print(f"Posible bloque detectado por {sensor_name} a {block_distance}cm")
    
    def _execute_collection_mission(self):
        """Misión de recolección - ir a bloque más cercano"""
        robot_pos = self.sensor_fusion.position
        
        # Buscar bloque más cercano
        nearest_block = self.competition_map.find_nearest_block(
            robot_pos['x'], robot_pos['y'], status='free'
        )
        
        if nearest_block:
            # Navegar hacia el bloque
            result = self.sensor_fusion.navigate_to_point(
                nearest_block['current_x'], 
                nearest_block['current_y']
            )
            
            if result['status'] == 'arrived':
                # Simular recogida del bloque
                self._simulate_block_pickup(nearest_block['id'])
    
    def _execute_delivery_mission(self):
        """Misión de entrega - llevar bloque a zona objetivo"""
        if not self.carrying_block:
            return
        
        # Obtener zona objetivo óptima
        target_zone = self.competition_map.get_optimal_target_zone(
            self.carrying_block['type']
        )
        
        if target_zone:
            result = self.sensor_fusion.navigate_to_point(
                target_zone['x'], target_zone['y']
            )
            
            if result['status'] == 'arrived':
                # Simular entrega
                self._simulate_block_delivery()
    
    def _simulate_block_pickup(self, block_id):
        """Simular recogida de bloque"""
        if self.competition_map.collect_block(
            block_id, 
            self.sensor_fusion.position['x'], 
            self.sensor_fusion.position['y']
        ):
            self.carrying_block = self.competition_map.blocks[block_id]
            
            # Log del evento
            if self.sensor_fusion.enable_logging:
                self.sensor_fusion.logger.log_mission_event(
                    'block_collected',
                    f"Bloque {block_id} recogido",
                    {'block_id': block_id, 'type': self.carrying_block['type']}
                )
            
            print(f"Bloque {block_id} recogido")
    
    def _simulate_block_delivery(self):
        """Simular entrega de bloque"""
        if self.carrying_block:
            points = self.competition_map.deliver_block(
                self.carrying_block['id'],
                self.sensor_fusion.position['x'],
                self.sensor_fusion.position['y']
            )
            
            # Log del evento
            if self.sensor_fusion.enable_logging:
                self.sensor_fusion.logger.log_mission_event(
                    'block_delivered',
                    f"Bloque {self.carrying_block['id']} entregado",
                    {'block_id': self.carrying_block['id'], 'points': points}
                )
            
            print(f"Bloque {self.carrying_block['id']} entregado - {points} puntos")
            self.carrying_block = None
    
    # ========================================================================
    # FUNCIONES AUXILIARES
    # ========================================================================
    
    def _search_for_blocks(self):
        """Buscar bloques en el área"""
        nav_data = self.sensor_fusion.get_navigation_data()
        potential_blocks = nav_data['potential_blocks']
        
        print(f"\nBúsqueda de bloques:")
        if potential_blocks:
            print(f"Bloques potenciales detectados: {len(potential_blocks)}")
            for i, block in enumerate(potential_blocks, 1):
                print(f"  {i}. Sensor: {block['sensor']}, Distancia: {block['distance']:.1f}cm")
        else:
            print("No se detectaron bloques cercanos")
        
        # Si hay competencia, mostrar bloques conocidos cercanos
        if self.competition_map:
            robot_pos = self.sensor_fusion.position
            nearest = self.competition_map.find_nearest_block(robot_pos['x'], robot_pos['y'])
            if nearest:
                distance = ((nearest['current_x'] - robot_pos['x'])**2 + 
                           (nearest['current_y'] - robot_pos['y'])**2)**0.5
                print(f"Bloque más cercano: {nearest['id']} a {distance:.1f}cm")
    
    def _print_status(self):
        """Mostrar estado completo del robot"""
        print(f"\n=== ESTADO DEL ROBOT ===")
        
        # Estado del sensor fusion
        self.sensor_fusion.print_status()
        
        # Estado de misión
        if self.carrying_block:
            print(f"Transportando: {self.carrying_block['id']} ({self.carrying_block['type']})")
        else:
            print("Sin bloque")
        
        print(f"Misión actual: {self.current_mission}")
        print(f"Control: {'Manual' if self.manual_control_active else 'Automático'}")
        
        # Estadísticas de competencia
        if self.competition_map:
            stats = self.competition_map.get_mission_statistics()
            print(f"Puntuación: {stats['total_score']} puntos")
            print(f"Tiempo restante: {stats['time_remaining']:.1f}s")
            print(f"Bloques entregados: {stats['status_counts'].get('delivered', 0)}")
    
    # ========================================================================
    # CONTROL DEL SISTEMA
    # ========================================================================
    
    def start_system(self, sensor_update_rate=10):
        """Iniciar sistema completo"""
        self.running = True
        
        # Iniciar sensor fusion
        self.sensor_fusion.start_sensor_loop(sensor_update_rate)
        
        print(f"Sistema iniciado (sensores a {sensor_update_rate}Hz)")
    
    def stop_system(self):
        """Detener sistema completo"""
        print("Deteniendo sistema...")
        
        # Detener misiones
        self.mission_active = False
        self.manual_control_active = False
        
        # Detener sensor fusion
        self.sensor_fusion.stop()
        
        self.running = False
        
        print("Sistema detenido")
    
    def get_system_summary(self):
        """Obtener resumen completo del sistema"""
        fusion_status = self.sensor_fusion.get_system_status()
        
        summary = {
            'robot_status': {
                'position': self.sensor_fusion.position.copy(),
                'carrying_block': self.carrying_block['id'] if self.carrying_block else None,
                'current_mission': self.current_mission,
                'system_health': fusion_status['system_health']
            },
            'sensor_fusion': fusion_status,
        }
        
        if self.competition_map:
            summary['competition'] = {
                'statistics': self.competition_map.get_mission_statistics(),
                'efficiency': self.competition_map.get_efficiency_metrics()
            }
        
        return summary
    
    def cleanup(self):
        """Limpiar recursos"""
        self.stop_system()
        self.sensor_fusion.cleanup()
        print("Recursos liberados")

# ============================================================================
# FUNCIONES DE PRUEBA
# ============================================================================

def test_robot_controller():
    """Función de prueba del controlador completo"""
    print("Prueba del controlador principal del robot")
    
    try:
        # Crear controlador
        robot = RobotController("test_controller", enable_competition_mode=True)
        
        # Iniciar sistema
        robot.start_system()
        
        # Menú de opciones
        while True:
            print(f"\nOPCIONES DEL ROBOT:")
            print("1. Control manual (WASD)")
            print("2. Misión autónoma")
            print("3. Navegación reactiva (10s)")
            print("4. Ir a punto específico")
            print("5. Mostrar estado completo")
            print("6. Buscar bloques")
            print("7. Estadísticas de competencia")
            print("0. Salir")
            
            choice = input("Elegir [0-7]: ").strip()
            
            if choice == '1':
                robot.start_manual_control()
            
            elif choice == '2':
                if robot.start_autonomous_mission():
                    print("Misión autónoma iniciada")
                    print("Presiona Enter para detener...")
                    input()
                    robot.mission_active = False
                    robot.sensor_fusion.motors.stop()
                    print("Misión detenida")
            
            elif choice == '3':
                print("Navegación reactiva por 10 segundos...")
                start_time = time.time()
                while (time.time() - start_time) < 10:
                    action = robot.sensor_fusion.reactive_navigation()
                    print(f"\rAcción: {action}", end="", flush=True)
                    time.sleep(0.5)
                robot.sensor_fusion.motors.stop()
                print("\nNavegación detenida")
            
            elif choice == '4':
                try:
                    x = float(input("X objetivo: "))
                    y = float(input("Y objetivo: "))
                    
                    print(f"Navegando a ({x}, {y})...")
                    while True:
                        result = robot.sensor_fusion.navigate_to_point(x, y)
                        print(f"\r{result['status']} - {result['distance']:.1f}cm", 
                              end="", flush=True)
                        
                        if result['status'] == 'arrived':
                            print("\nObjetivo alcanzado")
                            break
                        
                        time.sleep(0.5)
                        
                except (ValueError, KeyboardInterrupt):
                    robot.sensor_fusion.motors.stop()
                    print("Navegación cancelada")
            
            elif choice == '5':
                robot._print_status()
            
            elif choice == '6':
                robot._search_for_blocks()
            
            elif choice == '7':
                if robot.competition_map:
                    robot.competition_map.print_mission_summary()
                else:
                    print("Modo competencia no activado")
            
            elif choice == '0':
                break
            
            else:
                print("Opción inválida")
    
    except KeyboardInterrupt:
        print("\nPrueba interrumpida")
    
    finally:
        if 'robot' in locals():
            robot.cleanup()

if __name__ == "__main__":
    test_robot_controller()