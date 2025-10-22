#!/usr/bin/env python3
"""
main.py
Programa principal del robot de rescate
Punto de entrada para todas las funcionalidades
"""

import os
import sys
import argparse
from datetime import datetime
import RPi.GPIO as GPIO

# Importar módulos principales
from robot_controller import RobotController
from calibration import calibration_wizard
from movement_recorder import movement_recorder_menu
from utils import SessionManager, SimpleVisualizer
from config import validate_config, create_data_directories

def print_banner():
    """Mostrar banner del programa"""
    banner = """
╔══════════════════════════════════════════════════════════════╗
║                    ROBOT DE RESCATE 2025                    ║
║              Sistema de Sensor Fusion Autónomo              ║
╠══════════════════════════════════════════════════════════════╣
║  Hardware: 4x HC-SR04 + MPU6050 + L298N + Raspberry Pi 4   ║
║  Competencia: Robotics for Good Youth Challenge             ║
╚══════════════════════════════════════════════════════════════╝
"""
    print(banner)

def show_main_menu():
    """Mostrar menú principal"""
    print("\n" + "="*60)
    print("MENÚ PRINCIPAL")
    print("="*60)
    print("1. 🤖 Modo Experimental (Control Manual + Logging)")
    print("2. 🏆 Modo Competencia (Misión Autónoma)")
    print("3. 🎮 Solo Control Manual (Práctica)")
    print("4. 🔧 Calibración de Robot")
    print("5. 📝 Movement Recorder (Entrenamiento)")
    print("6. 📊 Análisis de Sesiones")
    print("7. 🗂️  Gestión de Datos")
    print("8. ⚙️  Configuración del Sistema")
    print("9. ℹ️  Información del Sistema")
    print("0. 🚪 Salir")
    print("="*60)

def experimental_mode():
    """Modo experimental - control manual con logging completo"""
    print("\n🤖 MODO EXPERIMENTAL")
    print("Control manual con registro completo de datos")
    
    session_name = input("Nombre de sesión (Enter=auto): ").strip()
    if not session_name:
        session_name = f"experimental_{datetime.now().strftime('%m%d_%H%M')}"
    
    print(f"Iniciando sesión: {session_name}")
    
    try:
        robot = RobotController(session_name, enable_competition_mode=False)
        robot.start_system(sensor_update_rate=10)
        
        print("Sistema iniciado. Comenzando control manual...")
        robot.start_manual_control()
        
    except KeyboardInterrupt:
        print("\nModo experimental interrumpido")
    except Exception as e:
        print(f"Error en modo experimental: {e}")
    finally:
        if 'robot' in locals():
            robot.cleanup()
        GPIO.cleanup()

def competition_mode():
    """Modo competencia - misión autónoma completa"""
    print("\n🏆 MODO COMPETENCIA")
    print("Misión autónoma de rescate (120 segundos)")
    
    session_name = f"competition_{datetime.now().strftime('%m%d_%H%M')}"
    
    print("Opciones de competencia:")
    print("1. Misión automática completa")
    print("2. Práctica con control manual")
    print("3. Navegación reactiva simple")
    
    choice = input("Elegir [1-3]: ").strip()
    
    try:
        robot = RobotController(session_name, enable_competition_mode=True)
        robot.start_system(sensor_update_rate=10)
        
        if choice == '1':
            print("Iniciando misión autónoma...")
            print("El robot buscará bloques y navegará autónomamente")
            
            robot.start_autonomous_mission()
            
            # Monitorear misión
            import time
            start_time = time.time()
            while (time.time() - start_time) < 120 and robot.mission_active:
                robot.competition_map.print_mission_summary()
                time.sleep(10)
            
            print("Misión completada")
            
        elif choice == '2':
            print("Práctica con mapa de competencia...")
            robot.start_manual_control()
            
        elif choice == '3':
            print("Navegación reactiva por 30 segundos...")
            import time
            start_time = time.time()
            while (time.time() - start_time) < 30:
                action = robot.sensor_fusion.reactive_navigation()
                print(f"\rAcción: {action}", end="", flush=True)
                time.sleep(0.5)
            robot.sensor_fusion.motors.stop()
            print("\nNavegación completada")
        
    except KeyboardInterrupt:
        print("\nModo competencia interrumpido")
    except Exception as e:
        print(f"Error en modo competencia: {e}")
    finally:
        if 'robot' in locals():
            robot.cleanup()
        GPIO.cleanup()

def manual_control_mode():
    """Solo control manual sin competencia"""
    print("\n🎮 MODO CONTROL MANUAL")
    print("Control directo del robot (sin mapa de competencia)")
    
    try:
        robot = RobotController("manual_practice", enable_competition_mode=False)
        robot.start_system(sensor_update_rate=5)
        
        print("Control manual activado...")
        robot.start_manual_control()
        
    except KeyboardInterrupt:
        print("\nControl manual interrumpido")
    except Exception as e:
        print(f"Error en control manual: {e}")
    finally:
        if 'robot' in locals():
            robot.cleanup()
        GPIO.cleanup()

def session_analysis():
    """Análisis de sesiones guardadas"""
    print("\n📊 ANÁLISIS DE SESIONES")
    
    session_manager = SessionManager()
    visualizer = SimpleVisualizer()
    
    while True:
        print("\nOpciones de análisis:")
        print("1. Listar todas las sesiones")
        print("2. Analizar sesión específica")
        print("3. Comparar múltiples sesiones")
        print("4. Vista rápida de sesiones recientes")
        print("5. Exportar análisis")
        print("0. Volver al menú principal")
        
        choice = input("Elegir [0-5]: ").strip()
        
        if choice == '1':
            sessions = session_manager.list_sessions()
            if sessions:
                print(f"\n📁 {len(sessions)} sesiones encontradas:")
                for i, session in enumerate(sessions[:10], 1):  # Mostrar últimas 10
                    print(f"{i:2d}. {session['session_name']:20} | "
                          f"{session['duration']:6.0f}s | "
                          f"{session['entries']:5d} logs | "
                          f"{session['size_kb']:5.1f}KB")
            else:
                print("No hay sesiones guardadas")
        
        elif choice == '2':
            sessions = session_manager.list_sessions()
            if not sessions:
                print("No hay sesiones para analizar")
                continue
            
            print("\nSesiones disponibles:")
            for i, session in enumerate(sessions[:10], 1):
                print(f"{i}. {session['session_name']}")
            
            try:
                idx = int(input("Elegir sesión: ")) - 1
                if 0 <= idx < len(sessions):
                    session_data = session_manager.load_session(sessions[idx]['filename'])
                    if session_data:
                        visualizer.print_session_summary(session_data)
                        
                        if input("¿Ver mapa? [y/N]: ").lower() == 'y':
                            visualizer.draw_trajectory_map(session_data)
                        
                        if input("¿Ver análisis de sensores? [y/N]: ").lower() == 'y':
                            visualizer.show_sensor_analysis(session_data)
            except (ValueError, IndexError):
                print("Selección inválida")
        
        elif choice == '3':
            sessions = session_manager.list_sessions()
            if len(sessions) < 2:
                print("Se necesitan al menos 2 sesiones para comparar")
                continue
            
            print("Selecciona sesiones para comparar (máximo 5):")
            for i, session in enumerate(sessions[:10], 1):
                print(f"{i}. {session['session_name']}")
            
            try:
                indices = input("Números separados por comas: ").split(',')
                session_names = []
                
                for idx_str in indices[:5]:  # Máximo 5 sesiones
                    idx = int(idx_str.strip()) - 1
                    if 0 <= idx < len(sessions):
                        session_names.append(sessions[idx]['filename'])
                
                if len(session_names) >= 2:
                    sessions_data = []
                    for name in session_names:
                        data = session_manager.load_session(name)
                        if data:
                            sessions_data.append(data)
                    
                    if sessions_data:
                        visualizer.compare_sessions_visual(sessions_data)
                
            except ValueError:
                print("Formato inválido")
        
        elif choice == '4':
            from utils.visualization import quick_session_overview
            quick_session_overview()
        
        elif choice == '5':
            print("Funcionalidad de exportación por implementar")
        
        elif choice == '0':
            break

def data_management():
    """Gestión de datos y archivos"""
    print("\n🗂️  GESTIÓN DE DATOS")
    
    session_manager = SessionManager()
    
    while True:
        print("\nOpciones de gestión:")
        print("1. Listar archivos de datos")
        print("2. Limpiar sesiones antiguas")
        print("3. Verificar integridad de datos")
        print("4. Exportar datos")
        print("5. Importar configuración")
        print("0. Volver")
        
        choice = input("Elegir [0-5]: ").strip()
        
        if choice == '1':
            sessions = session_manager.list_sessions()
            total_size = sum(s['size_kb'] for s in sessions)
            print(f"\n📊 Resumen de datos:")
            print(f"Total sesiones: {len(sessions)}")
            print(f"Espacio ocupado: {total_size:.1f} KB")
            
            if sessions:
                oldest = min(sessions, key=lambda x: x['created'])
                newest = max(sessions, key=lambda x: x['created'])
                print(f"Sesión más antigua: {oldest['session_name']}")
                print(f"Sesión más reciente: {newest['session_name']}")
        
        elif choice == '2':
            print("Limpieza de sesiones antiguas...")
            days = input("Eliminar sesiones más antiguas que (días) [30]: ").strip()
            days = int(days) if days.isdigit() else 30
            
            keep = input("Mantener mínimo de sesiones [5]: ").strip()
            keep = int(keep) if keep.isdigit() else 5
            
            deleted = session_manager.cleanup_old_sessions(days_old=days, keep_minimum=keep)
            if deleted:
                print(f"Eliminadas {len(deleted)} sesiones")
            else:
                print("No hay sesiones que eliminar")
        
        elif choice == '3':
            print("Verificando integridad de datos...")
            sessions = session_manager.list_sessions()
            corrupted = []
            
            for session in sessions:
                if 'error' in session:
                    corrupted.append(session['filename'])
            
            if corrupted:
                print(f"Archivos con problemas: {len(corrupted)}")
                for file in corrupted:
                    print(f"  - {file}")
            else:
                print("Todos los archivos están íntegros")
        
        elif choice == '4':
            print("Exportación de datos por implementar")
        
        elif choice == '5':
            print("Importación de configuración por implementar")
        
        elif choice == '0':
            break

def system_configuration():
    """Configuración del sistema"""
    print("\n⚙️  CONFIGURACIÓN DEL SISTEMA")
    
    while True:
        print("\nOpciones de configuración:")
        print("1. Verificar configuración actual")
        print("2. Mostrar configuración de hardware")
        print("3. Verificar conexiones")
        print("4. Configurar directorios de datos")
        print("5. Exportar configuración")
        print("0. Volver")
        
        choice = input("Elegir [0-5]: ").strip()
        
        if choice == '1':
            print("\nVerificando configuración...")
            if validate_config():
                print("Configuración válida")
            else:
                print("Hay errores en la configuración")
        
        elif choice == '2':
            from config import MOTOR_PINS, ULTRASONIC_PINS, SENSOR_POSITIONS, ROBOT_DIMENSIONS
            
            print("\nConfiguración de hardware:")
            print("Motores L298N:")
            for pin_name, pin_num in MOTOR_PINS.items():
                print(f"  {pin_name}: GPIO {pin_num}")
            
            print("\nSensores HC-SR04:")
            for sensor_name, pins in ULTRASONIC_PINS.items():
                print(f"  {sensor_name}: Trig={pins['trig']}, Echo={pins['echo']}")
            
            print("\nDimensiones del robot:")
            for dim_name, value in ROBOT_DIMENSIONS.items():
                print(f"  {dim_name}: {value}")
        
        elif choice == '3':
            print("Verificación de conexiones por implementar")
        
        elif choice == '4':
            print("Creando directorios de datos...")
            create_data_directories()
            print("Directorios verificados")
        
        elif choice == '5':
            print("Exportación de configuración por implementar")
        
        elif choice == '0':
            break

def system_info():
    """Información del sistema"""
    print("\nℹ️  INFORMACIÓN DEL SISTEMA")
    
    print("\n=== ROBOT DE RESCATE 2025 ===")
    print("Hardware:")
    print("  - Raspberry Pi 4")
    print("  - 4x Sensores HC-SR04 (ultrasonido)")
    print("  - 1x MPU6050 (IMU)")
    print("  - 1x L298N (driver de motores)")
    print("  - 2x Motores DC N20")
    
    print("\nFuncionalidades:")
    print("  - Sensor fusion (ultrasonido + IMU + odometría)")
    print("  - Navegación autónoma")
    print("  - Detección y mapeo de obstáculos")
    print("  - Sistema de logging completo")
    print("  - Análisis y visualización de datos")
    
    print("\nCompetencia:")
    print("  - Robotics for Good Youth Challenge 2024-2025")
    print("  - Misión: Rescate en desastres naturales")
    print("  - Tiempo límite: 120 segundos")
    print("  - Campo: 117.1 x 114.3 cm")
    
    print("\nDesarrollado con:")
    print("  - Python 3")
    print("  - RPi.GPIO")
    print("  - Arquitectura modular")
    print("  - Control manual y autónomo")
    
    # Información del sistema
    try:
        import platform
        print(f"\nSistema operativo: {platform.system()} {platform.release()}")
        print(f"Arquitectura: {platform.machine()}")
        print(f"Python: {platform.python_version()}")
    except:
        pass
    
    input("\nPresiona Enter para continuar...")

def main():
    """Función principal"""
    # Argumentos de línea de comandos
    parser = argparse.ArgumentParser(description='Robot de Rescate 2025')
    parser.add_argument('--mode', choices=['experimental', 'competition', 'manual', 'calibration', 'movement'],
                       help='Modo directo de ejecución')
    parser.add_argument('--session', help='Nombre de sesión personalizado')
    parser.add_argument('--no-banner', action='store_true', help='No mostrar banner')
    
    args = parser.parse_args()
    
    # Mostrar banner
    if not args.no_banner:
        print_banner()
    
    # Verificar configuración
    if not validate_config():
        print("Error en configuración. Revisa config.py")
        return 1
    
    # Crear directorios
    create_data_directories()
    
    # Modo directo desde argumentos
    if args.mode:
        try:
            if args.mode == 'experimental':
                experimental_mode()
            elif args.mode == 'competition':
                competition_mode()
            elif args.mode == 'manual':
                manual_control_mode()
            elif args.mode == 'calibration':
                calibration_wizard()
            elif args.mode == 'movement':
                movement_recorder_menu()
            return 0
        except Exception as e:
            print(f"Error en modo {args.mode}: {e}")
            return 1
    
    # Menú interactivo
    try:
        while True:
            show_main_menu()
            choice = input("Elegir opción [0-9]: ").strip()

            if choice == '1':
                experimental_mode()
            elif choice == '2':
                competition_mode()
            elif choice == '3':
                manual_control_mode()
            elif choice == '4':
                calibration_wizard()
            elif choice == '5':
                movement_recorder_menu()
            elif choice == '6':
                session_analysis()
            elif choice == '7':
                data_management()
            elif choice == '8':
                system_configuration()
            elif choice == '9':
                system_info()
            elif choice == '0':
                print("¡Hasta luego!")
                break
            else:
                print("Opción inválida. Intenta de nuevo.")
    
    except KeyboardInterrupt:
        print("\nPrograma interrumpido por el usuario")
    
    except Exception as e:
        print(f"Error inesperado: {e}")
        return 1
    
    finally:
        # Limpieza final
        try:
            GPIO.cleanup()
        except:
            pass
    
    return 0

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)