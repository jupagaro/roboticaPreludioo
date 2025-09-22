#!/bin/bash
# install.sh - Script de instalación para Robot de Rescate 2025
# Para Raspberry Pi OS (Raspbian) Headless

set -e  # Salir si cualquier comando falla

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

echo_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

echo_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

echo_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Banner
echo_info "=============================================="
echo_info "    Robot de Rescate 2025 - Instalación"
echo_info "       Raspberry Pi 4 Setup Script"
echo_info "=============================================="

# Verificar que estamos en Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo_warning "No se detectó Raspberry Pi. Continuando de todas formas..."
fi

# Verificar permisos de sudo
if ! sudo -n true 2>/dev/null; then
    echo_error "Este script necesita permisos sudo. Ejecuta:"
    echo "sudo visudo"
    echo "Y añade: $(whoami) ALL=(ALL) NOPASSWD: ALL"
    exit 1
fi

echo_info "Usuario actual: $(whoami)"
echo_info "Directorio: $(pwd)"

# Actualizar sistema
echo_info "Actualizando sistema..."
sudo apt update -y
sudo apt upgrade -y

# Instalar dependencias del sistema
echo_info "Instalando dependencias del sistema..."
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    i2c-tools \
    git \
    nano \
    htop \
    screen

# Verificar Python 3
if ! command -v python3 &> /dev/null; then
    echo_error "Python 3 no está instalado"
    exit 1
fi

python_version=$(python3 --version)
echo_success "Python instalado: $python_version"

# Habilitar I2C y GPIO
echo_info "Configurando interfaces hardware..."

# Backup del config actual
sudo cp /boot/config.txt /boot/config.txt.backup

# Habilitar I2C
if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
    echo_info "I2C habilitado en /boot/config.txt"
else
    echo_info "I2C ya estaba habilitado"
fi

# Habilitar SPI (por si se necesita)
if ! grep -q "dtparam=spi=on" /boot/config.txt; then
    echo "dtparam=spi=on" | sudo tee -a /boot/config.txt
    echo_info "SPI habilitado en /boot/config.txt"
fi

# Añadir usuario al grupo i2c
sudo usermod -a -G i2c $(whoami)
echo_info "Usuario añadido al grupo i2c"

# Verificar si existe el directorio del proyecto
PROJECT_DIR="robot_rescate"

if [ ! -d "$PROJECT_DIR" ]; then
    echo_info "Creando directorio del proyecto..."
    mkdir -p $PROJECT_DIR
else
    echo_info "Directorio del proyecto ya existe"
fi

cd $PROJECT_DIR

# Crear entorno virtual
echo_info "Creando entorno virtual de Python..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo_success "Entorno virtual creado"
else
    echo_info "Entorno virtual ya existe"
fi

# Activar entorno virtual
echo_info "Activando entorno virtual..."
source venv/bin/activate

# Actualizar pip en el entorno virtual
echo_info "Actualizando pip..."
pip install --upgrade pip

# Instalar dependencias si existe requirements.txt
if [ -f "requirements.txt" ]; then
    echo_info "Instalando dependencias desde requirements.txt..."
    pip install -r requirements.txt
    echo_success "Dependencias instaladas"
else
    echo_info "Instalando dependencias básicas..."
    pip install RPi.GPIO smbus2 numpy
    
    # Crear requirements.txt básico
    echo_info "Creando requirements.txt..."
    cat > requirements.txt << EOF
# Robot de Rescate 2025 - Dependencias básicas
RPi.GPIO==0.7.1
smbus2==0.4.2
numpy>=1.21.0,<2.0.0
EOF
    echo_success "requirements.txt creado"
fi

# Crear directorios necesarios
echo_info "Creando estructura de directorios..."
mkdir -p data/sessions
mkdir -p data/calibration  
mkdir -p data/maps
mkdir -p hardware
mkdir -p utils

echo_success "Directorios creados"

# Crear script de activación fácil
echo_info "Creando script de activación..."
cat > activate.sh << 'EOF'
#!/bin/bash
# Script para activar el entorno virtual del robot

cd "$(dirname "$0")"
echo "Activando entorno virtual del robot..."
source venv/bin/activate
echo "Entorno activado. Para ejecutar el robot usa:"
echo "  python3 main.py"
echo ""
echo "Para salir del entorno virtual usa:"
echo "  deactivate"
exec bash
EOF

chmod +x activate.sh
echo_success "Script activate.sh creado"

# Crear script de ejecución rápida
cat > run_robot.sh << 'EOF'
#!/bin/bash
# Script para ejecutar el robot directamente

cd "$(dirname "$0")"
source venv/bin/activate

echo "Iniciando Robot de Rescate 2025..."
python3 main.py "$@"
EOF

chmod +x run_robot.sh
echo_success "Script run_robot.sh creado"

# Verificar I2C
echo_info "Verificando configuración I2C..."
if command -v i2cdetect &> /dev/null; then
    echo_info "Escaneando dispositivos I2C..."
    i2c_devices=$(i2cdetect -y 1 2>/dev/null | grep -E '[0-9a-f]{2}' | wc -l)
    if [ $i2c_devices -gt 0 ]; then
        echo_success "Dispositivos I2C detectados: $i2c_devices"
        echo_info "Ejecuta 'i2cdetect -y 1' para ver detalles"
    else
        echo_warning "No se detectaron dispositivos I2C (normal si no están conectados)"
    fi
else
    echo_warning "i2cdetect no disponible"
fi

# Crear archivo de configuración local
echo_info "Creando configuración local..."
cat > local_config.py << 'EOF'
# local_config.py - Configuración específica de esta instalación
# Este archivo será importado por config.py

# Configuración específica del hardware instalado
LOCAL_HARDWARE = {
    'i2c_bus': 1,
    'mpu6050_present': True,
    'motor_driver': 'L298N',
    'ultrasonic_count': 4
}

# Rutas locales
LOCAL_PATHS = {
    'logs': '/home/pi/robot_logs',
    'backups': '/home/pi/robot_backups'
}

# Configuración de red (si se necesita)
LOCAL_NETWORK = {
    'enable_remote_access': False,
    'telemetry_port': 8080
}
EOF

echo_success "Configuración local creada"

# Crear servicio systemd opcional
echo_info "¿Quieres crear un servicio systemd para inicio automático? [y/N]"
read -r create_service

if [[ $create_service =~ ^[Yy]$ ]]; then
    echo_info "Creando servicio systemd..."
    
    sudo tee /etc/systemd/system/robot-rescate.service > /dev/null << EOF
[Unit]
Description=Robot de Rescate 2025
After=network.target

[Service]
Type=simple
User=$(whoami)
WorkingDirectory=$(pwd)
Environment=PATH=$(pwd)/venv/bin
ExecStart=$(pwd)/venv/bin/python main.py --mode competition
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    echo_success "Servicio systemd creado"
    echo_info "Para habilitar: sudo systemctl enable robot-rescate"
    echo_info "Para iniciar: sudo systemctl start robot-rescate"
fi

# Test básico de GPIO
echo_info "Probando acceso GPIO..."
python3 << 'EOF'
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.cleanup()
    print("GPIO: OK")
except Exception as e:
    print(f"GPIO: ERROR - {e}")
EOF

# Test básico de I2C
echo_info "Probando acceso I2C..."
python3 << 'EOF'
try:
    import smbus
    bus = smbus.SMBus(1)
    print("I2C: OK")
except Exception as e:
    print(f"I2C: ERROR - {e}")
EOF

# Información final
echo_success "=============================================="
echo_success "     INSTALACIÓN COMPLETADA"
echo_success "=============================================="
echo_info ""
echo_info "Para usar el robot:"
echo_info "1. Activa el entorno virtual:"
echo_info "   ./activate.sh"
echo_info ""
echo_info "2. O ejecuta directamente:"
echo_info "   ./run_robot.sh"
echo_info ""
echo_info "3. Para diferentes modos:"
echo_info "   ./run_robot.sh --mode experimental"
echo_info "   ./run_robot.sh --mode competition"
echo_info "   ./run_robot.sh --mode calibration"
echo_info ""
echo_warning "IMPORTANTE: Es necesario reiniciar para aplicar cambios de I2C"
echo_warning "sudo reboot"
echo_info ""
echo_info "Archivos creados en: $(pwd)"
echo_info "Logs del sistema: journalctl -u robot-rescate -f"
echo_success "=============================================="

# Preguntar si reiniciar ahora
echo_info "¿Reiniciar ahora para aplicar configuración I2C? [y/N]"
read -r reboot_now

if [[ $reboot_now =~ ^[Yy]$ ]]; then
    echo_info "Reiniciando sistema..."
    sudo reboot
else
    echo_warning "Recuerda reiniciar antes de usar I2C: sudo reboot"
fi