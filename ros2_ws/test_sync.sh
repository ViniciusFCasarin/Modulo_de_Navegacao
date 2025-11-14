#!/bin/bash
# Script de teste das correções de sincronização
# Uso: ./test_sync.sh

echo "🔍 TESTE DE SINCRONIZAÇÃO E MOVIMENTO CIRCULAR"
echo "=============================================="
echo ""

# Cores
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 1. Verificar compilação
echo -e "${BLUE}1. Verificando compilação...${NC}"
if [ -f "install/robot_bringup/lib/robot_bringup/odom_monitor.py" ]; then
    echo -e "  ${GREEN}✓${NC} odom_monitor.py instalado"
else
    echo -e "  ${RED}✗${NC} odom_monitor.py NÃO encontrado"
    echo "    Execute: colcon build --packages-select robot_bringup"
fi
echo ""

# 2. Verificar parâmetros SLAM
echo -e "${BLUE}2. Verificando parâmetros SLAM...${NC}"

check_param() {
    param=$1
    expected=$2
    actual=$(grep "$param:" src/robot_bringup/config/mapper_params_online_async.yaml | awk '{print $2}')
    if [ "$actual" == "$expected" ]; then
        echo -e "  ${GREEN}✓${NC} $param: $actual"
    else
        echo -e "  ${YELLOW}⚠${NC} $param: $actual (esperado: $expected)"
    fi
}

check_param "transform_timeout" "0.5"
check_param "minimum_time_interval" "0.1"
check_param "map_update_interval" "2.0"
check_param "minimum_travel_distance" "0.1"
check_param "minimum_travel_heading" "0.1"
echo ""

# 3. Verificar velocidades do cmd_vel
echo -e "${BLUE}3. Verificando velocidades de movimento circular...${NC}"

linear_x=$(grep "linear: {x:" src/robot_bringup/launch/cmd_vel.launch.py | grep -oP 'x: \K[0-9.]+' | head -1)
angular_z=$(grep "angular: {x:" src/robot_bringup/launch/cmd_vel.launch.py | grep -oP 'z: \K[0-9.]+' | head -1)

if [ -n "$linear_x" ] && [ -n "$angular_z" ]; then
    echo -e "  ${GREEN}✓${NC} linear.x: $linear_x m/s"
    echo -e "  ${GREEN}✓${NC} angular.z: $angular_z rad/s"
    
    # Calcular raio (em bash, precisamos usar bc)
    if command -v bc &> /dev/null; then
        radius=$(echo "scale=2; $linear_x / $angular_z" | bc)
        period=$(echo "scale=1; 6.28318 / $angular_z" | bc)
        echo -e "  ${BLUE}→${NC} Raio do círculo: ~${radius} metros"
        echo -e "  ${BLUE}→${NC} Tempo por volta: ~${period} segundos"
    fi
else
    echo -e "  ${RED}✗${NC} Não foi possível ler velocidades"
fi
echo ""

# 4. Instruções de teste
echo "=============================================="
echo -e "${GREEN}📋 INSTRUÇÕES DE TESTE${NC}"
echo "=============================================="
echo ""

echo -e "${YELLOW}Teste 1: Movimento Circular${NC}"
echo "  Terminal 1: ros2 launch robot_bringup gazebo_world.launch.py"
echo "  Terminal 2: ros2 launch robot_bringup mapping.launch.py"
echo "  Terminal 3: ros2 launch robot_bringup cmd_vel.launch.py"
echo ""
echo "  Esperado:"
echo "    ✓ Robô anda em círculo de ~1 metro"
echo "    ✓ Completa volta em ~21 segundos"
echo "    ✓ Gazebo e RViz sincronizados"
echo ""

echo -e "${YELLOW}Teste 2: Monitor de Colisão${NC}"
echo "  1. Inicie o sistema (3 terminais acima)"
echo "  2. Deixe o robô colidir com uma parede"
echo "  3. Observe o terminal do mapping.launch.py"
echo ""
echo "  Esperado:"
echo "    ✓ Aviso: 'Possível colisão detectada!'"
echo "    ✓ Robô para no Gazebo"
echo ""

echo -e "${YELLOW}Teste 3: Sincronização${NC}"
echo "  Terminal extra: ros2 topic hz /odom"
echo ""
echo "  Esperado:"
echo "    ✓ Frequência: ~30 Hz"
echo "    ✓ Sem delays ou travamentos"
echo ""

echo "=============================================="
echo -e "${GREEN}🚀 Sistema pronto para teste!${NC}"
echo ""
echo "Para iniciar testes, abra 3 terminais e execute os comandos acima."
echo ""

# 5. Verificar se ROS2 está disponível
if command -v ros2 &> /dev/null; then
    echo -e "${BLUE}Dica:${NC} Para monitorar em tempo real:"
    echo "  ros2 topic echo /odom --field pose.pose.position"
    echo "  ros2 topic echo /cmd_vel"
    echo "  ros2 topic hz /odom"
else
    echo -e "${YELLOW}⚠${NC} ROS2 não encontrado. Execute: source install/setup.bash"
fi

echo ""
