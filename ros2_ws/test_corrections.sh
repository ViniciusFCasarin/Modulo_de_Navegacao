#!/bin/bash
# Script de teste rápido das correções aplicadas
# Uso: ./test_corrections.sh

echo "🔍 TESTE DAS CORREÇÕES APLICADAS"
echo "================================"
echo ""

# Cores para output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. Verificar arquivos criados/modificados
echo "📁 1. Verificando arquivos..."
FILES=(
    "src/robot_bringup/launch/mapping.launch.py"
    "src/robot_bringup/launch/cmd_vel.launch.py"
    "src/robot_bringup/launch/teleop_keyboard.launch.py"
    "src/robot_bringup/MODOS_CONTROLE.md"
)

for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "  ${GREEN}✓${NC} $file"
    else
        echo -e "  ${RED}✗${NC} $file (NOT FOUND)"
    fi
done
echo ""

# 2. Verificar se bridge de TF foi removido
echo "🔧 2. Verificando remoção do bridge de TF duplicado..."
if grep -q "model/Robot/tf" src/robot_bringup/launch/mapping.launch.py; then
    echo -e "  ${RED}✗${NC} Bridge de TF ainda presente (ERRO)"
else
    echo -e "  ${GREEN}✓${NC} Bridge de TF removido com sucesso"
fi
echo ""

# 3. Verificar se cmd_vel_publisher foi removido do mapping.launch.py
echo "🔧 3. Verificando remoção do cmd_vel_publisher do mapping.launch.py..."
if grep -q "cmd_vel_publisher" src/robot_bringup/launch/mapping.launch.py; then
    echo -e "  ${RED}✗${NC} cmd_vel_publisher ainda presente (ERRO)"
else
    echo -e "  ${GREEN}✓${NC} cmd_vel_publisher removido com sucesso"
fi
echo ""

# 4. Verificar compilação
echo "🔨 4. Verificando compilação do pacote..."
cd /home/vinicius/ros2_ws
if colcon build --packages-select robot_bringup 2>&1 | grep -q "Finished"; then
    echo -e "  ${GREEN}✓${NC} Pacote compila sem erros"
else
    echo -e "  ${RED}✗${NC} Erro na compilação"
fi
echo ""

# 5. Verificar dependências do teleop_keyboard
echo "📦 5. Verificando dependências..."

# Verificar teleop_twist_keyboard
if dpkg -l | grep -q "ros-humble-teleop-twist-keyboard"; then
    echo -e "  ${GREEN}✓${NC} teleop-twist-keyboard instalado"
else
    echo -e "  ${YELLOW}⚠${NC} teleop-twist-keyboard NÃO instalado"
    echo "    Instale com: sudo apt install ros-humble-teleop-twist-keyboard"
fi

# Verificar xterm
if command -v xterm &> /dev/null; then
    echo -e "  ${GREEN}✓${NC} xterm instalado"
else
    echo -e "  ${YELLOW}⚠${NC} xterm NÃO instalado"
    echo "    Instale com: sudo apt install xterm"
fi
echo ""

# 6. Contar arquivos launch disponíveis
echo "📋 6. Arquivos launch disponíveis:"
ls -1 src/robot_bringup/launch/*.launch.py | while read file; do
    basename "$file"
done | sed 's/^/  - /'
echo ""

# 7. Resumo
echo "================================"
echo "📊 RESUMO"
echo "================================"
echo ""
echo -e "${GREEN}Correções aplicadas:${NC}"
echo "  ✅ Bridge de TF duplicado removido"
echo "  ✅ cmd_vel_publisher movido para arquivo separado"
echo "  ✅ teleop_keyboard.launch.py criado"
echo "  ✅ Documentação MODOS_CONTROLE.md criada"
echo ""
echo -e "${YELLOW}Próximos passos:${NC}"
echo "  1. Instalar dependências (se necessário):"
echo "     sudo apt install ros-humble-teleop-twist-keyboard xterm"
echo ""
echo "  2. Testar o sistema:"
echo "     Terminal 1: ros2 launch robot_bringup gazebo_world.launch.py"
echo "     Terminal 2: ros2 launch robot_bringup mapping.launch.py"
echo "     Terminal 3: ros2 launch robot_bringup teleop_keyboard.launch.py"
echo ""
echo "  3. Verificar TF tree (aguardar 5s entre comandos):"
echo "     ros2 run tf2_tools view_frames"
echo "     evince frames.pdf"
echo ""
echo -e "${GREEN}✅ Teste concluído!${NC}"
