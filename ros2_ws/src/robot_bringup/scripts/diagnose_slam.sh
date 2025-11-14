#!/bin/bash

echo "======================================"
echo "🔍 DIAGNÓSTICO DO SISTEMA SLAM"
echo "======================================"
echo ""

echo "1️⃣ Verificando nós ROS2 ativos..."
ros2 node list
echo ""

echo "2️⃣ Verificando tópicos ROS2..."
ros2 topic list | grep -E "(map|lidar|scan|odom|cmd_vel)"
echo ""

echo "3️⃣ Informações do tópico /scan..."
ros2 topic info /scan
echo ""

echo "4️⃣ Verificando frame_id do LIDAR (primeiras linhas)..."
timeout 3 ros2 topic echo /scan --once | head -20
echo ""

echo "5️⃣ Verificando tópico /map..."
timeout 3 ros2 topic echo /map --once | head -10
echo ""

echo "6️⃣ Verificando se SLAM Toolbox está rodando..."
ros2 node list | grep slam
if [ $? -eq 0 ]; then
    echo "✅ SLAM Toolbox encontrado!"
    echo ""
    echo "7️⃣ Parâmetros do SLAM Toolbox:"
    ros2 param list /slam_toolbox | grep -E "(frame|topic)"
else
    echo "❌ SLAM Toolbox NÃO está rodando!"
fi
echo ""

echo "8️⃣ Verificando TF tree (gerando frames.pdf)..."
ros2 run tf2_tools view_frames
echo "✅ Arquivo frames.pdf gerado. Abra para ver a árvore TF."
echo ""

echo "9️⃣ Testando transformada map -> base_link..."
timeout 3 ros2 run tf2_ros tf2_echo map base_link
echo ""

echo "🔟 Testando transformada odom -> base_link..."
timeout 3 ros2 run tf2_ros tf2_echo odom base_link
echo ""

echo "======================================"
echo "✅ Diagnóstico completo!"
echo "======================================"
