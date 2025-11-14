# 🤖 Sistema de SLAM com ROS2 + Gazebo Fortress

## 📋 Como Executar o Sistema Completo

### ✅ **MÉTODO CORRETO (2 terminais)**

#### **Terminal 1 - Gazebo com modelos:**
```bash
source /home/vinicius/ros2_ws/install/setup.bash
ros2 launch robot_bringup gazebo_world.launch.py
```

**OU use o método manual com variável de ambiente:**
```bash
source /home/vinicius/ros2_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=/home/vinicius/ros2_ws/install/robot_bringup/share/robot_bringup/models:$IGN_GAZEBO_RESOURCE_PATH
ign gazebo src/robot_bringup/worlds/main.sdf
```

#### **Terminal 2 - Mapping + RViz:**
```bash
source /home/vinicius/ros2_ws/install/setup.bash
ros2 launch robot_bringup mapping.launch.py
```

---

## ❌ **NÃO EXECUTE:**
```bash
# ❌ NÃO EXECUTE MAIS ISSO:
python3 src/robot_bringup/scripts/odom_tf_publisher.py
```

**Por quê?**  
O plugin DiffDrive do Gazebo já publica a TF `odom` → `base_link`. Executar o script Python causa **conflito de TF** (warnings `TF_OLD_DATA`).

---

## 🔍 Diagnóstico

Para verificar se tudo está funcionando:
```bash
source /home/vinicius/ros2_ws/install/setup.bash
bash /home/vinicius/ros2_ws/src/robot_bringup/scripts/diagnose_slam.sh
```

---

## 🎯 O que Deve Acontecer

### ✅ **Gazebo:**
- Mundo carregado com plano de chão
- Parede azul (wall) na posição (5, 0, 0)
- Robô azul com rodas pretas e caster verde

### ✅ **RViz:**
- Pontos vermelhos do LIDAR detectando a parede
- Mapa sendo construído em tempo real (grid cinza/preto)
- Setas do TF tree conectadas corretamente
- Robô se movendo quando você publica `/cmd_vel`

---

## 🧪 Testar LIDAR

Verifique se o LIDAR está detectando a parede:
```bash
ros2 topic echo /scan --once | grep -A 10 "ranges:"
```

**Esperado:** Alguns valores numéricos (distâncias em metros) ao invés de apenas `.inf`

---

## 🗺️ TF Tree Esperada

```
map (criado pelo SLAM Toolbox)
 └─ odom (frame de odometria)
     └─ base_link (base do robô)
         └─ chassis (corpo do robô)
             ├─ lidar_link (sensor LIDAR)
             ├─ left_wheel (roda esquerda)
             └─ right_wheel (roda direita)
```

---

## 🐛 Troubleshooting

### **Problema: LIDAR retorna apenas `.inf`**
**Solução:** A parede não foi carregada. Certifique-se de:
1. Usar `gazebo_world.launch.py` OU
2. Exportar `GZ_SIM_RESOURCE_PATH` antes de rodar o Gazebo

### **Problema: Warnings `TF_OLD_DATA`**
**Solução:** Não execute `odom_tf_publisher.py`. Mate todos os processos e reinicie.

### **Problema: Frame `map` não existe**
**Solução:** O SLAM Toolbox precisa de dados válidos do LIDAR. Verifique se a parede está sendo detectada.

---

## 📊 Comandos Úteis

```bash
# Ver todos os nós ativos
ros2 node list

# Ver todos os tópicos
ros2 topic list

# Ver TF tree
ros2 run tf2_tools view_frames

# Echo em tópicos importantes
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /map

# Publicar velocidade manualmente
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" --once
```

---

## 📁 Estrutura de Arquivos Importantes

```
robot_bringup/
├── worlds/
│   ├── main.sdf          # Mundo Gazebo
│   ├── robot.model.sdf   # Modelo do robô
│   └── wall.model.sdf    # Modelo da parede
├── models/
│   ├── Robot/
│   │   └── model.sdf     # Modelo instalável
│   └── wall/
│       └── model.sdf     # Modelo instalável
├── config/
│   ├── mapper_params_online_async.yaml  # Parâmetros SLAM
│   └── mapping_config.rviz              # Config RViz
├── launch/
│   ├── gazebo_world.launch.py   # ✅ NOVO: Inicia Gazebo com modelos
│   └── mapping.launch.py        # Sistema de mapeamento
└── scripts/
    ├── diagnose_slam.sh         # Script de diagnóstico
    └── odom_tf_publisher.py     # ❌ NÃO USE MAIS
```
