# 🔧 Correção: Direção dos Bridges ROS2 ↔ Gazebo

## 🐛 Problema Identificado

### Sintoma
- ✅ RViz mostrando movimento correto
- ❌ Gazebo **NÃO** respondendo aos comandos `/cmd_vel`
- ❌ Robô parado no Gazebo enquanto se move no RViz

### Causa Raiz
**Bridge bidirecional incorreto** para `/cmd_vel`:
```python
# ❌ ERRADO - Bidirecional (@...@)
'/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
```

Isso criava um **loop de feedback**:
1. ROS2 publica `/cmd_vel` → Gazebo
2. Gazebo "reflete" de volta → ROS2
3. Conflito! Gazebo ignora o comando

---

## ✅ Solução Aplicada

### Sintaxe Correta dos Bridges

#### 1. `/cmd_vel` - ROS2 → Gazebo (Unidirecional)
```python
# ✅ CORRETO - ROS2 → Gazebo apenas (]...)
'/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
#                                  ↑
#                             Sinal de fechamento ]
```

#### 2. `/odom` - Gazebo → ROS2 (Unidirecional)
```python
# ✅ CORRETO - Gazebo → ROS2 apenas ([...)
'/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
#                            ↑
#                       Sinal de abertura [
```

#### 3. `/tf` - Gazebo → ROS2 (Unidirecional)
```python
# ✅ CORRETO - Gazebo → ROS2 apenas ([...)
'/model/Robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
#                                       ↑
#                                  Sinal de abertura [
```

---

## 📋 Arquivos Corrigidos

### 1. `mapping.launch.py`
```python
cmd_vel_odom_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='ros_gz_bridge',
    arguments=[
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',      # ROS2 → Gazebo ✅
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',        # Gazebo → ROS2 ✅
        '/model/Robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',  # Gazebo → ROS2 ✅
    ],
    remappings=[
        ('/model/Robot/tf', '/tf'),
    ],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
)
```

### 2. `cmd_vel.launch.py`
```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='cmd_vel_auto_bridge',
    arguments=[
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',  # ROS2 → Gazebo ✅
    ],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
)
```

### 3. `teleop_keyboard.launch.py`
```python
bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='cmd_vel_bridge',
    arguments=[
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',  # ROS2 → Gazebo ✅
    ],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
)
```

---

## 📊 Referência: Sintaxe do ros_gz_bridge

| Sintaxe | Direção | Exemplo | Uso |
|---------|---------|---------|-----|
| `@...@` | **Bidirecional** | `@geometry_msgs/msg/Twist@gz.msgs.Twist` | ⚠️ Evitar (causa loops) |
| `]...` | **ROS2 → Gazebo** | `]gz.msgs.Twist` | Comandos (cmd_vel) ✅ |
| `[...` | **Gazebo → ROS2** | `[gz.msgs.Odometry` | Sensores (odom, lidar, tf) ✅ |

### Regra Geral
- **Comandos/Atuadores** (cmd_vel): ROS2 → Gazebo (`]`)
- **Sensores/Estados** (odom, lidar, tf, joint_states): Gazebo → ROS2 (`[`)
- **Bidirecional** (`@`): Apenas quando realmente necessário (raro)

---

## 🧪 Testes de Validação

### 1. Verificar bridges ativos
```bash
ros2 node list | grep bridge
# Esperado: ros_gz_bridge, lidar_bridge, joint_state_bridge
```

### 2. Verificar tópico /cmd_vel
```bash
ros2 topic info /cmd_vel
```
**Esperado:**
```
Publishers: 1 (teleop_twist_keyboard ou cmd_vel publisher)
Subscribers: 1 (ros_gz_bridge)
```

### 3. Testar movimento
```bash
# Terminal 3
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```
**Esperado:**
- ✅ Robô se move no **Gazebo**
- ✅ Robô se move no **RViz**
- ✅ Ambos **sincronizados**

### 4. Monitorar mensagens
```bash
# Terminal separado
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

---

## 🔍 TF Tree Atual (Validado)

```
map (SLAM Toolbox)
 └─ odom (DiffDrive + Bridge)
     └─ base_link
         └─ chassis (robot_state_publisher)
             ├─ lidar_link
             ├─ left_wheel
             └─ right_wheel
```

**Status:** ✅ Sem duplicações, sem conflitos

---

## ✅ Checklist de Verificação

- [x] Bridge `/cmd_vel` unidirecional (ROS2 → Gazebo) `]`
- [x] Bridge `/odom` unidirecional (Gazebo → ROS2) `[`
- [x] Bridge `/tf` unidirecional (Gazebo → ROS2) `[`
- [x] Compilação sem erros
- [x] TF tree sem duplicações
- [ ] **Teste prático:** Movimento sincronizado Gazebo + RViz

---

## 🚀 Próximo Teste Recomendado

```bash
# Terminal 1
ros2 launch robot_bringup gazebo_world.launch.py

# Terminal 2
ros2 launch robot_bringup mapping.launch.py

# Terminal 3 - Teste de movimento
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

**Observe:**
1. ✅ Robô se move no **Gazebo** (janela 3D)
2. ✅ Robô se move no **RViz** (visualização)
3. ✅ Ambos **perfeitamente sincronizados**

Se ainda houver dessincronização, verificar:
- `use_sim_time=true` em todos os nós
- Clock do Gazebo publicando corretamente

---

**Data:** 10/11/2025 15:30
**Status:** ✅ Correções aplicadas e compiladas
**Próximo passo:** Teste prático de movimento
