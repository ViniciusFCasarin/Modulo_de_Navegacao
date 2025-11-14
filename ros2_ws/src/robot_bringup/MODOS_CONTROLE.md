# 🎮 Modos de Controle do Robô

Este documento descreve os diferentes modos de controle disponíveis para o robô.

## 📋 Pré-requisito: Sistema Básico

Sempre inicie o Gazebo e o sistema de mapeamento primeiro:

```bash
# Terminal 1 - Gazebo
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup gazebo_world.launch.py

# Terminal 2 - SLAM e RViz
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup mapping.launch.py
```

---

## 🕹️ Modo 1: Controle via Teclado Gazebo (Nativo)

**Descrição:** Controle direto usando as setas do teclado na janela do Gazebo.

**Como usar:**
1. Garanta que o sistema básico está rodando (Terminais 1 e 2)
2. Clique na janela do **Gazebo** (interface gráfica)
3. Use as seguintes teclas:

| Tecla | Ação |
|-------|------|
| `↑` (Seta para cima) | Avançar |
| `↓` (Seta para baixo) | Recuar |
| `←` (Seta esquerda) | Girar esquerda |
| `→` (Seta direita) | Girar direita |
| `B` | Freiar/Parar |

**Vantagens:**
- ✅ Não requer terminal adicional
- ✅ Controle imediato e intuitivo
- ✅ Já configurado no `model.sdf`

**Limitações:**
- ⚠️ Requer foco na janela do Gazebo
- ⚠️ Pode conflitar se outros publishers de `/cmd_vel` estiverem ativos

---

## ⌨️ Modo 2: Controle via Teclado ROS2 (teleop_twist_keyboard)

**Descrição:** Controle através do terminal usando o pacote `teleop_twist_keyboard`.

**Como usar:**
```bash
# Terminal 3 - Teleop Keyboard
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup teleop_keyboard.launch.py
```

**Controles:**

| Tecla | Ação |
|-------|------|
| `i` | Avançar |
| `k` | Parar |
| `,` | Recuar |
| `j` | Girar esquerda |
| `l` | Girar direita |
| `u` | Diagonal: frente + esquerda |
| `o` | Diagonal: frente + direita |
| `m` | Diagonal: trás + esquerda |
| `.` | Diagonal: trás + direita |
| `q` | Aumentar velocidade |
| `z` | Diminuir velocidade |
| `ESPAÇO` | Parar imediatamente |

**Vantagens:**
- ✅ Controle mais preciso
- ✅ Ajuste de velocidade em tempo real
- ✅ Feedback no terminal
- ✅ Funciona mesmo sem janela Gazebo em foco

**Pré-requisitos:**
```bash
# Instalar pacote (se necessário)
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install xterm  # Para janela separada
```

---

## 🤖 Modo 3: Movimento Autônomo Contínuo

**Descrição:** O robô se move automaticamente em círculo (útil para testes de mapeamento).

**Como usar:**
```bash
# Terminal 3 - Movimento Automático
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup cmd_vel.launch.py
```

**Comportamento:**
- Velocidade linear: `0.5 m/s` (avançar)
- Velocidade angular: `0.05 rad/s` (girar levemente)
- Resultado: O robô se move em círculo

**Para parar:**
```bash
# Pressione Ctrl+C no Terminal 3
```

**Para ajustar velocidades:**
Edite o arquivo `src/robot_bringup/launch/cmd_vel.launch.py` linha ~47:
```python
'{linear: {x: 0.5, ...}, angular: {..., z: 0.05}}'
#              ^^^                        ^^^^
#          linear (m/s)              angular (rad/s)
```

**Vantagens:**
- ✅ Útil para mapeamento automático
- ✅ Não requer intervenção manual
- ✅ Velocidades configuráveis

**Limitações:**
- ⚠️ Movimento pré-definido (não adaptativo)
- ⚠️ Requer `Ctrl+C` para parar

---

## 🔧 Modo 4: Controle Manual via Comando ROS2

**Descrição:** Publicar comandos de velocidade manualmente via terminal.

**Como usar:**
```bash
# Terminal 3 - Comando Manual
source ~/ros2_ws/install/setup.bash

# Avançar
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Girar esquerda
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Parar
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

**Vantagens:**
- ✅ Controle preciso de valores
- ✅ Útil para debug e testes
- ✅ Scripting e automação

**Limitações:**
- ⚠️ Menos prático para controle contínuo
- ⚠️ Requer novos comandos para cada mudança

---

## ⚠️ Conflitos de Controle

**IMPORTANTE:** Apenas **UM** modo de controle deve estar ativo por vez!

Se múltiplos publishers de `/cmd_vel` estiverem ativos simultaneamente:
- O comportamento será imprevisível
- Comandos conflitantes podem causar movimento errático

**Solução:**
1. Pare todos os launchers de controle (`Ctrl+C`)
2. Escolha apenas um modo
3. Relance apenas o modo desejado

---

## 🧪 Verificação de Sistema

**Verificar tópico `/cmd_vel`:**
```bash
# Ver publishers ativos
ros2 topic info /cmd_vel

# Monitorar mensagens
ros2 topic echo /cmd_vel
```

**Verificar bridges ativos:**
```bash
ros2 node list | grep bridge
```

---

## 📊 Recomendações de Uso

| Cenário | Modo Recomendado |
|---------|------------------|
| Mapeamento manual rápido | Teclado Gazebo (Modo 1) |
| Controle preciso/ajustável | Teleop Keyboard (Modo 2) |
| Testes de mapeamento automático | Movimento Autônomo (Modo 3) |
| Debug/desenvolvimento | Comando Manual (Modo 4) |

---

## 🚀 Quick Start

**Para controle via teclado ROS2 (mais comum):**
```bash
# Terminal 1
ros2 launch robot_bringup gazebo_world.launch.py

# Terminal 2
ros2 launch robot_bringup mapping.launch.py

# Terminal 3
ros2 launch robot_bringup teleop_keyboard.launch.py
```

Agora use as teclas `i/j/k/l` para controlar o robô! 🎮
