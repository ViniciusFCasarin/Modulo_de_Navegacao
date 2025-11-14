# ⚡ Guia Rápido de Comandos

## 🚀 Inicialização Básica (3 Terminais)

### Terminal 1 - Gazebo
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_bringup gazebo_world.launch.py
```

### Terminal 2 - SLAM + RViz
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_bringup mapping.launch.py
```

### Terminal 3 - Controle (escolha um):

**Opção A: Controle via Teclado (Recomendado)**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_bringup teleop_keyboard.launch.py
# Use teclas: i/j/k/l para controlar
```

**Opção B: Movimento Automático**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_bringup cmd_vel.launch.py
# Robô anda em círculo automaticamente
```

**Opção C: Controle via Gazebo**
```bash
# Não precisa de comando - apenas clique na janela do Gazebo
# Use setas do teclado: ↑ ↓ ← → e B para parar
```

---

## 🔍 Diagnóstico e Verificação

### Verificar TF Tree
```bash
# Gerar diagrama da TF tree
ros2 run tf2_tools view_frames

# Aguardar 5 segundos, depois:
evince frames.pdf  # ou xdg-open frames.pdf
```

### Verificar Tópicos Ativos
```bash
# Listar todos os tópicos
ros2 topic list

# Ver info de um tópico específico
ros2 topic info /cmd_vel
ros2 topic info /scan
ros2 topic info /odom

# Monitorar mensagens em tempo real
ros2 topic echo /cmd_vel
ros2 topic echo /scan --no-arr  # LiDAR (sem arrays grandes)
ros2 topic echo /odom
```

### Verificar Nós ROS2
```bash
# Listar nós ativos
ros2 node list

# Ver info de um nó
ros2 node info /slam_toolbox
ros2 node info /robot_state_publisher
```

### Verificar Transformações (TF)
```bash
# Ver todas as transformações disponíveis
ros2 run tf2_ros tf2_echo odom base_link

# Monitorar transformação específica
ros2 run tf2_ros tf2_monitor
```

### Verificar Bridges Gazebo-ROS2
```bash
# Listar nós de bridge
ros2 node list | grep bridge

# Ver tópicos do Gazebo (gz/ign)
ign topic -l  # ou gz topic -l
```

---

## 🛠️ Compilação e Build

### Build Completo
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Build Apenas robot_bringup
```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup
source install/setup.bash
```

### Limpar Build
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build
```

---

## 🗺️ Salvar e Carregar Mapas

### Salvar Mapa Atual
```bash
# Enquanto SLAM está rodando
cd ~/ros2_ws
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: meu_mapa}}"

# OU usando run:
ros2 run nav2_map_server map_saver_cli -f ~/maps/meu_mapa
```

### Listar Mapas Salvos
```bash
ls -lh ~/maps/
# OU se salvou no workspace:
ls -lh ~/ros2_ws/*.pgm ~/ros2_ws/*.yaml
```

---

## 🎮 Controle Manual Direto

### Publicar Comandos de Velocidade
```bash
# Avançar
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Girar esquerda
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Girar direita
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.5}}"

# Parar
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## 🐛 Debug e Troubleshooting

### Verificar se Gazebo está rodando
```bash
ign gazebo --version  # ou gz gazebo --version
ps aux | grep gz-sim
```

### Verificar Logs de um Nó
```bash
# Ver logs em tempo real
ros2 run rqt_console rqt_console

# OU via terminal:
ros2 topic echo /rosout
```

### Verificar Parâmetros do SLAM
```bash
# Listar parâmetros do SLAM Toolbox
ros2 param list /slam_toolbox

# Ver valor de um parâmetro específico
ros2 param get /slam_toolbox use_sim_time
```

### Reiniciar Sistema Completo
```bash
# Fechar todos os terminais (Ctrl+C em cada um)
# Depois reiniciar na ordem:
# 1. Gazebo
# 2. Mapping
# 3. Controle (opcional)
```

### Matar Processos Travados
```bash
# Matar Gazebo
pkill -9 gz-sim
# OU
pkill -9 ign

# Matar todos os nós ROS2
pkill -9 -f ros2

# Limpar memória compartilhada (se necessário)
rm -rf /dev/shm/fastrtps*
```

---

## 📊 Visualização

### RViz (já incluído no mapping.launch.py)
- ✅ Abre automaticamente com configuração salva

### RViz Manual (se necessário)
```bash
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/robot_bringup/config/mapping_config.rviz
```

### Gazebo Interface
- Clique com botão direito para mover câmera
- Scroll para zoom
- Ctrl+Click para rotacionar

---

## 🔄 Atalhos Úteis

### Sourcear Workspace (adicionar ao ~/.bashrc)
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Alias Úteis (adicionar ao ~/.bashrc)
```bash
alias ws='cd ~/ros2_ws'
alias build='cd ~/ros2_ws && colcon build && source install/setup.bash'
alias source_ws='source ~/ros2_ws/install/setup.bash'
alias launch_gazebo='ros2 launch robot_bringup gazebo_world.launch.py'
alias launch_slam='ros2 launch robot_bringup mapping.launch.py'
alias launch_teleop='ros2 launch robot_bringup teleop_keyboard.launch.py'
```

Depois:
```bash
source ~/.bashrc
```

---

## 📚 Documentação

- **MODOS_CONTROLE.md** - Guia completo de modos de controle
- **CORRECOES_APLICADAS.md** - Resumo das correções feitas
- **HISTORICO_CONFIGURACAO_SLAM.txt** - Histórico de configurações anteriores
- **COMO_EXECUTAR.md** - Instruções gerais de execução

---

## 🎯 Workflow Típico de Mapeamento

1. **Iniciar Gazebo**
   ```bash
   ros2 launch robot_bringup gazebo_world.launch.py
   ```

2. **Iniciar SLAM**
   ```bash
   ros2 launch robot_bringup mapping.launch.py
   ```

3. **Controlar Robô**
   ```bash
   ros2 launch robot_bringup teleop_keyboard.launch.py
   ```

4. **Navegar pelo Ambiente**
   - Use i/j/k/l para movimentar
   - Observe mapa sendo construído no RViz

5. **Salvar Mapa**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/ambiente_mapeado
   ```

6. **Finalizar**
   - Ctrl+C em cada terminal (ordem inversa: 3→2→1)

---

## ⚠️ Lembretes Importantes

- ✅ Sempre source o workspace: `source ~/ros2_ws/install/setup.bash`
- ✅ Iniciar Gazebo ANTES do SLAM
- ✅ Usar APENAS UM modo de controle por vez
- ✅ `use_sim_time=true` em todos os nós durante simulação
- ✅ Verificar TF tree se houver problemas de localização
- ⚠️ Não rodar múltiplos publishers de `/cmd_vel` simultaneamente

---

**Última atualização:** $(date +"%d/%m/%Y")
**Workspace:** ~/ros2_ws
**Pacote:** robot_bringup
