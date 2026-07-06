# Sistema de Navegação Robótica - ROS2

Workspace ROS2 para navegação autônoma de robôs com integração SLAM e Nav2.

O projeto funciona em ambiente Linux, as execuções foram testadas no Ubuntu. A pasta raiz deve estar em ~/ros2_ws.

Os arquivos principais do projeto estão em ros2_ws\src\robot_bringup.

## Como Rodar

> **⚠️ Importante:** Em todos os terminais, primeiramente execute:
> ```bash
> source /home/vinicius/ros2_ws/install/setup.bash
> ```

### Terminal 1 - Gazebo e Simulação

Compilar (apenas na primeira vez):
```bash
colcon build --packages-select robot_bringup --symlink-install
```

Executar o Gazebo com cenário carregado:
```bash
ros2 launch robot_bringup gazebo_world.launch.py
```

### Terminal 2 - Visualizador RViz

Abrir o visualizador RViz com mapeamento:
```bash
ros2 launch robot_bringup mapping.launch.py
```

### Terminal 3 - Nav2 e SLAM

Iniciar o Nav2 para navegação autônoma e o SLAM para mapeamento:
```bash
ros2 launch robot_bringup navigation_launch.py
```

### Terminal 4 - Controle Manual

Controlar robô manualmente para exploração inicial:
```bash
ros2 launch robot_bringup teleop_keyboard.launch.py
```

### Terminal 5 (Opcional) - Navegação Automática

Executar script para navegar até pontos determinados no arquivo JSON:
```bash
python3 src/robot_bringup/scripts/navigate_to_goal.py
```

---

## Estrutura do Projeto

### 📁 `src/robot_bringup`
Pacote principal contendo toda a configuração e lógica do sistema de navegação.

#### **config**
Parâmetros de inicialização do sistema:
- `nav2_params.yaml` - Configurações do Nav2 (planejador, controle)
- `mapper_params_online_async.yaml` - Parâmetros do SLAM assíncrono
- `goals.json` - Definição dos waypoints/objetivos de navegação
- `*.rviz` - Configurações de visualização do RViz

#### **launch**
Automatizadores de inicialização, bridge de comunicação e configuração:
- `navigation_launch.py` - Inicia sistema de navegação completo
- `mapping.launch.py` - Inicializa módulo SLAM para mapeamento
- `gazebo_world.launch.py` - Lança simulador Gazebo com ambiente
- `teleop_keyboard.launch.py` - Controle manual via teclado
- `cmd_vel.launch.py` - Gerenciador de velocidades
- `view_map.launch.py` - Visualizador de mapas

#### **models**
Modelos 3D para simulação:
- **Robot/** - Definição do robô em SDF (Gazebo)
- **Cenarios/** - Ambientes de simulação (obstáculos, paredes)

#### **scripts**
Utilidades e interfaces para operação do sistema:
- `navigate_to_goal.py` - Interface de pontos objetivos rotulados
- `save_map.py` - Salvamento de mapas gerados pelo SLAM
- `odom_monitor.py` - Monitoramento de odometria
- `odom_tf_publisher.py` - Publicador de transformações (TF)

#### **worlds**
Ambientes de simulação:
- Arquivo principal de configuração do mundo
- Robô em URDF para RViz e Nav2

#### **setup.py**
Arquivo de configuração das pastas para build com `colcon`:
- Define dependências
- Configura instalação de recursos (launch files, configurações, modelos)

---

### 📁 `src/ros_gz`
Ferramentas e plugins ROS2 para integração com Gazebo.

---

### 📁 `maps`
Repositório de mapas gerados pelo SLAM:
- Mapas em formato PGM (imagem)
- Poses YAML com informações de posição
- Grafos de poses (.posegraph)

---

### 🔧 `quick_save_map.sh`
Script auxiliar para salvar mapas SLAM rapidamente durante mapeamento.

---

## Funcionalidades Principais

✅ Mapeamento autônomo com SLAM  
✅ Navegação autônoma com Nav2  
✅ Simulação em Gazebo  
✅ Visualização em RViz  
✅ Controle manual via teclado  
✅ Planejamento de rotas para múltiplos objetivos  

---

## Scripts Auxiliares (.sh)

### 1️⃣ `install_nav2.sh` - Instalação
**Função**: Instala e configura o **Nav2** e todas suas dependências no sistema.

**O que faz**:
- Verifica se ROS2 Humble está instalado
- Instala Navigation2, SLAM Toolbox e pacotes relacionados
- Verifica se a instalação foi bem-sucedida
- Orienta próximos passos

**Quando usar**: Uma única vez, na primeira configuração do ambiente

```bash
./install_nav2.sh
```

---

### 2️⃣ `clean_system.sh` - Limpeza do Sistema
**Função**: Mata todos os processos ROS2 e limpa o sistema.

**O que faz**:
- Encerra `rviz2` (visualizador)
- Encerra `gz` (Gazebo)
- Mata todos os nós ROS2 ativos
- Para SLAM Toolbox e outros serviços
- Limpa variáveis de ambiente
- Reseta a implementação de middleware

**Quando usar**: Quando o sistema trava ou você precisa resetar tudo (ao invés de reiniciar o PC)

```bash
./clean_system.sh
```

---

### 3️⃣ `quick_save_map.sh` - Salvamento Rápido de Mapa
**Função**: Salva rapidamente o mapa gerado pelo SLAM.

**O que faz**:
- Verifica se SLAM Toolbox está rodando
- Chama serviço para salvar o mapa (gera `.yaml` e `.pgm`)
- Serializa o grafo de poses (gera `.posegraph`)
- Salva tudo automaticamente em `/home/vinicius/ros2_ws/maps/`

**Quando usar**: Após finalizar o mapeamento do ambiente

```bash
# Nome automático com data/hora
./quick_save_map.sh

# Nome customizado
./quick_save_map.sh meu_ambiente
```

---

## Resumo de Uso dos Scripts
| Script | Função | Frequência |
|--------|--------|-----------|
| `install_nav2.sh` | Instalar Nav2 | Primeira vez |
| `quick_save_map.sh` | Salvar mapa | Após mapeamento |
| `clean_system.sh` | Resetar sistema | Se travar |

---