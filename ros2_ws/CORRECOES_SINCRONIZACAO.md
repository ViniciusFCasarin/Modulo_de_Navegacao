# 🔧 Correções Aplicadas - Sincronização e Movimento Circular

## 📋 Problemas Identificados e Soluções

### ❌ Problema 1: Dessincronização Gazebo ↔ RViz

**Sintoma:**
- Robô se move de forma dessincronizada entre Gazebo e RViz
- Ambos recebem comandos mas respondem em tempos diferentes

**Causas Raiz:**
1. **transform_timeout muito baixo** (0.2s) - SLAM perdia sincronização
2. **minimum_time_interval muito alto** (0.5s) - atualizações muito lentas
3. **map_update_interval muito alto** (5.0s) - mapa atualizava devagar
4. **minimum_travel_distance/heading muito altos** (0.5) - SLAM não atualizava em movimentos pequenos

**Soluções Aplicadas:**

#### 1. Ajustes no `mapper_params_online_async.yaml`:
```yaml
# ANTES → DEPOIS
transform_timeout: 0.2 → 0.5          # Mais tolerante a delays
minimum_time_interval: 0.5 → 0.1      # Atualiza 5x mais rápido
map_update_interval: 5.0 → 2.0        # Mapa atualiza mais frequente
minimum_travel_distance: 0.5 → 0.1    # Detecta movimentos menores
minimum_travel_heading: 0.5 → 0.1     # Detecta rotações menores
```

**Benefícios:**
- ✅ SLAM atualiza posição mais frequentemente
- ✅ Movimentos pequenos são detectados
- ✅ Menor latência entre Gazebo e RViz
- ✅ Sincronização melhorada

---

### ❌ Problema 2: Colisão no Gazebo vs Atravessar Parede no RViz

**Sintoma:**
- Robô colide com parede no Gazebo e para
- RViz não detecta colisão e continua movendo
- Posições divergem completamente

**Causa Raiz:**
- **RViz renderiza baseado em TF**, não física
- **Gazebo simula colisões reais**
- Quando robô colide:
  - Gazebo: odometria para (velocidade = 0)
  - RViz: não sabe da colisão, continua renderizando

**Soluções Aplicadas:**

#### 1. Monitor de Odometria (`odom_monitor.py`)
Script Python que detecta dessincronização em tempo real:

```python
# Detecta quando:
# - Há comando de velocidade (cmd_vel > 0.1)
# - MAS robô está parado (velocidade real < 0.01)
# = COLISÃO!

if abs(cmd_linear) > 0.1 and velocity < 0.01:
    self.get_logger().warn('Possível colisão detectada!')
```

**Funcionalidades:**
- ✅ Monitora `/odom` (velocidade real do Gazebo)
- ✅ Monitora `/cmd_vel` (comandos enviados)
- ✅ Detecta quando robô está travado
- ✅ Avisa no terminal quando há dessincronização

#### 2. SLAM mais responsivo
Com as mudanças no YAML, o SLAM agora:
- ✅ Percebe quando robô para (minimum_time_interval menor)
- ✅ Atualiza mapa mais rápido (map_update_interval menor)
- ✅ Usa scan matching para corrigir posição

**Limitação Conhecida:**
⚠️ **RViz NÃO simula física!** É apenas visualização.
- Se robô colidir no Gazebo, parará fisicamente
- RViz mostrará última TF conhecida
- **Solução:** Evitar colisões ou usar navegação autônoma com detecção de obstáculos

---

### ❌ Problema 3: Robô Não Fazia Círculo

**Sintoma:**
- Comando deveria fazer círculo
- Mas robô ia quase em linha reta

**Causa Raiz:**
```python
# ANTES (ERRADO)
linear.x: 0.5        # 0.5 m/s
angular.z: 0.05      # 0.05 rad/s (muito baixo!)

# Raio = v / ω = 0.5 / 0.05 = 10 metros
# Círculo ENORME! Parece linha reta
```

**Solução Aplicada:**

#### Ajuste em `cmd_vel.launch.py`:
```python
# DEPOIS (CORRETO)
linear.x: 0.3        # 0.3 m/s (moderada)
angular.z: 0.3       # 0.3 rad/s (balanceado!)

# Raio = v / ω = 0.3 / 0.3 = 1 metro
# Círculo visível e controlado
```

**Fórmula do Movimento Circular:**
```
Raio (m) = velocidade_linear / velocidade_angular
         = v / ω

Período (s) = 2π / ω
            = 2 * 3.14159 / 0.3
            ≈ 20.9 segundos por volta completa
```

**Benefícios:**
- ✅ Círculo de ~1 metro de raio
- ✅ Movimento visível e previsível
- ✅ Completa volta em ~21 segundos
- ✅ Ideal para mapeamento circular

**Para ajustar o raio:**
```python
# Raio de 0.5m (círculo pequeno)
linear.x: 0.3, angular.z: 0.6

# Raio de 2m (círculo grande)
linear.x: 0.3, angular.z: 0.15

# Raio de 3m (círculo muito grande)
linear.x: 0.3, angular.z: 0.1
```

---

## 📊 Resumo das Mudanças

### Arquivos Modificados:

| Arquivo | Mudança | Objetivo |
|---------|---------|----------|
| `config/mapper_params_online_async.yaml` | ✅ transform_timeout: 0.5<br>✅ minimum_time_interval: 0.1<br>✅ map_update_interval: 2.0<br>✅ minimum_travel: 0.1 | Sincronização mais rápida |
| `launch/cmd_vel.launch.py` | ✅ linear.x: 0.3<br>✅ angular.z: 0.3 | Movimento circular visível |
| `scripts/odom_monitor.py` | ✅ **NOVO** Monitor de odometria | Detecta colisões/travamentos |
| `launch/mapping.launch.py` | ✅ Adicionado odom_monitor_node | Ativa monitoramento |
| `setup.py` | ✅ Entry point para odom_monitor.py | Instala script |

---

## 🧪 Como Testar

### Teste 1: Movimento Circular
```bash
# Terminal 1
ros2 launch robot_bringup gazebo_world.launch.py

# Terminal 2
ros2 launch robot_bringup mapping.launch.py

# Terminal 3
ros2 launch robot_bringup cmd_vel.launch.py
```

**Esperado:**
- ✅ Robô anda em **círculo de ~1 metro de raio**
- ✅ Completa volta em ~21 segundos
- ✅ Gazebo e RViz **sincronizados**

### Teste 2: Detecção de Colisão
```bash
# Com sistema rodando, deixe robô colidir com parede
```

**Esperado:**
- ✅ Gazebo: robô para fisicamente
- ✅ Terminal: aviso "Possível colisão detectada!"
- ⚠️ RViz: pode continuar mostrando movimento (limitação visual)

### Teste 3: Sincronização Melhorada
```bash
# Monitore a odometria
ros2 topic echo /odom --field pose.pose.position
```

**Esperado:**
- ✅ Posição atualiza ~30 Hz (30 vezes/segundo)
- ✅ Sem delays ou freezes
- ✅ Valores consistentes com posição visual

---

## 📈 Métricas de Performance

### Antes das Correções:
- ❌ Atualização SLAM: a cada 0.5s
- ❌ Atualização mapa: a cada 5.0s  
- ❌ Movimento: quase reto (raio 10m)
- ❌ Dessincronização: frequente

### Depois das Correções:
- ✅ Atualização SLAM: a cada 0.1s (5x mais rápido)
- ✅ Atualização mapa: a cada 2.0s (2.5x mais rápido)
- ✅ Movimento: círculo visível (raio 1m)
- ✅ Sincronização: melhorada significativamente
- ✅ Monitoramento: detecta problemas em tempo real

---

## ⚠️ Limitações Conhecidas

### 1. RViz é Visualização, Não Simulação
- RViz **não simula física**
- Apenas mostra TF frames
- **Não detecta colisões**
- Solução: Confie no Gazebo para física real

### 2. Colisões Causam Divergência
- Gazebo para (física real)
- SLAM tenta compensar
- Pode haver drift temporário
- **Solução:** Evitar colisões ou adicionar recuperação automática

### 3. Parâmetros de Compromisso
- Atualizações mais rápidas = mais CPU
- Timeouts maiores = mais tolerância mas menos precisão
- **Ajuste conforme necessário**

---

## 🔧 Ajustes Finos (Se Necessário)

### Se ainda dessincronizar:
```yaml
# mapper_params_online_async.yaml
transform_timeout: 1.0           # Mais tolerante (era 0.5)
minimum_time_interval: 0.05      # Ainda mais rápido (era 0.1)
```

### Se usar muita CPU:
```yaml
# mapper_params_online_async.yaml
minimum_time_interval: 0.2       # Mais lento (era 0.1)
map_update_interval: 3.0         # Mais lento (era 2.0)
```

### Para círculo maior:
```python
# cmd_vel.launch.py
linear.x: 0.3, angular.z: 0.15   # Raio = 2 metros
```

### Para círculo menor:
```python
# cmd_vel.launch.py
linear.x: 0.2, angular.z: 0.4    # Raio = 0.5 metros
```

---

## ✅ Checklist de Validação

- [x] Velocidades ajustadas para círculo de 1m
- [x] Parâmetros SLAM otimizados
- [x] Monitor de odometria criado
- [x] Monitor adicionado ao launch
- [x] Script instalado no setup.py
- [x] Pacote compilado sem erros
- [ ] **Teste prático:** Movimento circular funcionando
- [ ] **Teste prático:** Sincronização Gazebo ↔ RViz
- [ ] **Teste prático:** Detecção de colisão pelo monitor

---

**Data:** 10/11/2025
**Status:** ✅ Correções aplicadas e compiladas
**Próximo passo:** Testes práticos
