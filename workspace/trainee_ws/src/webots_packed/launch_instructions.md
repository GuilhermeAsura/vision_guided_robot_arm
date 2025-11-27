# Guia de Execução: Controle do Braço Robótico sem Garra

Este guia explica como iniciar a simulação e controlar o braço robótico UR5e sem garra utilizando o teclado via ROS2.

## 1. Preparação do Ambiente (Webots)

Antes de rodar os comandos ROS2, o simulador precisa estar pronto para receber conexões externas.

1. **Abra o Webots** dentro do container.
    
2. **Carregue o mundo** (`basic_arm.wbt`) que contém o braço robótico - tal arquivo está em: workspace/trainee_ws/src/arm_simulation/worlds.
    
3. **Aguarde o Controlador estar pronto:**
    
    - deve aparecer o seguinte log no terminal do WeBots
    ```md
    INFO: 'UR5e' extern controller: Waiting for local or remote connection on port 1234 targeting robot named 'UR5e'.
    ```
        
4. **Inicie a Simulação:**
    - Após o log aparecer a simulação está carregada apropriadamente e o launch pode ser executado.
    - Pressione o botão **Play** (▶) no topo da interface do Webots.
    - O tempo de simulação deve estar correndo.
        

## 2. Execução (ROS2)

Abra um terminal no workspace (`/trainee/workspace/trainee_ws`) e execute:

Bash

``` bash
# 1. adicione o xterm para ter acesso ao terminal extra
sudo apt-get update && sudo apt-get install -y xterm

# 2. Garanta que o workspace está compilado
colcon build --symlink-install

# 3. Carregue as configurações do ambiente
source install/setup.bash

# 4. Inicie o sistema de controle
ros2 launch webots_packed robot_control.launch.py
```

> **Nota:** Ao executar o comando acima, uma nova janela branca (**xterm**) será aberta automaticamente. **É nela que você deve digitar os comandos.**

## 3. Comandos de Controle

Clique na janela do **xterm** para dar foco e utilize as seguintes teclas1:

### 🕹️ Seleção de Juntas

Use os números para escolher qual parte do braço mover:

- `1`: Base (Shoulder Pan)
    
- `2`: Ombro (Shoulder Lift)
    
- `3`: Cotovelo (Elbow)
    
- `4`: Punho 1 (Wrist 1)
    
- `5`: Punho 2 (Wrist 2)
    
- `6`: Punho 3 (Wrist 3)
    

### 🚀 Movimentação

Após selecionar a junta, use:

- `W` (ou Seta Cima): Move a junta em sentido **positivo** (+).
    
- `S` (ou Seta Baixo): Move a junta em sentido **negativo** (-).
    

### 🔄 Reset

- `R`: Retorna o robô para a posição inicial (Home)2.
    
