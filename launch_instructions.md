# 🤖 Guia de Execução de Pacotes

Este guia explica como iniciar a simulação e executar os diferentes modos de operação do braço robótico UR5e (Controle Simples e Sistema de Visão) utilizando ROS2.

## 1\. Preparação do Ambiente (Webots)

Antes de rodar qualquer comando ROS2, o simulador precisa estar pronto.

1.  **Abra o Webots** dentro do container.
2.  **Carregue o mundo correto:**
      * Para testes básicos: `basic_arm.wbt`.
      * Para testes de visão/garra: `armed_gripper_cam.wbt`
3.  **Verifique o Controlador:**
      * Certifique-se de que o campo `controller` do robô está definido como `<extern>`.
4.  **Inicie a Simulação:**
      * Pressione o botão **Play** (▶).
      * Aguarde um log semelhante a: `INFO: 'UR5e' extern controller: Waiting for local or remote connection...`

## 2\. Configuração Inicial do Workspace

Abra um terminal no workspace (`/trainee/workspace/trainee_ws`) e execute a preparação básica:

```bash
# 1. Instale o xterm (necessário para capturar o teclado)
sudo apt-get update && sudo apt-get install -y xterm

# 2. Compile o workspace (sempre que houver alterações)
colcon build --symlink-install

# 3. Carregue as variáveis de ambiente
source install/setup.bash
```

-----

## 3\. Modos de Execução

Escolha qual sistema você deseja iniciar abaixo.

### 🅰️ Opção A: Controle Básico via Teclado

Utilize este modo se quiser apenas testar a movimentação das juntas, sem carregar o processamento de imagem.

**Comando:**

```bash
ros2 launch webots_packed robot_control.launch.py
```

  * **O que abre:** Uma janela branca (**xterm**) para controle.
  * **Comportamento:** O robô obedece aos comandos do teclado, mas não há feedback visual da câmera.

### 🅱️ Opção B: Sistema Completo (Controle + Visão + Telemetria)

Utilize este modo para a **entrega final**, integrando controle manual e percepção visual.

**Comando:**

```bash
ros2 launch robot_vision complete_system.launch.py
```

*(Nota: Certifique-se de que o pacote `robot_vision` foi compilado)*

  * **O que abre:**
    1.  Janela branca (**xterm**): Para digitar comandos de movimento.
    2.  Janela de Vídeo (**Camera Feed**): Mostra a visão do robô e o processamento (círculo verde no objeto detectado).
    3.  Terminal Principal: Exibe logs de telemetria `(X, Y)` do objeto detectado. 

-----

## 4\. Comandos de Controle (Janela xterm)

Clique na janela do **xterm** para dar foco antes de digitar.

### 🕹️ Seleção de Juntas e Garra

Use os números para selecionar qual parte controlar:

  * `1`: Base (Shoulder Pan)
  * `2`: Ombro (Shoulder Lift)
  * `3`: Cotovelo (Elbow)
  * `4`: Punho 1 (Wrist 1)
  * `5`: Punho 2 (Wrist 2)
  * `6`: Punho 3 (Wrist 3)
  * `7`: **Garra** (Dedo Esquerdo)
  * `8`: **Garra** (Dedo Direito)

### 🚀 Movimentação

Após selecionar a junta ou garra:

  * `W` (ou Seta Cima): Move em sentido **positivo** (+) / Fecha a garra.
  * `S` (ou Seta Baixo): Move em sentido **negativo** (-) / Abre a garra.

### 🔄 Comandos Globais

  * `R`: **Reset** - Retorna o robô para a posição inicial (Home).
  * `T`: **Tracking** (Apenas no modo Visual Servoing antigo) - Ativa/Desativa o modo de perseguição automática (se implementado no controller híbrido).

-----

## 5\. Monitoramento de Visão (Apenas Opção B)

Ao rodar o **Sistema Completo**, você pode verificar a detecção do objeto:

1.  **Visual:** Observe a janela `Camera Feed`. Um ponto verde deve aparecer sobre o objeto vermelho.
2.  **Dados:** No terminal onde você rodou o launch, ou em um novo terminal usando `ros2 topic echo /vision/object_coordinates`, verifique as coordenadas:
      * **X:** Posição horizontal em pixels (0 a 640).
      * **Y:** Posição vertical em pixels (0 a 480).