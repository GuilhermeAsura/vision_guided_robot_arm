# Arm Simulation (UR5e Webots Interface)

Este pacote é a fundação do projeto, responsável por instanciar o ambiente de simulação e fornecer a interface de controle de baixo nível para o braço robótico UR5e.

Diferente de abordagens tradicionais que usam apenas `ros2_control`, este pacote implementa um **Nó Híbrido**. Este nó atua simultaneamente como um controlador nativo do Webots (acessando motores e sensores diretamente via API) e como um nó ROS2 (publicando estados e assinando tópicos de comando).

## 🎯 Objetivos

1.  **Simulação:** Carregar o mundo `basic_arm.wbt` com o robô UR5e.
2.  **Bridge de Hardware:** Atuar como driver, convertendo comandos ROS em `setPosition` dos motores.
3.  **Telemetria:** Publicar o estado atual das juntas (`joint_states`) e a imagem da câmera simulada.
4.  **Prototipagem de Controle:** Serviu como base para testar algoritmos de Visual Servoing (rastreamento de objetos) antes da separação em módulos dedicados.

## 📂 Estrutura do Pacote

```text
arm_simulation/
├── launch/
│   ├── hybrid_launch.py        # Launcher principal (Webots + Controller)
│   └── arm_launch.py           # (Legado) Tentativa via webots_ros2_driver
├── arm_simulation/
│   ├── hybrid_controller.py    # O DRIVER PRINCIPAL (Nó Híbrido)
│   ├── ur5e_controller_ros2.py # Versão anterior do controlador
│   ├── ur5e_controller2.py     # Teste de controle puro (sem ROS)
│   └── find_joints.py          # Utilitário para listar devices
├── worlds/
│   └── basic_arm.wbt           # Arquivo do mundo Webots
├── package.xml
└── setup.py
````

## ⚙️ Dependências

- **Webots R2023b+** (Testado no R2025a)
    
- **ROS2 Humble**
    
- Pacotes Python: `controller` (Webots API), `rclpy`.
    

## 🚀 Como Executar

Este pacote contém o lançador que inicializa o simulador.


``` Bash
cd ~/trainee_ws
colcon build --packages-select arm_simulation
source install/setup.bash

# Inicia o Webots e o Controlador Híbrido
ros2 launch arm_simulation hybrid_launch.py
```

## 🧠 Detalhes do `hybrid_controller.py`

Este script é o coração deste pacote. Ele roda um loop infinito sincronizado com o `timestep` do Webots.

### Funcionalidades:

1. **Controle Manual (Teclado Webots):**
    
    - Se a janela 3D do Webots estiver focada, é possível controlar o robô diretamente:
        
    - `1-6`: Seleciona a junta (Base, Ombro, Cotovelo, etc).
        
    - `↑ / ↓`: Move a junta selecionada (+/-).
        
    - `T`: Ativa/Desativa o modo de Rastreamento Visual (Tracking).
        
2. **Interface ROS2 (Tópicos):**
    

|**Tópico**|**Tipo**|**Direção**|**Descrição**|
|---|---|---|---|
|`/ur5e/webots_joint_states`|`Float64MultiArray`|Pub|Posição atual das 6 juntas (em radianos).|
|`/UR5e/camera_sensor/image_color`|`sensor_msgs/Image`|Pub|Stream de vídeo cru da câmera (BGRA).|
|`/ur5e/target_positions`|`Float64MultiArray`|Sub|Recebe vetor com 6 posições alvo para mover o braço.|
|`/vision/object_coordinates`|`geometry_msgs/Point`|Sub|Recebe coordenadas (x,y) do objeto para o Visual Servoing.|

### Visual Servoing Embutido

O hybrid_controller.py possui uma lógica interna de Proportional Control (Controlador P) no método vision_callback.

Quando o modo "Tracking" é ativado (tecla 'T'), ele usa o erro entre o centro da imagem e o ponto recebido no tópico /vision/object_coordinates para mover a Base (Pan) e o Ombro (Lift) automaticamente.

## 📜 Histórico de Desenvolvimento

A pasta contém scripts que mostram a evolução do aprendizado:

- `ur5e_controller2.py`: Primeiro teste, apenas Python e Webots, sem ROS.
    
- `ur5e_controller_ros2.py`: Primeira integração com ROS, adicionando Publishers básicos.
    
- `hybrid_controller.py`: Versão final robusta, com suporte a câmera, keyboard e controle de posição.
    