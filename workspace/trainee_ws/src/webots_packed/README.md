# Webots Packed (ROS2 Integration)

Este pacote é o núcleo de integração entre o simulador **Webots** e o **ROS2 Humble**. Ele implementa o controlador do robô (Robot Controller) que traduz mensagens ROS2 em comandos de motor via API do Webots e, simultaneamente, converte os dados da câmera simulada em tópicos de imagem ROS2.

## 🎯 Objetivos

1.  **Atuar como Controlador:** Receber comandos de teleoperação e aplicar torque/posição nas juntas do manipulador (UR5e).
2.  **Bridge de Visão:** Capturar o buffer de imagem bruta da câmera do Webots e publicar como `sensor_msgs/Image`.
3.  **Gerenciamento de Estado:** Monitorar e manter a posição atual de cada junta.

## 📂 Estrutura do Pacote

```text
webots_packed/
├── launch/
│   └── robot_control.launch.py  # Lança o controlador + nó de teclado
├── webots_packed/
│   ├── __init__.py
│   └── webots_listener.py       # Nó principal (Controller + Camera Pub)
├── package.xml
├── setup.py
└── setup.cfg
````

## ⚙️ Dependências

- **ROS2:** `rclpy`, `std_msgs`, `sensor_msgs`.
    
- **Webots:** `webots_ros2_driver` (ou acesso à API Python `controller` nativa do Webots).
    
- **Sistema:** `xterm` (necessário para abrir a janela separada do teclado via launch file).
    

## 🚀 Como Executar

Este pacote geralmente é executado em conjunto com a simulação aberta.


``` Bash
cd ~/trainee_ws
colcon build --packages-select webots_packed
source install/setup.bash

# Inicia o controlador e a interface de teclado
ros2 launch webots_packed robot_control.launch.py
```

> **Nota:** O arquivo launch utiliza o prefixo `xterm -e` para o nó do teclado. Certifique-se de que o `xterm` está instalado no seu container ou sistema (`sudo apt install xterm`), caso contrário, o nó do teclado falhará ao iniciar.

## 🧠 Nó Principal: `webots_listener.py`

Este script atua como um **Hybrid Node**: ele herda de `rclpy.node.Node` para funcionalidades ROS, mas também instancia `controller.Robot` para comandar a simulação.

### Tópicos Assinados (Subscribers)

|**Tópico**|**Tipo**|**Descrição**|
|---|---|---|
|`/keyboard_input`|`std_msgs/Int32`|Recebe códigos inteiros do pacote `keyboard_check` para mover o robô.|

### Tópicos Publicados (Publishers)

|**Tópico**|**Tipo**|**Descrição**|
|---|---|---|
|`/UR5e/camera_sensor/image_color`|`sensor_msgs/Image`|Stream de vídeo da câmera simulada (Encoding: `bgra8`).|
|`ur5e/joint_targets`|`std_msgs/Float64MultiArray`|Vetor com as posições alvo atuais das juntas (para debug/feedback).|

### Lógica de Controle

O nó mantém um vetor de estado das juntas do robô (UR5e + Garra). O controle funciona através de uma máquina de estados simples controlada pelo teclado:

1. **Seleção de Junta:** O usuário seleciona qual motor quer mover (Base, Ombro, Cotovelo, Punhos ou Garra).
    
2. **Incremento/Decremento:** O usuário comanda o movimento positivo ou negativo.
    
3. **Segurança:** O script verifica os limites físicos (`min_position`, `max_position`) do motor antes de aplicar o comando `setPosition`. 

## 📸 Sistema de Câmera

O nó detecta automaticamente um dispositivo chamado `camera_sensor` no robô.

- **Taxa de Atualização:** Sincronizada com o `timestep` da simulação (geralmente 32ms ou aprox. 30Hz).
    
- **Formato:** As imagens são extraídas do Webots em formato `bgra8` (Blue-Green-Red-Alpha) e publicadas diretamente.
