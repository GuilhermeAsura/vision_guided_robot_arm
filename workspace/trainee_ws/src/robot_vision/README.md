# Robot Vision & Control (ROS2 Package)

Este pacote implementa o sistema de percepção e controle autônomo do manipulador UR5e. Ele é responsável por processar as imagens da câmera simulada, extrair coordenadas de objetos de interesse (círculos vermelhos) e comandar o braço robótico utilizando tanto **Visual Servoing** (controle reativo) quanto **Cinemática Inversa** (planejamento de trajetória).

## 🎯 Funcionalidades Principais

1.  **Processamento de Imagem:** Detecção de cores e cálculo de centroides usando OpenCV.
2.  **Visual Servoing:** Algoritmo de controle proporcional para alinhar a câmera com o objeto.
3.  **Cinemática Inversa (IK):** Uso da biblioteca `ikpy` para calcular os ângulos das juntas a partir de coordenadas (x, y, z).
4.  **Automação (Pick & Place):** Máquina de estados finita para executar a tarefa de pegar e levantar um objeto.

## 📂 Estrutura do Pacote

```text
robot_vision/
├── launch/
│   ├── complete_system.launch.py    # Lança todo o sistema (Visão + Controle Manual)
│   └── visual_servoing.launch.py    # Lança o sistema de Visão + Controle Autônomo
├── robot_vision/
│   ├── __init__.py
│   ├── vision_node.py               # Nó de Percepção (OpenCV)
│   ├── visual_controller.py         # Nó de Autonomia (IK + Máquina de Estados)
│   └── visual_servoing_controller.py # Nó de Rastreamento Simples (P-Controller)
├── package.xml
└── setup.py
````

## ⚙️ Dependências

Além das dependências padrão do ROS2 (`rclpy`, `std_msgs`, `geometry_msgs`), este pacote requer:

- **OpenCV (`python3-opencv`):** Para processamento de imagem.
    
- **CvBridge (`cv_bridge`):** Para converter mensagens ROS `sensor_msgs/Image` em arrays NumPy.
    
- **IKPy (`ikpy`):** Para cálculos de cinemática inversa baseados em URDF.
    
- **NumPy:** Para álgebra linear.
    

## 🧠 Nós (Nodes) Detalhados

### 1. `vision_node.py` (O "Olho")

Este nó atua puramente na camada de percepção. Ele não sabe o que é um robô, apenas processa imagens.

- **Assina:** `/UR5e/camera_sensor/image_color`
    
- **Publica:** `/vision/object_coordinates` (Tipo: `geometry_msgs/Point`)
    
- **Algoritmo:**
    
    1. Aplica desfoque (Gaussian Blur) para reduzir ruído.
        
    2. Converte espaço de cor BGR para **HSV**.
        
    3. Aplica máscaras para filtrar a cor vermelha.
        
    4. Encontra contornos e calcula o **Momento** da imagem para achar o centroide (X, Y).
        

### 2. `visual_servoing_controller.py` (O "Seguidor")

Um controlador híbrido que implementa um comportamento de "olhar para o objeto".

- **Lógica:** Recebe as coordenadas do `vision_node` e calcula o **Erro** (distância do objeto ao centro da imagem).
    
- **Controle:** Aplica um ganho proporcional ($P$) para mover a Base e o Ombro do robô, tentando zerar o erro (centralizar o objeto na imagem).
    
- **Interatividade:** Pressione a tecla `T` para ativar/desativar o rastreamento (Tracking).
    

### 3. `visual_controller.py` (O "Braço Autônomo")

Este é o script mais avançado, responsável pelo _Pick and Place_.

- **Cinemática Inversa:** Carrega o URDF do robô e utiliza `ikpy` para calcular os ângulos necessários para atingir uma posição (X, Y, Z) com a garra apontada para baixo.
    
- **Máquina de Estados:**
    
    - `SEARCH`: Procura o alvo.
        
    - `APPROACH`: Posiciona-se acima do objeto.
        
    - `LOWER`: Desce até a altura de pega.
        
    - `GRASP`: Fecha a garra.
        
    - `LIFT`: Levanta o objeto.
        
- **Ground Truth:** Para garantir a precisão do "agarre" físico, este nó utiliza o `Supervisor` do Webots para obter a posição 3D absoluta do objeto, enquanto a câmera é usada para monitoramento visual.
    

## 🚀 Como Executar

### Cenário 1: Sistema Completo (Manual)

Roda a visão computacional e permite controle pelo teclado.


``` Bash
ros2 launch robot_vision complete_system.launch.py
```

### Cenário 2: Autonomia (Pick & Place ou Servoing)

Roda o nó de visão junto com o controlador autônomo escolhido.


``` Bash
ros2 launch robot_vision visual_servoing.launch.py
```

## 🛠️ Destaques Técnicos

- **Desacoplamento:** A visão computacional roda em um processo separado, publicando coordenadas genéricas. Isso permite trocar o algoritmo de visão sem quebrar o controle do robô.
    
- **Tratamento de Cores:** O uso de HSV em vez de RGB torna a detecção mais robusta a mudanças de iluminação.
    
- **Segurança:** O controlador autônomo verifica limites de juntas e utiliza máscaras de links no IKPy para garantir movimentos suaves.

### Abordagem IBVS - Image-Based Visual Servoing

Adotamos uma arquitetura de Visual Servoing Baseado em Imagem **(IBVS - Image-Based Visual Servoing)** com uma abordagem heurística direta.<br>

(_"Servoing"_ refere-se à técnica de controle de movimento de um robô usando feedback visual extraído de uma câmera.)

  

**1. Eliminamos a Cinemática Inversa (IK)**:

Na robótica clássica, o fluxo seria:

Detectar objeto em pixels `(u, v)`; converter pixels para coordenadas 3D no mundo `(x, y, z)` usando a matriz intrínseca da câmera e profundidade; calcular a Cinemática Inversa para descobrir quais ângulos de junta `(θ1, θ2, ...)` levam o efetuador até `(x, y, z)`; mover para esses ângulos.<br>

<br>

Nós sabemos que se o objeto está à esquerda na imagem, precisamos girar a Base para a esquerda. Sabemos que se o objeto está em cima na imagem, precisamos levantar o Ombro. Assim, mapeamos o **Erro em Pixels** diretamente para **Velocidade da Junta**, sem passar pela matemática complexa de coordenadas cartesianas 3D.

<br>

  

**2. Utilizamos um Controlador Proporcional (P-Controller)**:

<br>

A lógica matemática se resume a: `Velocidade = Ganho x Erro`

- **Erro (e)**: Ele calcula a diferença entre onde o objeto está `(x, y)` e o centro da imagem (320, 240).

- **Lei de Controle**: _`Vjunta = Kp x e`_

Se o objeto está à direita (erro positivo), movemos a junta positivamente.

Se o erro é zero (centralizado), a velocidade é zero.

- **Loop Principal**: A cada passo da simulação (step), lemos os bytes da câmera e empacotamos numa mensagem ROS padrão (bgra8 é o padrão do Webots, o cv_bridge no outro nó fará a conversão automática).

  

Dessa forma o **nó de controle** tem duas responsabilidades:

<br>

**Output (Atuadores)**: Receber comandos e mover juntas (já implementado).<br>

**Input (Sensores)**: Ler a câmera do Webots e publicar a imagem bruta para o ROS.

<br>

Não precisamos de integrais ou derivadas porque o loop de controle roda muito rápido (32ms a 60ms). O robô faz correções minúsculas e contínuas. Se ele não chegar lá na primeira tentativa, o loop roda de novo e ele corrige mais um pouco. Isso remove a necessidade de planejamento de trajetória complexo (Splines, Curvas de Bezier).<br>  

Simplificamos o problema de um sistema **MIMO (Múltiplas Entradas, Múltiplas Saídas)** para dois sistemas simples **SISO (Entrada Única e Saída Única)**, onde o erro em X controla apenas a Junta 0 e o erro em Y controla apenas a Junta 1.<br>

Se fossemos usar a Matriz Jacobiana de Imagem (a forma tradicional), o código teria que calcular matrizes 2x6, inverte-las e fazer multiplicação matricial a cada frame. Nossa abordagem heurística funciona perfeitamente para centralizar objetos sem essa sobrecarga computacional. A lógica diminuiu porque trocamos um **cálculo geométrico explícito** (pesado e extenso) por **controle reativo em malha fechada** (leve e iterativo). O robô não "sabe" onde o objeto está no espaço 3D, ele apenas sabe que precisa reduzir o erro na imagem para zero.