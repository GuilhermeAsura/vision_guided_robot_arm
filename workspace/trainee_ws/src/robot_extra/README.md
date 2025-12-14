# Robot Extra Challenges (IK & Depth Estimation)

Este pacote contém as implementações avançadas do projeto, focando em desafios de percepção 3D e controle autônomo baseado em coordenadas espaciais. Ele estende as capacidades do manipulador UR5e para entender a profundidade de objetos usando uma câmera monocular e calcular trajetórias usando Cinemática Inversa (IK).

## 🎯 Objetivos

1.  **Estimativa de Profundidade Monocular:** Calcular a distância ($Z$) de um objeto conhecido (bola) baseando-se apenas no seu tamanho em pixels na imagem (Modelo Pinhole).
2.  **Transformação de Coordenadas (TF2):** Converter coordenadas do referencial da Câmera (Optical Frame) para o referencial da Base do Robô (World Frame).
3.  **Cinemática Inversa (IK):** Utilizar a biblioteca `ikpy` para mover o efetuador do robô para uma coordenada $(x, y, z)$ específica no espaço.

## 📂 Estrutura do Pacote

```text
robot_extra/
├── launch/
│   ├── extra_challenge.launch.py  # Lança Visual Servoing com profundidade
│   ├── ik_challenge.launch.py     # O MAIS COMPLETO: IK + TF + URDF + Visão
│   └── depth_estimator.launch.py  # Apenas o nó de visão (debug)
├── robot_extra/
│   ├── depth_estimator_node.py    # Percepção 3D e Transformada TF
│   ├── ik_controller_node.py      # Controlador IK (Inverse Kinematics)
│   └── robot_controller_node.py   # Controlador P (Visual Servoing Simples)
├── package.xml
└── setup.py
````

## ⚙️ Dependências

Este pacote faz uso intensivo de bibliotecas matemáticas e de transformação do ROS2:

- **`tf2_ros` & `tf2_geometry_msgs`**: Para gerenciar a árvore de transformadas (TF Tree).
    
- **`python3-ikpy`**: Motor de Cinemática Inversa.
    
- **`robot_state_publisher`**: Para publicar o modelo estático do robô baseado no URDF.
    
- **`cv_bridge` & `opencv`**: Processamento de imagem.
    

## 🚀 Como Executar

### Cenário 1: O Desafio Completo (IK + TF)

Este launch carrega o URDF do robô, publica as transformadas, calcula a posição da bola e move o robô usando cinemática inversa.


``` Bash
ros2 launch robot_extra ik_challenge.launch.py
```

> **Fluxo de Operação:**
> 
> 1. Pressione `R` no terminal do teclado para ir para a posição inicial.
>     
> 2. Pressione `T` para iniciar o cálculo de trajetória até a bola detectada.
>     

### Cenário 2: Visual Servoing com Profundidade

Uma abordagem alternativa que usa lógica de controle proporcional (não IK) para alinhar e aproximar.


``` Bash
ros2 launch robot_extra extra_challenge.launch.py
```

## 🧠 Nós (Nodes) Detalhados

### 1. `depth_estimator_node.py` (O Matemático)

Este nó é responsável por converter "pixels" em "metros" e "visão" em "coordenadas de mundo".

- Matemática (Pinhole Camera):
    
    Utiliza a relação de semelhança de triângulos:
    
    $$ Z = \frac{f \cdot R_{real}}{R_{pixel}} $$
    
    Onde $f$ é a distância focal, $R_{real}$ é o raio físico da bola (7cm) e $R_{pixel}$ é o raio detectado na imagem.
    
- Transformação TF2:
    
    O nó escuta a transformação entre base_link e camera_link_optical. Quando detecta a bola, ele converte a posição $(x,y,z)$ da câmera para a base do robô e publica no tópico /vision/target_world_frame.
    

### 2. `ik_controller_node.py` (O Planejador)

Este nó substitui o controle manual por planejamento de movimento.

- **Entrada:** Recebe um `PointStamped` com as coordenadas $(x, y, z)$ do alvo no referencial do mundo.
    
- **Processamento:**
    
    1. Lê o estado atual dos motores (juntas).
        
    2. Utiliza `ikpy` carregando o URDF do UR5e.
        
    3. Calcula a solução IK para posicionar o efetuador final nas coordenadas recebidas.
        
- **Saída:** Publica `/ur5e/joint_targets` para mover o robô.
    
- **Destaque:** Ele também publica `/joint_states` para manter a árvore TF do ROS atualizada em tempo real.
    

### 3. `robot_controller_node.py` (Alternativa Lógica)

Um controlador reativo que não usa IK, mas sim lógica sequencial:

1. Gira a base para alinhar X.
    
2. Move o ombro para alinhar Y.
    
3. Estende o cotovelo para reduzir a diferença de profundidade ($Z_{atual} - Z_{alvo}$).
    

## 🛠️ Arquitetura de Transformadas (TF)

O sucesso deste pacote depende da correta configuração do TF no arquivo `ik_challenge.launch.py`:

1. **`robot_state_publisher`**: Lê o URDF e publica as transformações estáticas entre as juntas do braço.
    
2. **`static_transform_publisher`**: Cria o link inexistente entre o punho do robô (`wrist_3_link`) e a câmera (`camera_link_optical`).
    
    - Ajuste fino realizado: Translação de `-5cm` em Z e rotação para alinhar o eixo óptico (Z-frente) com o eixo do robô.
