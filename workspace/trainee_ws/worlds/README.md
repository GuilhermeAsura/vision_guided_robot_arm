# Webots Simulation Worlds (.wbt)

Esta pasta contém os arquivos de cenário do Webots utilizados no projeto. Cada arquivo `.wbt` define o ambiente físico, o robô, os sensores e os objetos de interação para uma etapa específica do desenvolvimento.

## 🌍 Lista de Ambientes

| Arquivo                        | Pacote Associado | Descrição e Finalidade                                                                                                                                                                                                                                                                  |
| :----------------------------- | :--------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`armed_gripper_cam.wbt`**    | `robot_vision`   | **Cenário Principal de Visão.**<br>Contém o robô UR5e equipado com a garra Robotiq 2F-85 e uma câmera no punho. O ambiente possui iluminação otimizada para detecção de cores e inclui a "Red Ball" (alvo). Usado para testes de *Visual Servoing* (IBVS).                              |
| **`arm_gripper_physics.wbt`**  | `arm_simulation` | **Cenário de Física e Mecânica.**<br>Focado em testes de colisão e dinâmica da garra. Geralmente usado nas etapas iniciais para validar se o robô consegue segurar objetos sem que eles "escorreguem" ou atravessem a malha da garra (bug comum em simulações).                         |
| **`ik_armed_gripper_cam.wbt`** | `robot_extra`    | **Cenário de Cinemática Inversa.**<br>Similar ao primeiro, mas pode conter marcadores visuais extras ou o objeto posicionado em coordenadas cartesianas conhecidas para validar os cálculos do *Depth Estimator* e do algoritmo IK. O sistema de coordenadas aqui é crítico para o TF2. |

## 🤖 Configuração do Robô (Padrão)

Todos os mundos acima compartilham a seguinte configuração base do manipulador:

* **Robô:** Universal Robots UR5e (6 Graus de Liberdade).
* **Efetuador Final (End-Effector):** Robotiq 2F-85 Gripper.
* **Sensores Adicionados:**
    * `camera_sensor`: Câmera RGB acoplada ao punho (`wrist_3_link`), apontando para a garra.
    * `joint_sensor`: Necessários na IK.

## 🔴 Objetos de Interação

* **Red Ball:** Uma esfera vermelha com física habilitada (massa e atrito).
    * **Propósito:** Ser o alvo fácil de detectar via segmentação de cor (HSV) para as tarefas de *Pick-and-Place*.
    * **Dimensões:** Raio aprox. de 7cm (importante para o cálculo de profundidade monocular).

## 🚀 Como Carregar

Estes arquivos possam ser abertos manualmente no Webots:`File > Open World`.