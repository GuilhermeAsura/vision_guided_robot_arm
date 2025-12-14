# Keyboard Check (ROS2 Package)

Este pacote ROS2 Humble é responsável pela captura de entrada de teclado (teleoperação) e publicação de comandos padronizados para o controle do braço robótico e outros componentes da simulação.

Ele implementa leitura de terminal em modo *raw* (não bloqueante), permitindo o controle em tempo real sem a necessidade de pressionar "Enter" após cada comando.

## 📂 Estrutura do Pacote

```text
keyboard_check/
├── keyboard_check/
│   ├── __init__.py
│   ├── keyboard_listener.py   # Nó de depuração (Subscriber)
│   └── keyboard_publisher.py  # Nó principal de captura (Publisher)
├── package.xml                # Definição de dependências
├── setup.py                   # Configuração de instalação Python
└── setup.cfg
````

## ⚙️ Dependências

Este pacote foi desenvolvido em **Python** e utiliza as seguintes dependências ROS2 e de sistema:

- `rclpy`: Client Library do ROS2 para Python.
    
- `std_msgs`: Para mensagens do tipo `Int32`.
    
- **Bibliotecas Python (Standard):** `sys`, `termios`, `tty`, `select` (para manipulação do terminal Linux).
    

## 🚀 Como Executar

Certifique-se de ter compilado o workspace e carregado o ambiente:


``` Bash
cd ~/trainee_ws
colcon build --packages-select keyboard_check
source install/setup.bash
```

### 1. Iniciar o Publisher (Controle)

Este nó captura as teclas e publica no tópico `/keyboard_input`.

Bash

``` Bash
ros2 run keyboard_check keyboard_publisher
```

> **Nota:** Mantenha o terminal deste nó focado para que as teclas sejam capturadas.

### 2. Iniciar o Listener (Depuração - Opcional)

Para verificar se os comandos estão sendo enviados corretamente:

Bash

``` Bash
ros2 run keyboard_check keyboard_listener
```

## 📡 Tópicos e Mensagens

|**Tópico**|**Tipo de Mensagem**|**Direção**|**Descrição**|
|---|---|---|---|
|`/keyboard_input`|`std_msgs/msg/Int32`|Pub|Envia um código inteiro correspondente à tecla pressionada.|

## 🗺️ Mapeamento de Teclas

O nó `keyboard_publisher` converte teclas físicas em códigos inteiros para facilitar o processamento pelos controladores do robô.

|**Categoria**|**Tecla**|**Código (Int32)**|**Função Sugerida**|
|---|---|---|---|
|**Movimentação (WASD)**|`W`|`0`|Mover Eixo/Base Frente|
||`S`|`1`|Mover Eixo/Base Trás|
||`D`|`2`|Mover Eixo/Base Direita|
||`A`|`3`|Mover Eixo/Base Esquerda|
|**Setas Direcionais**|`UP` (⬆️)|`4`|Elevar/Avançar|
||`DOWN` (⬇️)|`5`|Baixar/Recuar|
||`RIGHT` (➡️)|`6`|Rotacionar Direita|
||`LEFT` (⬅️)|`7`|Rotacionar Esquerda|
|**Seleção de Juntas**|`1` - `6`|`101` - `106`|Selecionar Junta 1 a 6|
|**Atuadores**|`7`|`107`|Controle da Garra (Abrir)|
||`8`|`108`|Controle da Garra (Fechar)|
|**Funções Especiais**|`R`|`201`|Reset / Recalibrar|
||`T`|`202`|Função Auxiliar (Troca de Modo)|
|**Outros**|Qualquer outra|`-1`|Tecla não mapeada|

## 🛠️ Detalhes de Implementação

- **Manipulação de Terminal:** O script utiliza `termios` e `tty` para alterar as configurações do terminal (`stdin`) para o modo `cbreak`. Isso permite a leitura caractere a caractere.
    
- **Sequências ANSI:** O código trata sequências de escape (iniciadas por `\x1b`) para identificar corretamente as setas direcionais, que enviam múltiplos bytes.
