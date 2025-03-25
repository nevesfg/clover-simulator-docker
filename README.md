# 🚁 Simulador Clover

[![ROS](https://i.imgur.com/FLrvMPo.png)](https://clover.coex.tech/en/simulation_native.html)
[![ROS](https://i.imgur.com/pJ296cP.png)](https://clover.coex.tech/en/simulation_native.html)

Um ambiente de simulação containerizado para drones Clover usando ROS Noetic e Gazebo 11. Este projeto simplifica a configuração e o uso do [Clover Simulator](https://clover.coex.tech/en/simulation_native.html) através de Docker, permitindo experimentos e desenvolvimento com drones virtuais em um ambiente isolado e reproduzível.

## 📋 Sumário

- [Sobre o Projeto](#-sobre-o-projeto)
- [Requisitos](#-requisitos)
- [Instalação](#-instalação)
- [Uso](#-uso)
  - [Inicialização do Simulador](#inicialização-do-simulador)
  - [Execução de Scripts](#execução-de-scripts)
  - [Comandos Úteis](#comandos-úteis)
- [Estrutura do Projeto](#-estrutura-do-projeto)
- [Desenvolvimento](#️-desenvolvimento)
- [Solução de Problemas](#-solução-de-problemas)
- [Recursos Adicionais](#-recursos-adicionais)

## 🔍 Sobre o Projeto

Este projeto encapsula o ambiente de simulação Clover em um container Docker, facilitando o desenvolvimento e teste de algoritmos para drones quadricópteros. O simulador utiliza ROS Noetic e Gazebo 11 para criar um ambiente virtual realista para o drone Clover.

A containerização resolve problemas comuns de dependências e configuração, permitindo que você comece a trabalhar com o simulador rapidamente, sem preocupações com conflitos de software ou configurações complexas.

## 💻 Requisitos

- Docker e Docker Compose instalados
- Sistema X11 (para visualização gráfica)
- Suporte a aceleração gráfica (Intel/AMD/NVIDIA)
- Pelo menos 4GB de RAM e 10GB de espaço em disco

## 🚀 Instalação e Configuração (PRIMEIRA INSTALAÇÃO AMBIENTE DOCKER)

### Explicação dos Arquivos de Configuração

Antes de começar, é importante entender os arquivos que compõem este projeto:

#### docker-compose.yml

Este arquivo define os serviços que compõem a aplicação e como eles devem ser executados:

```yaml
services:
  clover-sim:
    build:
      context: .
      dockerfile: Dockerfile
    container_name: clover-sim
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /home/anaju/Documentos/clover-simulator:/root/catkin_ws
    devices:
      - /dev/dri:/dev/dri
    tty: true
    stdin_open: true
```

**Elementos importantes:**
- `DISPLAY=${DISPLAY}`: Permite que aplicações gráficas do container sejam exibidas no host
- `/tmp/.X11-unix:/tmp/.X11-unix`: Monta o socket X11 para renderização gráfica
- `/home/anaju/Documentos/clover-simulator:/root/catkin_ws`: Monta um diretório local no workspace do ROS (catkin_ws)
- `/dev/dri:/dev/dri`: Compartilha o dispositivo de renderização para aceleração gráfica
- `tty: true` e `stdin_open: true`: Permitem interação com o terminal do container

#### Dockerfile

O Dockerfile contém as instruções para criar a imagem Docker:

```Dockerfile
FROM osrf/ros:noetic-desktop-full

# Instalar dependências necessárias, incluindo utilitários gráficos e bibliotecas OpenGL
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    python3-rosinstall-generator \
    python3-vcstool \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-cv-camera \
    ros-noetic-web-video-server \
    ros-noetic-rosbridge-suite \
    ros-noetic-joy \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf2-web-republisher \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-cmake-modules \
    libgeographic-dev \
    geographiclib-tools \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# Configurar o workspace do Clover
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Instalar dependências do projeto com rosdep
WORKDIR /root/catkin_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

# Compilar o workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Fazer backup do workspace compilado para um diretório interno
RUN cp -r /root/catkin_ws /root/catkin_ws_default

# Configurar o ambiente de execução
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Copiar o entrypoint script para a imagem
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Definir diretório de trabalho padrão
WORKDIR /root/catkin_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
```

**Principais etapas:**
1. Usa a imagem base do ROS Noetic
2. Instala as dependências necessárias, incluindo bibliotecas gráficas
3. Compila o workspace ROS catkin
4. Cria um backup do workspace compilado
5. Configura o ambiente ROS no container

#### entrypoint.sh

Este script é executado sempre que o container é iniciado:

```bash
#!/bin/bash
set -e

if [ ! -f /root/catkin_ws/devel/setup.bash ]; then
    echo "Workspace não encontrado em /root/catkin_ws. Restaurando backup..."
    cp -r /root/catkin_ws_default/* /root/catkin_ws/
fi

exec "$@"
```

**Função:** Verifica se o workspace ROS está configurado corretamente. Se não estiver (por exemplo, se o diretório montado estiver vazio), restaura o backup do workspace compilado.

### Preparação do Ambiente

1. Clone este repositório:

```bash
git clone https://github.com/nevesfg/clover-simulator-docker.git
cd clover-simulator
```

2. **Importante:** Ajuste o caminho do volume no arquivo `docker-compose.yml` para seu ambiente:

```yaml
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix
  - /caminho/para/seu/diretorio/local:/root/catkin_ws
```

Substitua `/caminho/para/seu/diretorio/local` pelo caminho absoluto do diretório onde você clonou o repositório.

3. Se você estiver usando um sistema operacional diferente, pode ser necessário ajustar as configurações de exibição:

   - **Linux:** Geralmente funciona sem alterações
   - **macOS:** Necessita de um servidor X11 como XQuartz
   - **Windows:** Necessita de um servidor X como VcXsrv ou Xming

### Construção e Inicialização do Container

1. Construa a imagem Docker (pode levar alguns minutos na primeira vez):

```bash
docker-compose build
```

2. Inicie o container:

```bash
docker-compose up -d
```

3. Verifique se o container está rodando:

```bash
docker ps -a
```

4. Configure as permissões de acesso para o servidor X11 no host(Terminal do seu PC):

```bash
xhost +local:docker
```

Você deve ver um container chamado `clover-sim` na lista.

## 🔧 Instalação da Biblioteca Clover no Container Docker (PRIMEIRA INSTALAÇÃO)

Após configurar o ambiente Docker, você precisará instalar a biblioteca Clover dentro do container. Estes passos devem ser realizados dentro do container após acessá-lo com `docker exec -it clover-sim /bin/bash`.

### Instalar o ROS

O ROS Noetic já está pré-instalado na imagem Docker. Se necessário, verifique a instalação:

```bash
rosversion -d
```

### Criar um workspace para a simulação

O workspace já está configurado no container em `/root/catkin_ws`. Verifique se está tudo correto:

```bash
cd /root/catkin_ws
ls -la
```

### Clonar os repositórios do Clover

Os repositórios principais já estão clonados no container, mas se precisar atualizá-los:

```bash
cd /root/catkin_ws/src
git pull
```

### Obter o código do PX4

Para ter a versão mais recente do PX4:

```bash
git clone --recursive --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git /root/catkin_ws/PX4-Autopilot
ln -s /root/catkin_ws/PX4-Autopilot /root/catkin_ws/src/
ln -s /root/catkin_ws/PX4-Autopilot/Tools/sitl_gazebo /root/catkin_ws/src/
ln -s /root/catkin_ws/PX4-Autopilot/mavlink /root/catkin_ws/src/
```

> **Dica:** Você pode usar uma versão mais recente do PX4, mas algo pode não funcionar como esperado nesse caso.

> **Observação:** Eu alterei os caminhos simbólicos e o local do clone do PX4-Autopilot para que os volumes do Docker persistam os arquivos, evitando perda de dados.


### Instalar os pré-requisitos do PX4

O PX4 vem com seu próprio script para instalação de dependências:

```bash
cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh
```

**⚠️ Aviso**: Durante a instalação, pode ocorrer o seguinte erro:
```
ERROR: pandas 2.0.3 has requirement numpy>=1.20.3; python_version < "3.10", but you'll have numpy 1.17.4 which is incompatible.
```

Não se preocupe com este erro. Deixe a instalação do restante das dependências terminar e, em seguida, atualize o numpy com:

```bash
python3 -m pip install --upgrade numpy
```

Verifique a versão após a atualização:

```bash
python3 -c "import numpy; print(numpy.__version__)"
```

Instale mais pacotes Python necessários:

```bash
pip3 install --user toml
```

### Adicionar o airframe do Clover

Adicione o airframe do Clover ao PX4 usando o comando:

```bash
ln -s /root/catkin_ws/src/clover/clover_simulation/airframes/* /root/catkin_ws/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

### Instalar os datasets do geographiclib

O pacote `mavros` requer que os datasets do geographiclib estejam presentes:

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### Compilar o simulador

Compile seu workspace:

```bash
cd ~/catkin_ws
catkin_make -j1
```

> **Observação:** A flag `-j1` significa que a compilação não usará processos paralelos, pois compilar com processos paralelos em um container pode esgotar a memória. Se seu sistema tiver memória suficiente, pode omitir essa flag.

### Executar o simulador

Para ter certeza de que tudo foi compilado corretamente, tente executar o simulador pela primeira vez:

```bash
roslaunch clover_simulation simulator.launch
```

Você pode testar voos autônomos usando os scripts de exemplo no diretório `~/catkin_ws/src/clover/clover/examples`.


## 🎮 Uso (Após instalação do ambiente docker e primeira execução!)

### Comandos Básicos

Abaixo estão os comandos básicos para trabalhar com o simulador sempre que iniciar o computador:

```bash
# Listar containers no computador
docker ps -a

# Iniciar o container
docker start clover-sim

# Acessar o terminal do container
docker exec -it clover-sim /bin/bash

# Dentro do container, inicializar o Gazebo
roslaunch clover_simulation simulator.launch

# Abra outro terminal, entre no container, para executar os scripts dentro do gazebo
docker exec -it clover-sim /bin/bash

# Navegar até a pasta de exemplos
cd /root/catkin_ws/src/clover/clover/examples

# Sair do terminal do container
exit

# Parar o container
docker stop clover-sim
```

### Preparando o Ambiente Após a Instalação

Antes de usar o simulador pela primeira vez ou após mover para um novo computador:

1. Verifique se o Docker está corretamente configurado:

```bash
docker --version
docker-compose --version
```

2. Configure as permissões de acesso para o servidor X11:

```bash
xhost +local:docker
```

### Inicialização do Simulador

1. Inicie o container (se não estiver rodando):

```bash
docker start clover-sim
```

2. Acesse o terminal do container:

```bash
docker exec -it clover-sim /bin/bash
```

3. Inicie o simulador Gazebo com o modelo Clover:

```bash
roslaunch clover_simulation simulator.launch
```

O Gazebo deve abrir mostrando o ambiente de simulação com o drone Clover.

### Execução de Scripts

Os scripts de exemplo estão localizados em `/root/catkin_ws/src/clover/clover/examples`. Para executar um script:

1. Navegue até a pasta de exemplos:

```bash
cd /root/catkin_ws/src/clover/clover/examples
```

2. Execute um script (por exemplo, para decolagem simples):

```bash
python3 flight.py
```

### Comandos Úteis

| Comando | Descrição |
|---------|-----------|
| `docker ps -a` | Lista todos os containers |
| `docker start clover-sim` | Inicia o container do simulador |
| `docker exec -it clover-sim /bin/bash` | Abre um terminal no container |
| `exit` | Sai do terminal do container |
| `docker stop clover-sim` | Para o container do simulador |
| `docker-compose down` | Remove o container (preservando os arquivos) |
| `docker-compose up -d` | Recria e inicia o container |
| `xhost +local:docker` | Permite que o Docker acesse o servidor X11 |

## 📁 Estrutura do Projeto

```
clover-simulator/
├── docker-compose.yml     # Configuração do Docker Compose
├── Dockerfile             # Instruções para construir a imagem
├── entrypoint.sh          # Script de inicialização do container
└── README.md              # Este arquivo
```

## 🛠️ Desenvolvimento

Para desenvolver seus próprios scripts para o drone Clover:

1. Crie seus scripts dentro da pasta `/root/catkin_ws` no container.
2. Os scripts podem ser escritos em Python e usar a API ROS do Clover.
3. Você pode montar um diretório local no container para facilitar o desenvolvimento (já configurado no docker-compose.yml).

Exemplo de script simples para decolagem e pouso:

```python
#!/usr/bin/env python3

import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight_test')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

# Takeoff to 1.5 meters
navigate(x=0, y=0, z=1.5, frame_id='body', auto_arm=True)
rospy.sleep(5)

# Land
land()
```

## ⚠️ Solução de Problemas ( CASO VC TENHA ALGUM DESSES PROBLEMAS!)

### Portabilidade Entre Computadores

Ao usar este ambiente em um computador diferente, você pode encontrar os seguintes problemas:

#### 1. Problema com Caminhos de Diretórios

**Sintoma:** Erros de "volume não encontrado" ou workspace vazio

**Solução:** Edite o `docker-compose.yml` para atualizar o caminho do volume montado:

```yaml
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix
  - /caminho/no/novo/computador:/root/catkin_ws
```

#### 2. Problemas de Permissão

**Sintoma:** Erros de permissão ao acessar arquivos ou diretórios

**Solução:** Ajuste as permissões no host:

```bash
sudo chown -R $USER:$USER /caminho/no/novo/computador
chmod -R 755 /caminho/no/novo/computador
```

#### 3. Problemas com Grupos Docker

**Sintoma:** Necessidade de usar `sudo` com comandos Docker

**Solução:** Adicione seu usuário ao grupo docker:

```bash
sudo usermod -aG docker $USER
# Faça logout e login novamente para aplicar
```

### Problemas de Exibição Gráfica

Se o Gazebo não abrir ou ocorrerem erros gráficos:

```bash
# Verifique se o X11 está configurado corretamente
echo $DISPLAY

# Permita conexões X11 do container
xhost +local:docker

# Teste a aceleração gráfica no container
docker exec -it clover-sim glxinfo | grep "direct rendering"
```

O resultado deve ser "direct rendering: Yes".

#### Em caso de erro "Cannot connect to X server"

Adicione esta linha antes de iniciar o container:

```bash
xhost +local:docker
```

### Problemas de Compilação

Se o workspace não compilar corretamente:

```bash
# Recompile o workspace
cd /root/catkin_ws
catkin_make clean
catkin_make
```

### Container Não Inicia

Se o container não iniciar:

```bash
# Verifique logs do container
docker logs clover-sim

# Remova e recrie o container
docker-compose down
docker-compose up -d
```

### Erros de Acesso ao Dispositivo Gráfico

Se ocorrerem erros relacionados ao acesso ao dispositivo `/dev/dri`:

```bash
# Verifique as permissões do dispositivo
ls -la /dev/dri

# Adicione seu usuário ao grupo video (se necessário)
sudo usermod -aG video $USER
# Faça logout e login novamente para aplicar
```

## 📚 Recursos Adicionais

- [Documentação oficial do Clover](https://clover.coex.tech/en/)
- [Documentação do simulador nativo Clover](https://clover.coex.tech/en/simulation_native.html)
- [Tutoriais de ROS](http://wiki.ros.org/ROS/Tutorials)
- [Tutoriais de Gazebo](http://gazebosim.org/tutorials)
- [Referência da API do Clover](https://clover.coex.tech/en/programming.html)

---

📝 **Desenvolvido por:** [Victor Neves](https://www.linkedin.com/in/nevesfg/)

💡 **Inspirado no projeto:** [CopterExpress/clover](https://github.com/CopterExpress/clover)
