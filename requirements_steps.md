## 📋 Pré-requisitos (Instalação no Host)

Antes de começar, sua máquina (host) **precisa** ter os seguintes softwares instalados. Este guia foca em **Linux (Ubuntu/Debian)**.

### 1. Git

Se ainda não o tiver, instale o Git para clonar o repositório:
```bash
sudo apt update
sudo apt install git
```

**Verificação:**
```bash
git --version
```

### 2. Docker Engine

O Docker é o que nos permite construir e rodar o contêiner.
```bash
# Adiciona o repositório oficial do Docker
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Instala o Docker Engine
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

**Pós-instalação (Importante):** Adicione seu usuário ao grupo do Docker para não precisar usar `sudo` para cada comando:
```bash
sudo groupadd docker
sudo usermod -aG docker $USER
```

> ⚠️ **Atenção:** Você precisa **fechar e reabrir seu terminal** (ou reiniciar o computador) para que esta mudança tenha efeito.

**Verificação (após reabrir o terminal):**
```bash
docker --version
docker run hello-world
```

Se o comando `hello-world` funcionar sem erros, o Docker está instalado corretamente!

### 3. NVIDIA Container Toolkit (Opcional porém Recomendado para GPUs NVIDIA)

Se sua máquina possui uma GPU NVIDIA, esta etapa é **essencial** para que o contêiner possa usar a aceleração gráfica (GUI) e ter uma boa performance na simulação 3D.

**Primeiro, verifique se os drivers NVIDIA estão instalados:**
```bash
nvidia-smi
```

Se este comando mostrar informações sobre sua GPU, os drivers estão instalados. Caso contrário, você precisa instalar os drivers NVIDIA primeiro:
```bash
# Instala os drivers NVIDIA (se necessário)
sudo apt update
sudo apt install nvidia-driver-535  # ou a versão mais recente disponível
sudo reboot  # Reinicie após instalar os drivers
```

**Agora instale o NVIDIA Container Toolkit:**
```bash
# Adiciona o repositório da NVIDIA
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && \
    sudo apt-get update

# Instala o toolkit
sudo apt-get install -y nvidia-container-toolkit

# Configura o Docker para usar a GPU
sudo nvidia-ctk runtime configure --runtime=docker

# Reinicia o serviço do Docker para aplicar as mudanças
sudo systemctl restart docker
```

**Verificação:**
```bash
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi
```

Se você visualizar as informações da sua GPU dentro do contêiner, tudo está configurado corretamente! 🎉

---

### ✅ Checklist de Instalação

Antes de prosseguir para os próximos passos, certifique-se de que:

- [ ] Git está instalado (`git --version` funciona)
- [ ] Docker está instalado e funcionando (`docker run hello-world` funciona)
- [ ] Você pode executar comandos Docker sem `sudo`
- [ ] (Se tiver GPU NVIDIA) `nvidia-smi` mostra sua GPU
- [ ] (Se tiver GPU NVIDIA) O teste do contêiner NVIDIA foi bem-sucedido