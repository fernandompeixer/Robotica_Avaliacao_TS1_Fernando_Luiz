# Controle de Singularidades Cinemáticas — UR10 (URSim)

Este repositório contém três scripts em Python que demonstram diferentes estratégias de controle de um robô **UR10** ao se aproximar de uma **singularidade cinemática de punho**, usando o simulador oficial da Universal Robots (**URSim**) via protocolo **RTDE**.

## Estrutura do repositório

| Arquivo | O que é |
|---|---|
| `Resolvedores_Jacobianos.py` | **Biblioteca/módulo** com todas as funções de controle, telemetria e geração de gráficos. Não é executado diretamente — é importado pelos outros dois scripts. |
| `Main_testes.py` | **Script executável** — roda os 3 testes comparativos (linear ingênuo, DLS, malha fechada com espaço nulo) em sequência e gera o gráfico comparativo final. |
| `Teste_controlador_xpontos.py` | **Script executável** — roda uma navegação contínua por 4 waypoints usando o controlador de malha fechada com espaço nulo, gerando telemetria única do trajeto inteiro. |

---

## 1. Pré-requisitos

### 1.1. Software necessário
- **Python 3.8+**
- **URSim** (simulador da Universal Robots) rodando localmente ou em uma VM/Docker acessível pela rede.
- Bibliotecas Python:
  ```bash
  pip install numpy matplotlib roboticstoolbox-python ur-rtde ur-dashboard-client
  ```
  > Os pacotes `rtde_receive`, `rtde_control` e `dashboard_client` vêm do pacote **ur_rtde** (biblioteca oficial da UR para controle em tempo real).

### 1.2. Rodando o URSim via Docker (UR10 CB3)

O jeito mais rápido de ter o simulador rodando é usando a imagem Docker oficial `universalrobots/ursim_cb3`, que é a série usada pelo controlador **CB3** (a mesma linha do seu `robo_ur10 = rtb.models.UR10()`).

#### a) Instalar o Docker
- **Windows/Mac:** baixe e instale o **Docker Desktop** em https://www.docker.com/products/docker-desktop/
- **Linux:**
  ```bash
  curl -fsSL https://get.docker.com | sh
  sudo usermod -aG docker $USER
  ```
  (depois disso, faça logout/login para o usuário entrar no grupo `docker`)

Confirme a instalação:
```bash
docker --version
```

#### b) Baixar a imagem do URSim (CB3)
```bash
docker pull universalrobots/ursim_cb3
```

#### c) Subir o container com o modelo UR10
A variável de ambiente `ROBOT_MODEL` define o modelo do braço (`UR3`, `UR5` ou `UR10` na linha CB3). As portas expostas são:
- `5900` → acesso via cliente VNC tradicional
- `6080` → acesso via navegador (noVNC)
- `29999` → Dashboard Server (usado por `DashboardClient`)
- `30001-30004` → interfaces RTDE/URScript (usadas por `RTDEReceiveInterface`/`RTDEControlInterface`)

```bash
docker run --rm -it \
  -p 5900:5900 \
  -p 6080:6080 \
  -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  -e ROBOT_MODEL=UR10 \
  --name ursim_ur10 \
  universalrobots/ursim_cb3
```

> No Windows (PowerShell), coloque tudo em uma linha só, sem as barras invertidas `\`.

Deixe esse terminal aberto — ele mostra os logs do container rodando em primeiro plano. Para encerrar, use `Ctrl+C` (a flag `--rm` já remove o container automaticamente ao parar).

#### d) Acessar a interface gráfica (Polyscope) pelo navegador
Com o container rodando, abra no navegador:
```
http://localhost:6080/vnc.html?host=localhost&port=6080
```
Clique em **Connect** (às vezes aparece um botão simples "Connect" no canto da página). Você verá a tela do Polyscope (a interface do robô real), já inicializando o simulador do UR10.

Na primeira vez, é comum precisar:
1. Aceitar/pular a tela de inicialização do controlador.
2. Ir em **"On/Off" → "ON" → "START"** dentro do Polyscope para ligar o robô virtual e liberar o modo **Remote Control**, que é o modo usado pelos scripts Python (via RTDE).

#### e) Confirmar o IP a usar nos scripts
Como o container está publicando as portas para `localhost`, mantenha nos scripts:
```python
robot_ip = "127.0.0.1"
```
Se você estiver rodando o Docker em outra máquina/VM da rede, troque `127.0.0.1` pelo IP dessa máquina.

---

## 2. Clonando o repositório

```bash
git clone <URL_DO_SEU_REPOSITORIO>
cd <NOME_DO_REPOSITORIO>
```

Certifique-se de que os três arquivos (`Resolvedores_Jacobianos.py`, `Main_testes.py`, `Teste_controlador_xpontos.py`) estão na **mesma pasta**, pois os dois scripts executáveis fazem `import Resolvedores_Jacobianos as singu`.

---

## 3. Configurando o IP do robô

Em `Main_testes.py` e em `Teste_controlador_xpontos.py`, ajuste a variável no topo do arquivo:

```python
robot_ip = "127.0.0.1"
```

Troque `"127.0.0.1"` pelo IP real do seu URSim, caso ele não esteja rodando na mesma máquina do script.

---

## 4. Como executar cada script

Com o URSim já rodando e em modo Remote Control, execute a partir da pasta do projeto:

### 4.1. `Main_testes.py`
```bash
python Main_testes.py
```
**O que ele roda:** compara três abordagens diferentes para o mesmo movimento cartesiano (mesmo ponto de partida e mesmo alvo):

1. **Teste 1 — Trajetória linear padrão:** tenta uma linha reta pura no espaço cartesiano. Deve **falhar de propósito**, detectando a singularidade (via manipulabilidade de Yoshikawa e velocidade de juntas) e freando o robô antes do colapso.
2. **Teste 2 — Pseudoinversa Amortecida (DLS):** usa amortecimento na inversão da Jacobiana para evitar velocidades absurdas de junta perto da singularidade, sacrificando um pouco a fidelidade da trajetória.
3. **Teste 3 — Controle em malha fechada com espaço nulo:** controlador proporcional em tempo real que desvia da singularidade e ainda usa o espaço nulo da Jacobiana para manter a postura das juntas próxima da configuração inicial.

Ao final, gera um **gráfico de barras comparativo** dos erros de posição/orientação dos três métodos (`Comparacao_Erros_Finais.png`, na pasta `Graficos_Resultados`).

### 4.2. `Teste_controlador_xpontos.py`
```bash
python Teste_controlador_xpontos.py
```
**O que ele roda:** usa **apenas** o controlador de malha fechada com espaço nulo (o mesmo do Teste 3 acima), mas aplicado a uma sequência de **4 waypoints** consecutivos (deslocamentos relativos à pose inicial), navegando de um ponto a outro sem parar o robô entre eles. Gera uma telemetria contínua do percurso inteiro e um único gráfico de manipulabilidade × velocidade de juntas ao longo do tempo (`Trajetoria_Continua_Waypoints.png`).

> Ambos os scripts salvam os gráficos automaticamente na pasta `Graficos_Resultados/`, criada no diretório de execução.

---

## 5. Encerramento seguro

Os dois scripts possuem um bloco `try/finally` que:
- Fecha popups e desbloqueia paradas de proteção do painel do robô;
- Para qualquer movimento em andamento (`speedStop`);
- Desconecta RTDE Receive, RTDE Control e Dashboard Client, mesmo em caso de erro ou `Ctrl+C`.

Você pode interromper qualquer teste a qualquer momento com `Ctrl+C` que o robô será parado com segurança.

---

## 6. Dica de organização no GitHub

Sugestão de estrutura de pastas para o repositório:

```
.
├── README.md
├── Resolvedores_Jacobianos.py
├── Main_testes.py
├── Teste_controlador_xpontos.py
└── Graficos_Resultados/     # gerado automaticamente ao rodar os testes
```

Adicione um `.gitignore` com:
```
Graficos_Resultados/
__pycache__/
*.pyc
```
para não versionar os gráficos gerados a cada execução.
