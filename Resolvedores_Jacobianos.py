import numpy as np
import time
import random
from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import roboticstoolbox as rtb

def resetar_robo(rtde_c, dash, q_inicial, mensagem="Retornando para posicao inicial saudavel..."):
    """
    Reseta a posição do robô para a postura inicial de referência, 
    limpando alarmes de segurança e destravando o URSim se necessário.
    """
    print("\n[PREPARAÇÃO] Limpando alarmes e movendo o robô para a postura de referência...")
    
    # 1. Tenta limpar alarmes e destravar o painel de segurança
    try:
        dash.closePopup()
        dash.closeSafetyPopup()
        dash.unlockProtectiveStop()
        time.sleep(1.0)
        rtde_c.reuploadScript()
        time.sleep(0.5)
    except Exception:
        pass
        
    # 2. Exibe o aviso no painel
    if mensagem:
        popup_temporizado(dash, mensagem, 3.0)
        
    # 3. Tenta mover para a posição
    sucesso = rtde_c.moveJ(q_inicial, 1.0, 1.0)
    
    # 4. Fallback de reconexão se o robô rejeitar o primeiro comando (comum após colapsos)
    if not sucesso:
        print("[ERRO] moveJ falhou ao tentar voltar! Reconectando e tentando novamente...")
        time.sleep(1.0)
        rtde_c.reconnect()
        rtde_c.moveJ(q_inicial, 1.0, 1.0)
        
    time.sleep(1.0)


def explorar_trajetoria_e_gravar(rtde_c, rtde_r, modelo_cinematico, q_inicial, pose_cartesiana_final):
    """
    Move o robô em linha reta (no espaço cartesiano) até a pose alvo usando controle de
    velocidade, acompanhando a cada ciclo a manipulabilidade (o quão longe o robô está de uma
    singularidade) e a velocidade de juntas que essa linha reta estaria exigindo; assim que
    qualquer um dos dois indicadores piora demais, a função entende que uma singularidade está
    próxima, freia o robô imediatamente e devolve o ponto exato onde a falha ocorreu — servindo
    para demonstrar o colapso das juntas utilizando apenas o deslocamento linear na movimentação do
    braço.
    """
    print("\n[INFO] Iniciando exploracao de trajetoria...\n")

    rtde_c.moveJ(q_inicial, 1.0, 1.0)
    time.sleep(1.0)

    telemetria_exploracao = iniciar_telemetria()

    LIMIAR_FALHA_W = 0.07
    LIMITE_VEL_JUNTA = 2.5

    try:
        while True:

            # Le a posicao real do robo (juntas e pose do TCP) direto do simulador
            q_atual = rtde_r.getActualQ()
            pose_atual_ursim = rtde_r.getActualTCPPose()

            # Erro cartesiano: distancia (em xyz) entre onde o robo esta e onde ele precisa chegar
            erro_ursim_3d = np.array(pose_cartesiana_final[:3]) - np.array(pose_atual_ursim[:3])
            distancia = np.linalg.norm(erro_ursim_3d)

            if distancia < 0.001:
                rtde_c.speedStop()
                print("[OK] Trajetoria concluida sem encontrar singularidades.")
                return "sucesso", telemetria_exploracao, pose_atual_ursim

            # Limita a velocidade cartesiana de aproximacao a 10 cm/s
            vetor_comando_ursim = erro_ursim_3d.copy()
            if distancia > 0.1:
                vetor_comando_ursim = vetor_comando_ursim * (0.1 / distancia)

            # O modelo cinematico em Python usa X e Y invertidos em relacao ao frame do URSim/RTDE
            vetor_python = erro_ursim_3d.copy()
            vetor_python[0] = -vetor_python[0]
            vetor_python[1] = -vetor_python[1]

            # Jacobiana atual e manipulabilidade de Yoshikawa (quanto mais perto de zero, mais perto da singularidade)
            J_atual = modelo_cinematico.jacob0(q_atual)
            w = np.sqrt(max(0, np.linalg.det(J_atual @ J_atual.T)))

            J_pinv = np.linalg.pinv(J_atual)
            # O erro so tem componente de translacao (3 valores); completa com zeros para multiplicar pela Jacobiana 6x6
            vetor_python_6d = np.zeros(6)
            vetor_python_6d[:3] = vetor_python

            # Velocidade de juntas que seria necessaria para de fato seguir essa linha reta
            q_dot_teorico = J_pinv @ vetor_python_6d
            norma_velocidade_juntas = np.linalg.norm(q_dot_teorico)

            gravar_telemetria(telemetria_exploracao, w, norma_velocidade_juntas)

            # Gatilho de seguranca: manipulabilidade baixa OU juntas "explodindo" de velocidade = singularidade proxima
            if w < LIMIAR_FALHA_W or norma_velocidade_juntas > LIMITE_VEL_JUNTA:
                rtde_c.speedStop()
                print("\n[ALERTA] PERIGO: Singularidade atingida! Parando o robo.")
                plotar_analise_cinematica(telemetria_exploracao["tempo"], telemetria_exploracao["w"], telemetria_exploracao["qdot"], "Movimento_Linear_Strict")
                dicionario_falha = {
                    "q_inicial_critico": q_atual,
                    "vetor_cartesiano_ofensor": erro_ursim_3d.tolist(),
                    "manipulabilidade": round(w, 4)
                }
                return "singularidade", dicionario_falha, pose_atual_ursim

            # speedL trabalha no frame fisico do robo, por isso usamos o vetor no frame do URSim (nao invertido)
            vetor_final_speedL = np.zeros(6)
            vetor_final_speedL[:3] = vetor_comando_ursim
            rtde_c.speedL(vetor_final_speedL.tolist(), 0.5)
            time.sleep(0.02)

    except KeyboardInterrupt:
        # Permite interromper o teste com Ctrl+C sem deixar o robo em movimento
        rtde_c.speedStop()
        print("\nOperacao abortada pelo usuario.")
        return "abortado", None


def Pseudoinvers_amortecida(rtde_c, rtde_r, modelo_cinematico, q_inicial, pose_cartesiana_final):
    """
    Guia o robô até a mesma pose alvo, mas sem exigir uma trajetória perfeitamente reta: usa a
    Pseudoinversa Amortecida (DLS), uma técnica que evita o problema de perto de uma singularidade
    a Jacobiana normal pedir velocidades de junta absurdamente altas. A ideia é somar um fator de
    "amortecimento" à inversão da Jacobiana, o que suaviza a resposta às custas de seguir o
    caminho ideal com menos fidelidade — na prática, o robô desacelera e se desvia levemente da
    linha reta para não sobrecarregar os motores. Se, mesmo assim, a velocidade calculada continuar
    baixa por muitos ciclos seguidos, a função entende que o robô realmente travou (não vai
    progredir mais) e para em segurança, em vez de ficar tentando indefinidamente.
    """
    print("[INFO] Iniciando trajetoria com Pseudoinversa Amortecida (DLS)...")

    rtde_c.moveJ(q_inicial, 1.0, 1.0)
    time.sleep(1.0)

    ZONA_DE_ALERTA = 0.1
    tempo_inicio = time.time()

    telemetria_dls = iniciar_telemetria()
    ciclos_estagnado = 0

    try:
        while True:
            q_atual = rtde_r.getActualQ()
            pose_atual_ursim = rtde_r.getActualTCPPose()

            erro_ursim_3d = np.array(pose_cartesiana_final[:3]) - np.array(pose_atual_ursim[:3])
            distancia = np.linalg.norm(erro_ursim_3d)

            if distancia < 0.04:
                rtde_c.speedStop()
                plotar_analise_cinematica(telemetria_dls["tempo"], telemetria_dls["w"], telemetria_dls["qdot"], "DLS_PseudoInversa_Amoretecida")
                return "sucesso", telemetria_dls, pose_atual_ursim

            # Mesma conversao de frame usada no teste linear (X e Y invertidos para o modelo Python)
            vetor_python = erro_ursim_3d.copy()
            vetor_python[0] = -vetor_python[0]
            vetor_python[1] = -vetor_python[1]

            if distancia > 0.25:
                vetor_python = vetor_python * (0.25 / distancia)

            J_atual = modelo_cinematico.jacob0(q_atual)
            w = np.sqrt(max(0, np.linalg.det(J_atual @ J_atual.T)))

            # Quanto mais perto da zona de alerta, maior o amortecimento aplicado (lambda_sq)
            ZONA_DE_ALERTA = 0.07
            if w < ZONA_DE_ALERTA:
                lambda_sq = (1 - (w / ZONA_DE_ALERTA) ** 2) * 0.04
            else:
                lambda_sq = 0.0

            # So a parte de translacao da Jacobiana (3 linhas) entra na pseudoinversa amortecida
            J_v = J_atual[:3, :]
            Identidade_3x3 = np.eye(3)
            J_dls_3d = J_v.T @ np.linalg.inv(J_v @ J_v.T + lambda_sq * Identidade_3x3)

            q_dot = J_dls_3d @ vetor_python
            norma_qdot = np.linalg.norm(q_dot)

            gravar_telemetria(telemetria_dls, w, norma_qdot)

            # Conta quantos ciclos seguidos a velocidade ficou baixa demais (sinal de estagnacao real, nao apenas um pico momentaneo)
            if norma_qdot < 0.02:
                ciclos_estagnado += 1
            else:
                ciclos_estagnado = 0

            if ciclos_estagnado >= 20:
                rtde_c.speedStop()
                plotar_analise_cinematica(telemetria_dls["tempo"], telemetria_dls["w"], telemetria_dls["qdot"], "DLS")
                return "estagnado", telemetria_dls, pose_atual_ursim

            rtde_c.speedJ(q_dot.tolist(), 0.5, 0.02)
            time.sleep(0.02)
    except KeyboardInterrupt:
        rtde_c.speedStop()
        print("\nOperacao abortada pelo usuario.")
        return "abortado", None


def controlador_cartesiano_realtime(rtde_c, rtde_r, modelo_cinematico, q_inicial, pose_cartesiana_final, kp, titulo_grafico="Teste3_Espaco_Nulo"):
    """
    Controla o robô em malha fechada até a pose alvo combinando duas tarefas ao mesmo tempo. A
    tarefa primária é chegar na posição desejada usando a mesma ideia de Pseudoinversa Amortecida
    do DLS, para não sofrer perto de singularidades. A tarefa secundária aproveita o "espaço nulo"
    da Jacobiana — os graus de liberdade que sobram e que não afetam a posição do TCP — para puxar
    as juntas de volta à postura inicial (q_inicial), evitando que o robô fique torto ou em
    posturas estranhas enquanto desvia da singularidade. Por rodar em malha fechada, a cada ciclo
    ela relê a posição real do robô e recalcula o comando, corrigindo o erro continuamente até
    alcançar o alvo ou estourar o tempo limite.
    """
    telemetria_espaco_nulo = iniciar_telemetria()
    tempo_inicio = time.time()

    try:
        while True:
            q_atual = rtde_r.getActualQ()
            pose_atual_ursim = rtde_r.getActualTCPPose()

            erro_ursim_3d = np.array(pose_cartesiana_final[:3]) - np.array(pose_atual_ursim[:3])
            distancia = np.linalg.norm(erro_ursim_3d)

            if distancia < 0.008:
                rtde_c.speedStop()
                plotar_analise_cinematica(telemetria_espaco_nulo["tempo"], telemetria_espaco_nulo["w"], telemetria_espaco_nulo["qdot"], f"Controlador - {titulo_grafico}", titulo_grafico)
                return "CONCLUIDO", distancia, pose_atual_ursim

            if time.time() - tempo_inicio > 15.0:
                rtde_c.speedStop()
                plotar_analise_cinematica(telemetria_espaco_nulo["tempo"], telemetria_espaco_nulo["w"], telemetria_espaco_nulo["qdot"], f"Timeout - {titulo_grafico}", f"{titulo_grafico}_Timeout")
                return "TIMEOUT", distancia, pose_atual_ursim

            # Mesma conversao de frame das outras funcoes (X e Y invertidos para o modelo Python)
            vetor_python = erro_ursim_3d.copy()
            vetor_python[0] = -vetor_python[0]
            vetor_python[1] = -vetor_python[1]

            # Controle proporcional: velocidade desejada e proporcional ao erro (ganho kp), com limite maximo de velocidade
            v_desejado = kp * vetor_python

            VEL_MAX = 0.25
            norma_v = np.linalg.norm(v_desejado)
            if norma_v > VEL_MAX:
                v_desejado = v_desejado * (VEL_MAX / norma_v)

            J_completo = modelo_cinematico.jacob0(q_atual)
            w = np.sqrt(max(0, np.linalg.det(J_completo @ J_completo.T)))

            # Amortecimento cresce conforme a manipulabilidade cai (mesma logica de suavizacao do DLS)
            ZONA_DE_ALERTA = 0.07
            if w < ZONA_DE_ALERTA:
                lambda_sq = (1 - (w / ZONA_DE_ALERTA) ** 2) * 0.12
            else:
                lambda_sq = 0.0

            J_v = J_completo[:3, :]

            Identidade_3x3 = np.eye(3)
            J_dls_3d = J_v.T @ np.linalg.inv(J_v @ J_v.T + lambda_sq * Identidade_3x3)

            # Tarefa primaria: velocidade de juntas que aproxima o TCP do alvo
            q_dot_primario = J_dls_3d @ v_desejado

            # Projetor do espaco nulo (usa a pseudoinversa "pura", nao a amortecida, para nao vazar erro para a posicao)
            I_6x6 = np.eye(6)
            J_v_pinv = np.linalg.pinv(J_v)
            Projetor_Nulo = I_6x6 - (J_v_pinv @ J_v)

            # Tarefa secundaria: dentro do espaco nulo, tenta voltar a postura de referencia (q_inicial) sem mexer na posicao do TCP
            K_postura = 0.8
            erro_postura = np.array(q_inicial) - np.array(q_atual)
            q_dot_secundario = Projetor_Nulo @ (K_postura * erro_postura)

            # Soma as duas tarefas: anda em direcao ao alvo e ajeita a postura ao mesmo tempo
            q_dot_final = q_dot_primario + q_dot_secundario

            # Protecao contra saturacao extrema das juntas (limite de ~180 graus/s)
            norma_qdot_final = np.linalg.norm(q_dot_final)
            if norma_qdot_final > 3.14:
                q_dot_final = q_dot_final * (3.14 / norma_qdot_final)

            gravar_telemetria(telemetria_espaco_nulo, w, norma_qdot_final)

            # Envia o comando de velocidade; a funcao sera chamada de novo no proximo ciclo do while
            rtde_c.speedJ(q_dot_final.tolist(), 0.5, 0.02)
            time.sleep(0.02)

    except KeyboardInterrupt:
        rtde_c.speedStop()
        print("\nOperacao abortada pelo usuario.")
        return "ABORTADO", distancia


def popup_temporizado(dash_client, mensagem, tempo_segundos=5.0):
    """
    Exibe um popup de aviso na tela do URSim, aguarda o tempo definido para que a mensagem possa
    ser lida e depois fecha o popup automaticamente, liberando o próximo movimento do robô sem
    precisar de intervenção manual.
    """
    print(f"\n" + "-"*60)
    print(f"[URSIM] Exibindo aviso por {tempo_segundos} segundos...")
    print(f"Mensagem: '{mensagem}'")
    print("-" * 60)

    try:
        # Abre o pop-up na tela do robo
        dash_client.popup(mensagem)

        # Congela a execucao do Python pelo tempo definido, dando tempo de leitura
        time.sleep(tempo_segundos)

        # Fecha o pop-up automaticamente, sem precisar de clique manual
        dash_client.closePopup()

    except Exception as e:
        print(f"Aviso do Dashboard: {e}")

    print("[URSIM] Tempo esgotado! Iniciando o movimento...\n")


import os
import matplotlib.pyplot as plt


def plotar_analise_cinematica(historico_tempo, historico_w, historico_qdot, titulo_teste, nome_arquivo=None):
    """
    Gera e salva um gráfico com dois eixos verticais mostrando, ao longo do tempo de execução, a
    manipulabilidade do robô (o quão perto ele está de uma singularidade) e a norma da velocidade
    de juntas exigida, permitindo visualizar de forma conjunta o momento em que o robô se aproxima
    de uma configuração crítica; o gráfico é salvo em PNG na pasta "Graficos_Resultados".
    """
    if nome_arquivo is None:
        nome_arquivo = titulo_teste.replace(" ", "_")

    pasta_resultados = "Graficos_Resultados"
    if not os.path.exists(pasta_resultados):
        os.makedirs(pasta_resultados)

    print(f"[GRÁFICO] Gerando análise visual para: {titulo_teste}...")

    fig, ax1 = plt.subplots(figsize=(10, 5))

    # --- Eixo Y Esquerdo: Manipulabilidade (Saúde Geométrica) ---
    cor_w = '#1f77b4'  # Azul
    ax1.set_xlabel('Tempo de Execução (s)', fontweight='bold')
    ax1.set_ylabel('Manipulabilidade (w)', color=cor_w, fontweight='bold')
    ax1.plot(historico_tempo, historico_w, color=cor_w, linewidth=2.5, label='w (Yoshikawa)')
    ax1.tick_params(axis='y', labelcolor=cor_w)
    ax1.axhline(y=0.02, color=cor_w, linestyle='--', alpha=0.3, label='Limiar Crítico')

    # --- Eixo Y Direito: Esforço das Juntas (Norma da Velocidade) ---
    ax2 = ax1.twinx()
    cor_q = '#d62728'  # Vermelho
    ax2.set_ylabel('Norma Vel. Juntas ||q_dot|| (rad/s)', color=cor_q, fontweight='bold')
    ax2.plot(historico_tempo, historico_qdot, color=cor_q, linewidth=2.5, label='||q_dot||')
    ax2.tick_params(axis='y', labelcolor=cor_q)

    # Configuracoes esteticas
    plt.title(f'Análise de Desempenho Cinemático:\n{titulo_teste}', fontsize=14, pad=15)
    fig.tight_layout()
    plt.grid(True, linestyle=':', alpha=0.6)

    # Salva o grafico na pasta de resultados antes de exibir
    caminho_completo = os.path.join(pasta_resultados, f"{nome_arquivo}.png")
    plt.savefig(caminho_completo, dpi=300, bbox_inches='tight')
    print(f"[GRÁFICO] Salvo com sucesso em: {caminho_completo}")

    # Exibe o grafico e pausa o terminal ate a janela ser fechada
    plt.show()


def iniciar_telemetria():
    """
    Cria a "caixa preta" vazia (listas de tempo, manipulabilidade e velocidade de juntas) e marca
    o instante zero; deve ser chamada logo antes de entrar no laço de controle de qualquer teste.
    """
    return {
        "tempo": [],
        "w": [],
        "qdot": [],
        "tempo_inicio": time.time()
    }


def gravar_telemetria(caixa_preta, w, q_dot):
    """
    Tira uma "foto" do instante atual (tempo decorrido, manipulabilidade e norma da velocidade de
    juntas) e guarda na caixa preta; deve ser chamada uma vez por ciclo do laço de controle.
    """
    tempo_decorrido = time.time() - caixa_preta["tempo_inicio"]
    norma_velocidade = np.linalg.norm(q_dot)

    caixa_preta["tempo"].append(tempo_decorrido)
    caixa_preta["w"].append(w)
    caixa_preta["qdot"].append(norma_velocidade)


def plotar_comparacao_erros(resultados_poses, pose_final_alvo):
    """
    Gera um gráfico de barras comparando, para cada um dos métodos testados, o erro final de
    posição (em milímetros) e de orientação (em graus) em relação à pose alvo, permitindo avaliar
    visualmente qual estratégia chegou mais perto do objetivo e qual "custo" (em precisão) cada
    uma pagou para evitar a singularidade; o gráfico é salvo em PNG na pasta "Graficos_Resultados".
    """
    import os
    import matplotlib.pyplot as plt

    nomes = []
    erros_posicao = []
    erros_orientacao = []

    alvo_pos = np.array(pose_final_alvo[:3])
    alvo_ori = np.array(pose_final_alvo[3:])

    for nome, pose_real in resultados_poses.items():
        if pose_real is None:
            continue

        real_pos = np.array(pose_real[:3])
        real_ori = np.array(pose_real[3:])

        # Erro de Posicao em milimetros
        erro_pos_mm = np.linalg.norm(alvo_pos - real_pos) * 1000.0

        # Erro de Orientacao em graus
        erro_ori_rad = np.linalg.norm(alvo_ori - real_ori)
        erro_ori_deg = np.degrees(erro_ori_rad)

        nomes.append(nome)
        erros_posicao.append(erro_pos_mm)
        erros_orientacao.append(erro_ori_deg)

    if not nomes:
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Grafico de Posicao
    ax1.bar(nomes, erros_posicao, color=['#d62728', '#ff7f0e', '#2ca02c'][:len(nomes)])
    ax1.set_title('Erro de Posição Final (mm)', fontweight='bold')
    ax1.set_ylabel('Erro Absoluto (mm)')
    for i, v in enumerate(erros_posicao):
        ax1.text(i, v + (max(erros_posicao) * 0.02), f"{v:.1f}", ha='center', fontweight='bold')

    # Grafico de Orientacao
    ax2.bar(nomes, erros_orientacao, color=['#9467bd', '#8c564b', '#17becf'][:len(nomes)])
    ax2.set_title('Erro de Orientação Final (graus)', fontweight='bold')
    ax2.set_ylabel('Erro Absoluto (graus)')
    for i, v in enumerate(erros_orientacao):
        ax2.text(i, v + (max(erros_orientacao) * 0.02), f"{v:.1f}", ha='center', fontweight='bold')

    plt.suptitle("Comparação de Precisão dos Algoritmos (Alvo Inalcançável)", fontsize=14, y=1.05)
    fig.tight_layout()

    pasta_resultados = "Graficos_Resultados"
    if not os.path.exists(pasta_resultados):
        os.makedirs(pasta_resultados)

    caminho_completo = os.path.join(pasta_resultados, "Comparacao_Erros_Finais.png")
    plt.savefig(caminho_completo, dpi=300, bbox_inches='tight')
    print(f"[GRÁFICO] Comparativo de Erros salvo em: {caminho_completo}")
    plt.show()