import sys
import numpy as np
import time
import Resolvedores_Jacobianos as singu
from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
from dashboard_client import DashboardClient
import roboticstoolbox as rtb

robot_ip = "127.0.0.1"

try:
    print("\nTentando conectar ao URSim...")
    rtde_r = RTDEReceiveInterface(robot_ip)
    rtde_c = RTDEControlInterface(robot_ip)
    dash = DashboardClient(robot_ip)
    dash.connect()
    robo_ur10 = rtb.models.UR10()
    print("\nConexão bem-sucedida!")
    
    # ---------------------------------------------------------------------------
    # CONFIGURAÇÃO DO PAR DE COORDENADAS (Singularidade Clássica de Punho no UR5)
    # ---------------------------------------------------------------------------
    # Posição inicial: Robô bem aberto e estendido para o lado
    q_inicial = [0.0, -1.2, -1.5, -0.4, 1.57, 0.0]
    
    print("\n[PREPARAÇÃO] Movendo o robô para a postura inicial de referência...")
    singu.popup_temporizado(dash, "Inicializando apresentacao. Movendo para posicao inicial saudavel...", 3.0)
    rtde_c.moveJ(q_inicial, 1.0, 1.0)
    time.sleep(1.0)
    
    # Lemos a posição real
    pose_inicial_real = rtde_r.getActualTCPPose()
    pose_final = list(pose_inicial_real)
    # ===========================================================================
    # DEFINIÇÃO DOS WAYPOINTS (PONTOS ALVO)
    # ===========================================================================
    # Criamos uma lista de deslocamentos relativos à posição inicial para testar o controlador
    deslocamentos = [
        [0.15, 0.0, 0.0, 0.0, 0.0, 0.0],   # Ponto 1: +15cm no X
        [0.15, 0.15, 0.0, 0.0, 0.0, 0.0],  # Ponto 2: +15cm no Y (mantendo X)
        [-0.10, 0.15, 0.0, 0.0, 0.0, 0.0], # Ponto 3: Recua o X, mantem Y
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # Ponto 4: Volta para a posição inicial exata
    ]
    
    pontos_alvo = []
    for deslocamento in deslocamentos:
        alvo = list(pose_inicial_real)
        for i in range(6):
            alvo[i] += deslocamento[i]
        pontos_alvo.append(alvo)
        
    resultados_comparativos = {}

    # ===========================================================================
    # EXECUÇÃO DO CONTROLADOR EM MALHA FECHADA
    # ===========================================================================
    print("\n" + "="*60)
    print("INICIANDO NAVEGAÇÃO POR WAYPOINTS COM CONTROLADOR")
    print("="*60)
    
    singu.resetar_robo(rtde_c, dash, q_inicial, "Preparando para iniciar navegação por multiplos waypoints...")

    telemetria_total = singu.iniciar_telemetria()
    tempo_inicio_global = time.time()
    kp = 5.0

    for indice, alvo in enumerate(pontos_alvo):
        nome_ponto = f"Waypoint_{indice+1}"
        print(f"\n---> Navegando para o {nome_ponto} ...")
        
        tempo_inicio_ponto = time.time()
        
        while True:
            q_atual = rtde_r.getActualQ()
            pose_atual_ursim = rtde_r.getActualTCPPose()
            
            erro_ursim_3d = np.array(alvo[:3]) - np.array(pose_atual_ursim[:3])
            distancia = np.linalg.norm(erro_ursim_3d)
            
            # Se chegou perto o suficiente, rompe o while para puxar o próximo alvo IMEDIATAMENTE
            if distancia < 0.008:
                print(f"[OK] {nome_ponto} atingido com sucesso (Erro: {distancia*1000:.2f} mm).")
                break
                
            if time.time() - tempo_inicio_ponto > 15.0:
                print(f"[AVISO] Timeout ao buscar o {nome_ponto} (Erro: {distancia*1000:.2f} mm).")
                break

            vetor_python = erro_ursim_3d.copy()
            vetor_python[0] = -vetor_python[0]
            vetor_python[1] = -vetor_python[1]
            
            v_desejado = kp * vetor_python
            
            VEL_MAX = 0.25
            norma_v = np.linalg.norm(v_desejado)
            if norma_v > VEL_MAX:
                v_desejado = v_desejado * (VEL_MAX / norma_v)

            J_completo = robo_ur10.jacob0(q_atual)
            w = np.sqrt(max(0, np.linalg.det(J_completo @ J_completo.T)))
            
            ZONA_DE_ALERTA = 0.07
            if w < ZONA_DE_ALERTA:
                lambda_sq = (1 - (w / ZONA_DE_ALERTA)**2) * 0.12
            else:
                lambda_sq = 0.0

            J_v = J_completo[:3, :]
            
            Identidade_3x3 = np.eye(3)
            J_dls_3d = J_v.T @ np.linalg.inv(J_v @ J_v.T + lambda_sq * Identidade_3x3)
            
            q_dot_primario = J_dls_3d @ v_desejado

            I_6x6 = np.eye(6)
            J_v_pinv = np.linalg.pinv(J_v)
            Projetor_Nulo = I_6x6 - (J_v_pinv @ J_v)
            
            K_postura = 0.8 
            erro_postura = np.array(q_inicial) - np.array(q_atual)
            q_dot_secundario = Projetor_Nulo @ (K_postura * erro_postura)
            
            q_dot_final = q_dot_primario + q_dot_secundario
            
            norma_qdot_final = np.linalg.norm(q_dot_final)
            if norma_qdot_final > 3.14: 
                q_dot_final = q_dot_final * (3.14 / norma_qdot_final)
                
            # Grava telemetria unificada (tempo contínuo desde o Waypoint 1)
            tempo_decorrido = time.time() - tempo_inicio_global
            telemetria_total["tempo"].append(tempo_decorrido)
            telemetria_total["w"].append(w)
            telemetria_total["qdot"].append(norma_qdot_final)
                
            rtde_c.speedJ(q_dot_final.tolist(), 0.5, 0.02)
            time.sleep(0.02)
            
    # Para o robô apenas quando todos os waypoints foram visitados
    rtde_c.speedStop()

    # ===========================================================================
    # GERAÇÃO DO RELATÓRIO CONTÍNUO
    # ===========================================================================
    print("\n" + "="*60)
    print("GERANDO GRÁFICO DA TELEMETRIA CONTÍNUA (TODOS OS WAYPOINTS)")
    print("="*60)
    singu.plotar_analise_cinematica(
        telemetria_total["tempo"], 
        telemetria_total["w"], 
        telemetria_total["qdot"], 
        "Navegação Contínua por Múltiplos Waypoints",
        "Trajetoria_Continua_Waypoints"
    )
    print("[INFO] Execução finalizada. Gráfico gerado!")

except Exception as e:
    print(f"Falha na execução: {e}")

finally:
    print("\nLimpando conexões de rede e encerrando com segurança...")
    if 'dash' in locals():
        try: dash.disconnect()
        except Exception: pass
    if 'rtde_c' in locals():
        try: rtde_c.speedStop(); rtde_c.disconnect()
        except Exception: pass
    if 'rtde_r' in locals():
        try: rtde_r.disconnect()
        except Exception: pass
    print("\nConexões fechadas. Pronto para reiniciar.\n")
