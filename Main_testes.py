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
    #Configurações de conexão com o simulador URSim e obtenção de parametros do UR10.
    print("\nTentando conectar ao URSim...")
    rtde_r = RTDEReceiveInterface(robot_ip)
    rtde_c = RTDEControlInterface(robot_ip)
    dash = DashboardClient(robot_ip)
    dash.connect()
    robo_ur10 = rtb.models.UR10()
    print("\nConexão bem-sucedida!")
    
    # Configuração do par de coordenadas (Singularidade Clássica de Punho no UR5)
    # Posicao inicial: Robô bem aberto e estendido para o lado (pose de referencia para iniciar o movimento)
    q_inicial = [0.0, -1.2, -1.5, -0.4, 1.57, 0.0]
    
    singu.resetar_robo(rtde_c, dash, q_inicial, "Inicializando apresentacao. Movendo para posicao inicial saudavel...")
    
    # Lê-se a posição inicial real
    pose_inicial_real = rtde_r.getActualTCPPose()
    # Define-se a posição final a partir da inicial
    pose_final = list(pose_inicial_real)
    pose_final[0] -= 0.30 
    pose_final[1] += 0.20
    
    # Dicionário para armazenar os resultados comparativos - Plotar um gráfico de erros no final de todas as simulações.
    resultados_comparativos = {}

    # ===========================================================================
    # TESTE 1: EXPLORAÇÃO DE TRAJETÓRIA LINEAR PADRÃO (FALHA)
    # ===========================================================================

    print("\n" + "="*60)
    print("TESTE 1: TRAJETÓRIA CARTESIANA LINEAR PADRÃO")
    print("="*60)
    singu.resetar_robo(rtde_c, dash, q_inicial, "TESTE 1: Movimento Linear Estrito. Exigir uma linha reta forçara o colapso do punho.")
    
    status, dados, pose_real1 = singu.explorar_trajetoria_e_gravar(rtde_c, rtde_r, robo_ur10, q_inicial, pose_final)
    resultados_comparativos["Teste 1 (Linear)"] = pose_real1

    if status == "sucesso":
        print("\nO robô chegou ao destino sem acionar os alarmes de singularidade.")
        singu.popup_temporizado(dash, "RESULTADO INESPERADO: O robo chegou ao destino sem falhas.", 4.0)
    elif status == "singularidade":
        print("\n" + "="*60)
        print("(Falha de singularidade encontrada pelo Python - Interrompendo o movimento)")
        print("="*60)
        print(f"  Manipulabilidade (w):       {dados['manipulabilidade']:.4f}")
        print(f"  Juntas no ponto critico (q): " + ", ".join([f"{x:.4f}" for x in dados['q_inicial_critico']]))
        print(f"  Vetor cartesiano ofensor (v):" + ", ".join([f"{x:.4f}" for x in dados['vetor_cartesiano_ofensor']]))
        print("="*60 + "\n")
        singu.popup_temporizado(dash, "FALHA INTERCEPTADA: Singularidade detectada! O algoritmo puxou o freio de emergencia.", 5.0)

    # ===========================================================================
    # TESTE 2: CONTORNO VIA PSEUDOINVERSA AMORTECIDA (DLS)
    # ===========================================================================
    print("\n" + "="*60)
    print("TESTE 2: MÉTODO DA PSEUDOINVERSA AMORTECIDA (DLS)")
    print("="*60)
    singu.resetar_robo(rtde_c, dash, q_inicial, "TESTE 2: DLS. O algoritmo DLS irá sacrificar a orientação do punho para salvar os motores.")

    status, dados, pose_real2 = singu.Pseudoinvers_amortecida(rtde_c, rtde_r, robo_ur10, q_inicial, pose_final)
    resultados_comparativos["Teste 2 (DLS)"] = pose_real2
    
    if status == "sucesso":
        print("\nO robô concluiu a trajetória desviando da zona de colapso.")
        singu.popup_temporizado(dash, "SUCESSO DLS: Trajetoria concluida. O robo desviou do colapso em seguranca.", 4.0)
    elif status == "estagnado":
        print(f"\n[AVISO] O robô estagnou para evitar singularidade. Parou a {dados*100:.1f} cm do alvo.")
        singu.popup_temporizado(dash, f"ESTAGNACAO DLS: O robo parou a {dados*100:.1f} cm do alvo para nao quebrar.", 5.0)

    # ===========================================================================
    # TESTE 3: CONTROLE EM MALHA FECHADA COM ESPAÇO NULO
    # ===========================================================================
    print("\n" + "="*60)
    print("TESTE 3: CONTROLE EM MALHA FECHADA COM OTIMIZAÇÃO DE POSTURA")
    print("="*60)
    singu.resetar_robo(rtde_c, dash, q_inicial, "TESTE 3: Malha Fechada + Espaço Nulo. Desvio fluido com manutencao rigorosa de postura.")

    print("Iniciando controle dinâmico proporcional em malha fechada...")
    
    status, distancia_final, pose_real3 = singu.controlador_cartesiano_realtime(rtde_c, rtde_r, robo_ur10, q_inicial, pose_final, 5.0)
    resultados_comparativos["Teste 3 (Espaço Nulo)"] = pose_real3
    if status == "CONCLUIDO":
        print(f"\n[OK] Chegou ao alvo com sucesso. Distância final: {distancia_final*1000:.2f} mm.")
        singu.popup_temporizado(dash, "SUCESSO ABSOLUTO: Alvo atingido perfeitamente utilizando Controle Dinamico Avancado.", 5.0)
    elif status == "TIMEOUT":
        print(f"\n[AVISO] Timeout! O controle em malha fechada excedeu o limite. Distância final: {distancia_final*1000:.2f} mm.")
        singu.popup_temporizado(dash, "AVISO: Timeout na execucao da Malha Fechada.", 4.0)

    # ===========================================================================
    # GERAÇÃO DO RELATÓRIO COMPARATIVO FINAL
    # ===========================================================================
    print("\n" + "="*60)
    print("GERANDO GRÁFICO COMPARATIVO DE RESULTADOS")
    print("="*60)
    singu.plotar_comparacao_erros(resultados_comparativos, pose_final)

# Garante a saída limpa caso haja algum erro inesperado, clicando Ctrl+C, ou durante a execução normal.
except Exception as e:
    print(f"Falha na execução: {e}")

# Garante o encerramento correto das conexoes com o URSim mesmo que ocorra um erro inesperado.
finally:
    print("\nLimpando conexões de rede e encerrando com segurança...")
    # A verificação 'in locals()' previne erros em cascata: ela garante que o script só tentará 
    # fechar as conexões (disconnect) se essas variáveis chegaram de fato a ser criadas na memória.
    # Isso evita que o programa trave ao tentar encerrar algo que nunca chegou a conectar.
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
