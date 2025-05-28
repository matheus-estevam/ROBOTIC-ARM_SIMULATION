# =========================
# SIMULAÇÃO ROBÓTICA COM PYBULLET + MQTT
# =========================

# =========================
# IMPORTAÇÃO DAS BIBLIOTECAS
# =========================
import pybullet as p               # Biblioteca de simulação física
import pybullet_data               # Conjunto de modelos padrão do PyBullet
import time                        # Controle de tempo
import numpy as np                 # Operações matemáticas (vetores, magnitudes, etc.)
import paho.mqtt.client as mqtt    # Comunicação MQTT
import json                        # Formatação de dados em JSON para enviar ao MQTT


# =========================
# CONFIGURAÇÃO DO MQTT
# =========================
broker = "localhost"   # Endereço do broker MQTT (local). Pode ser IP externo se necessário.
port = 1883            # Porta padrão MQTT
topic = "robot/status" # Tópico para onde os dados do robô serão publicados

# Criar cliente MQTT e conectar
client = mqtt.Client()
client.connect(broker, port, 60)
client.loop_start()   # Mantém o cliente rodando em segundo plano


# =========================
# CONFIGURAÇÃO DO PYBULLET
# =========================
p.connect(p.GUI)                           # Abre a interface gráfica do PyBullet
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Permite carregar arquivos URDF padrão

# Carrega o plano como base
plane_id = p.loadURDF("plane.urdf")

# Carrega o robô KUKA iiwa com base fixa
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

# Define a gravidade na simulação
p.setGravity(0, 0, -9.8)


# =========================
# CRIAÇÃO DE UM OBJETO (CAIXA)
# =========================
box_half_size = 0.25  # Tamanho da caixa (metade do lado)

# Cria a caixa para colisão (parte física)
box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_half_size] * 3)

# Cria a caixa para visualização (cor vermelha)
box_visual = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[box_half_size] * 3, rgbaColor=[1, 0, 0, 1]
)

# Junta colisão + visual + posicionamento no espaço
box_id = p.createMultiBody(
    baseMass=0,                         # Massa zero → objeto fixo (não se move)
    baseCollisionShapeIndex=box_collision,
    baseVisualShapeIndex=box_visual,
    basePosition=[0.6, 0, box_half_size] # Posição inicial da caixa
)


# =========================
# CONFIGURAÇÃO DOS JOINTS
# =========================
end_effector_index = 6  # Índice do último elo (ponteira do robô)


# =========================
# DEFINIÇÃO DE FUNÇÕES AUXILIARES
# =========================

# Verifica se a posição desejada está dentro da área operacional do robô
def validar_alcance(x, y, z):
    return 0.4 <= x <= 0.8 and -0.3 <= y <= 0.3 and 0.3 <= z <= 0.7

# Verifica se há colisão entre o robô e a caixa
def verificar_colisao(robot_id, objeto_id, distancia_segura=0.05):
    pontos = p.getClosestPoints(robot_id, objeto_id, distancia_segura)
    return len(pontos) > 0

# Calcula a magnitude (tamanho) de um vetor (ex.: força ou torque)
def calcular_magnitude(vetor):
    return np.linalg.norm(vetor)

# Envia os dados do robô (posição, força, torque, status de contato) via MQTT em formato JSON
def enviar_status(posicao, forca, torque, contato):
    payload = {
        "posicao": {"x": posicao[0], "y": posicao[1], "z": posicao[2]},
        "forca_N": round(forca, 2),
        "torque_Nm": round(torque, 2),
        "contato": contato,
    }
    client.publish(topic, json.dumps(payload))


# =========================
# LOOP PRINCIPAL DE SIMULAÇÃO
# =========================
while True:

    # === LÊ A POSIÇÃO ATUAL DA PONTEIRA (end-effector) DO ROBÔ ===
    # Essa posição pode ser modificada manualmente na interface com o mouse
    link_state = p.getLinkState(robot_id, end_effector_index)
    target_position = link_state[0]   # Coordenadas (x, y, z) da ponteira
    x, y, z = target_position

    print("\n=== POSIÇÃO ATUAL ===")
    print(f"X: {x:.3f} | Y: {y:.3f} | Z: {z:.3f}")

    # === VERIFICA SE A POSIÇÃO É VÁLIDA ===
    if not validar_alcance(x, y, z):
        print("❌ Posição FORA DO ALCANCE permitido!")
        contato_status = False

    # === VERIFICA COLISÃO COM A CAIXA ===
    elif verificar_colisao(robot_id, box_id):
        print("⚠️ COLISÃO DETECTADA! Movimento travado.")
        contato_status = True

    else:
        # === CALCULA A CINEMÁTICA INVERSA ===
        # Cinemática inversa → dado um ponto no espaço (x, y, z),
        # retorna os ângulos das juntas necessários para o robô alcançar esse ponto.
        # A cinemática inversa é o processo de converter coordenadas cartesianas (posição no espaço) → para ângulos de cada junta do robô.
        joint_positions = p.calculateInverseKinematics(
            robot_id, end_effector_index, target_position
        )

        # === APLICA OS ÂNGULOS CALCULADOS NAS JUNTAS ===
        for i in range(len(joint_positions)):
            p.setJointMotorControl2(
                robot_id,
                i,
                p.POSITION_CONTROL,           # Controle de posição (não de torque direto)
                targetPosition=joint_positions[i], # Ângulo alvo da junta
                force=500                     # Força máxima aplicada no motor
            )

        contato_status = False  # Não há contato neste momento

    # === AVANÇA A SIMULAÇÃO ===
    p.stepSimulation()
    time.sleep(1.0 / 240.0)  # Delay para simular tempo real

    # === VERIFICA SE HÁ CONTATO ENTRE ROBÔ E CAIXA ===
    contact_points = p.getContactPoints(robot_id, box_id)

    if contact_points:
        # Se houver pelo menos um ponto de contato
        for contact in contact_points:
            link_index = contact[3]         # Qual link (elo) do robô está em contato
            force_normal = contact[9]       # Força normal do contato (Newton)
            contact_position = contact[6]   # Posição do ponto de contato no mundo
            contact_normal = contact[7]     # Vetor normal no ponto de contato

            # === Calcula torque devido ao contato ===
            link_state = p.getLinkState(robot_id, link_index)
            link_world_position = link_state[0]

            # Vetor r → distância entre centro do link e ponto de contato
            r = np.array(contact_position) - np.array(link_world_position)

            # Vetor força → força normal aplicada na direção da normal do contato
            force_vector = np.array(contact_normal) * force_normal

            # Cálculo do torque → torque = r × F (produto vetorial)
            torque = np.cross(r, force_vector)

            # Calcula magnitude da força e do torque
            força_magnitude = calcular_magnitude(force_vector)
            torque_magnitude = calcular_magnitude(torque)

            print("\n=== 🟩 CONTATO DETECTADO ===")
            print(f"Força total (N): {força_magnitude:.2f}")
            print(f"Torque total (Nm): {torque_magnitude:.2f}")
            print(f"Na posição: {contact_position}")

            # === Envia dados via MQTT ===
            enviar_status(target_position, força_magnitude, torque_magnitude, True)
    else:
        # Caso não haja contato
        print("Sem contato")
        enviar_status(target_position, 0, 0, False)

    # === RELATÓRIO DE STATUS FINAL ===
    link_state = p.getLinkState(robot_id, end_effector_index)
    posicao_atual = link_state[0]
    print("\n=== STATUS ATUAL ===")
    print(f"Posição do End-Effector: {posicao_atual}")
    print("✅ Sistema operacional.")



# === O que o código faz ===
# Controla o robô KUKA iiwa na simulação 3D.
# Verifica colisões com uma caixa vermelha.
# Calcula forças e torques gerados no contato.
# Publica esses dados em tempo real via MQTT para um dashboard externo (ex.: Node-RED).
# Permite mover o robô interativamente pela interface gráfica do PyBullet.
