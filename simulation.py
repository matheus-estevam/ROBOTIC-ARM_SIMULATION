import pybullet as p
import pybullet_data
import time
import numpy as np
import paho.mqtt.client as mqtt
import json


# ======= CONFIGURAÇÃO MQTT =======
broker = "localhost"  # Altere se for usar um broker externo
port = 1883
topic = "robot/status"

client = mqtt.Client()
client.connect(broker, port, 60)
client.loop_start()


# ======= CONFIGURAÇÃO PYBULLET =======
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

p.setGravity(0, 0, -9.8)


# ======= CRIAR CAIXA =======
box_half_size = 0.25
box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_half_size] * 3)
box_visual = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[box_half_size] * 3, rgbaColor=[1, 0, 0, 1]
)

box_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=box_collision,
    baseVisualShapeIndex=box_visual,
    basePosition=[0.6, 0, box_half_size],
)


# ======= JOINTS =======
end_effector_index = 6


# ======= FUNÇÕES =======


def validar_alcance(x, y, z):
    return 0.4 <= x <= 0.8 and -0.3 <= y <= 0.3 and 0.3 <= z <= 0.7


def verificar_colisao(robot_id, objeto_id, distancia_segura=0.05):
    pontos = p.getClosestPoints(robot_id, objeto_id, distancia_segura)
    return len(pontos) > 0


def calcular_magnitude(vetor):
    return np.linalg.norm(vetor)


def enviar_status(posicao, forca, torque, contato):
    payload = {
        "posicao": {"x": posicao[0], "y": posicao[1], "z": posicao[2]},
        "forca_N": round(forca, 2),
        "torque_Nm": round(torque, 2),
        "contato": contato,
    }
    client.publish(topic, json.dumps(payload))


# ======= LOOP PRINCIPAL =======
while True:

    # ======= PEGAR POSIÇÃO ATUAL DO BRAÇO (MOVIDO PELO MOUSE) =======
    link_state = p.getLinkState(robot_id, end_effector_index)
    target_position = link_state[0]
    x, y, z = target_position

    print("\n=== POSIÇÃO ATUAL ===")
    print(f"X: {x:.3f} | Y: {y:.3f} | Z: {z:.3f}")

    # ======= VALIDAÇÃO =======
    if not validar_alcance(x, y, z):
        print("❌ Posição FORA DO ALCANCE permitido!")
        contato_status = False
    elif verificar_colisao(robot_id, box_id):
        print("⚠️ COLISÃO DETECTADA! Movimento travado.")
        contato_status = True
    else:
        joint_positions = p.calculateInverseKinematics(
            robot_id, end_effector_index, target_position
        )
        for i in range(len(joint_positions)):
            p.setJointMotorControl2(
                robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=joint_positions[i],
                force=500,
            )
        contato_status = False

    # ======= SIMULAÇÃO =======
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

    # ======= DETECÇÃO DE CONTATO =======
    contact_points = p.getContactPoints(robot_id, box_id)

    if contact_points:
        for contact in contact_points:
            link_index = contact[3]
            force_normal = contact[9]
            contact_position = contact[6]
            contact_normal = contact[7]

            link_state = p.getLinkState(robot_id, link_index)
            link_world_position = link_state[0]

            r = np.array(contact_position) - np.array(link_world_position)
            force_vector = np.array(contact_normal) * force_normal
            torque = np.cross(r, force_vector)

            força_magnitude = calcular_magnitude(force_vector)
            torque_magnitude = calcular_magnitude(torque)

            print("\n=== 🟩 CONTATO DETECTADO ===")
            print(f"Força total (N): {força_magnitude:.2f}")
            print(f"Torque total (Nm): {torque_magnitude:.2f}")
            print(f"Na posição: {contact_position}")

            enviar_status(target_position, força_magnitude, torque_magnitude, True)
    else:
        print("Sem contato")
        enviar_status(target_position, 0, 0, False)

    # ======= REPORTAR STATUS =======
    link_state = p.getLinkState(robot_id, end_effector_index)
    posicao_atual = link_state[0]
    print("\n=== STATUS ATUAL ===")
    print(f"Posição do End-Effector: {posicao_atual}")
    print("✅ Sistema operacional.")
