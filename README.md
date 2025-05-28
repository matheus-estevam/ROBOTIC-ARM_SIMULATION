# Projeto Simulação Robótica com PyBullet, MQTT e Node-RED

## Descrição

Este projeto simula um braço robótico KUKA iiwa em PyBullet, enviando dados via MQTT para um dashboard criado no Node-RED. O robô monitora colisões e forças de contato com uma caixa vermelha, exibindo essas informações em tempo real e recebendo feedback do dashboard para adaptar seu comportamento.

---

## Tecnologias utilizadas

- Python 3.x  
- PyBullet (simulação física)  
- Paho-MQTT (comunicação MQTT)  
- Node-RED (dashboard visual)  
- Broker MQTT (ex: Mosquitto)  

---

## Instalação

```bash
pip install pybullet numpy paho-mqtt
sudo apt install mosquitto
npm install -g --unsafe-perm node-red
npm install node-red-dashboard
