#!/bin/bash

set -e

echo "Suppression des versions Docker précédentes..."
sudo apt-get remove -y docker docker-engine docker.io containerd runc || true

echo "Mise à jour des paquets..."
sudo apt-get update

echo "Installation des dépendances de Docker..."
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

echo "Ajout de la clé GPG officielle de Docker..."
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
    sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

echo "Ajout du dépôt Docker officiel..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

echo "Installation de Docker..."
sudo apt-get update
sudo apt-get install -y \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin

echo "Test de Docker avec 'hello-world'..."
sudo docker run hello-world

echo "Ajout de l'utilisateur courant au groupe 'docker'..."
sudo usermod -aG docker $USER
newgrp docker

echo "Docker installé avec succès !"

echo "Installation des pilotes NVIDIA automatiquement..."
sudo ubuntu-drivers autoinstall

echo "Ajout du dépôt NVIDIA Docker (Ubuntu 22.04 workaround pour 24.04)..."
distribution=ubuntu22.04
sudo mkdir -p /etc/apt/keyrings
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
    sudo gpg --dearmor -o /etc/apt/keyrings/nvidia-docker.gpg

curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sed 's#deb https://#deb [signed-by=/etc/apt/keyrings/nvidia-docker.gpg] https://#' | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list

echo "Installation de NVIDIA Docker..."
sudo apt-get update
sudo apt-get install -y nvidia-docker2

echo "Redémarrage du service Docker..."
sudo systemctl restart docker

echo "Test du GPU dans un conteneur Docker..."
docker run --rm --gpus all nvidia/cuda:12.3.2-base-ubuntu22.04 nvidia-smi

echo "Installation terminée avec succès ! Redémarrage recommandé."
