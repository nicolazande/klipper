#!/bin/bash

# Definisci i valori di default
DEFAULT_FILE="$HOME/klipper/out/klipper.bin"
DEFAULT_IP="100.100.69.119"

# Usa il primo argomento passato allo script come file, oppure il default
FILE=${1:-$DEFAULT_FILE}

# Usa il secondo argomento passato allo script come IP, oppure il default
IP=${2:-$DEFAULT_IP}

# Esegui il comando tailscale con i parametri definiti
sudo tailscale file cp "$FILE" "$IP:"
