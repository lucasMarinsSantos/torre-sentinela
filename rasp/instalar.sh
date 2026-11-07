#!/bin/bash
# Instala o servidor da Torre Sentinela no Raspberry Pi.
# Uso: cd ~/torre-sentinela/rasp && bash instalar.sh
set -e
PASTA="$(cd "$(dirname "$0")" && pwd)"
USUARIO="$(whoami)"

echo ">> Instalando dependências do sistema"
sudo apt-get update -qq
sudo apt-get install -y -qq python3-venv

echo ">> Criando ambiente Python"
python3 -m venv "$PASTA/venv"
"$PASTA/venv/bin/pip" install -q -r "$PASTA/requirements.txt"

echo ">> Liberando acesso à porta USB do Arduino para $USUARIO"
sudo usermod -aG dialout "$USUARIO"

[ -f "$PASTA/config.env" ] || cp "$PASTA/config.env.exemplo" "$PASTA/config.env"

echo ">> Configurando para iniciar junto com o Rasp"
sed -e "s|User=pi|User=$USUARIO|" -e "s|/home/pi/torre-sentinela/rasp|$PASTA|g" \
    "$PASTA/torre.service" | sudo tee /etc/systemd/system/torre.service > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now torre.service

IP=$(hostname -I | awk '{print $1}')
echo
echo "Pronto! Abra no notebook: http://$IP:5000"
echo "Ver os logs: journalctl -u torre -f"
