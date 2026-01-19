#!/usr/bin/env bash
set -euo pipefail

PREFIX="/opt/ugv_rover_pt_scan"

sudo mkdir -p "$PREFIX"
sudo rsync -a --delete ./ "$PREFIX/"

sudo install -m 0644 "$PREFIX/systemd/ugv-rover-pt.service" /etc/systemd/system/ugv-rover-pt.service

sudo systemctl daemon-reload
sudo systemctl enable ugv-rover-pt.service
sudo systemctl restart ugv-rover-pt.service

echo "Installed and started ugv-rover-pt.service"
