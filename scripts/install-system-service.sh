#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
service_template="$repo_root/deploy/systemd/system/sysguage.service.in"
rendered_service="$(mktemp)"
trap 'rm -f "$rendered_service"' EXIT

sudo install -m 0755 "$repo_root/target/release/sysguage" /usr/local/bin/sysguage
sed "s/@SYS_GUAGE_USER@/$USER/g" "$service_template" > "$rendered_service"
sudo install -m 0644 "$rendered_service" /etc/systemd/system/sysguage.service
sudo systemctl daemon-reload
sudo systemctl enable sysguage.service
sudo systemctl restart sysguage.service

echo "Installed and started system service: sysguage.service"
