#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
service_dir="$HOME/.config/systemd/user"

mkdir -p "$HOME/.local/bin" "$service_dir"
install -m 0755 "$repo_root/target/release/sysguage" "$HOME/.local/bin/sysguage"
install -m 0644 "$repo_root/deploy/systemd/user/sysguage.service" "$service_dir/sysguage.service"

systemctl --user daemon-reload
systemctl --user enable sysguage.service
systemctl --user restart sysguage.service

echo "Installed and started user service: sysguage.service"
echo "For boot-before-login, run: sudo loginctl enable-linger $USER"
