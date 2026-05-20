#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rule_src="$repo_root/deploy/udev/99-sysguage.rules"
rule_dest="/etc/udev/rules.d/99-sysguage.rules"

sudo install -m 0644 "$rule_src" "$rule_dest"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

echo "Installed $rule_dest"
echo "Unplug and reconnect the Arduino if permissions do not update immediately."
