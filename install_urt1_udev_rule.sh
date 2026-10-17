#!/usr/bin/env bash
set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-urt1.rules"
RULE='SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666", SYMLINK+="urt1"'

echo "$RULE" | sudo tee "$RULE_FILE" >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "설치 완료: $RULE_FILE"
echo "URT-1을 뺐다가 다시 꽂으면 /dev/urt1 로도 접근할 수 있습니다."
