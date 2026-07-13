#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-1000000}"

sudo ip link set down "${IFACE}" 2>/dev/null || true
sudo ip link set "${IFACE}" type can bitrate "${BITRATE}"
sudo ip link set "${IFACE}" txqueuelen 100
sudo ip link set up "${IFACE}"

ip -details link show "${IFACE}"
