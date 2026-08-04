#!/usr/bin/env bash
set -euo pipefail

readonly BITRATE=1000000
readonly RESTART_MS=100
readonly INTERFACES=(can0 can1)

if (( $# != 0 )); then
  echo "usage: sudo $0" >&2
  exit 2
fi

if (( EUID != 0 )); then
  echo "run this script as root: sudo $0" >&2
  exit 1
fi

if ! command -v ip >/dev/null 2>&1; then
  echo "iproute2 is required to configure SocketCAN interfaces" >&2
  exit 1
fi

for interface in "${INTERFACES[@]}"; do
  if ! ip link show dev "$interface" >/dev/null 2>&1; then
    echo "SocketCAN interface $interface does not exist" >&2
    exit 1
  fi
done

for interface in "${INTERFACES[@]}"; do
  ip link set dev "$interface" down
  ip link set dev "$interface" type can bitrate "$BITRATE" restart-ms "$RESTART_MS"
  ip link set dev "$interface" up
done

ip -details link show dev can0
ip -details link show dev can1
