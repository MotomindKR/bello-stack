#!/usr/bin/env bash
set -euo pipefail

readonly BITRATE=1000000
readonly RESTART_MS=100
readonly TX_QUEUE_LENGTH=64
readonly INTERFACES=(can0 can1)

usage() {
  echo "usage: sudo $0 | $0 --check" >&2
}

if (( $# > 1 )); then
  usage
  exit 2
fi
if (( $# == 1 )) && [[ $1 != "--check" ]]; then
  usage
  exit 2
fi

CHECK_ONLY=false
if (( $# == 1 )); then
  CHECK_ONLY=true
fi
readonly CHECK_ONLY

if [[ $CHECK_ONLY == false ]] && (( EUID != 0 )); then
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

check_interface() {
  local interface=$1
  local details
  local first_line
  local flags

  if ! details=$(ip -details link show dev "$interface"); then
    echo "could not inspect SocketCAN interface $interface" >&2
    return 1
  fi
  first_line=${details%%$'\n'*}
  flags=${first_line#*<}
  if [[ $flags == "$first_line" || $flags != *">"* ]]; then
    echo "could not parse link flags for SocketCAN interface $interface" >&2
    return 1
  fi
  flags=${flags%%>*}
  if [[ ,$flags, != *,UP,* ]]; then
    echo "SocketCAN interface $interface is not UP" >&2
    return 1
  fi
  if [[ ! $first_line =~ [[:space:]]mtu[[:space:]]16([[:space:]]|$) ]]; then
    echo "SocketCAN interface $interface is not configured for classic CAN (expected MTU 16)" >&2
    return 1
  fi
  if [[ $details =~ (^|[[:space:]])fd[[:space:]]on([[:space:]]|$) ]]; then
    echo "SocketCAN interface $interface has CAN FD enabled" >&2
    return 1
  fi
  if [[ ! $details =~ (^|[[:space:]])bitrate[[:space:]]$BITRATE([[:space:]]|$) ]]; then
    echo "SocketCAN interface $interface is not configured for $BITRATE bit/s" >&2
    return 1
  fi
}

if [[ $CHECK_ONLY == true ]]; then
  for interface in "${INTERFACES[@]}"; do
    check_interface "$interface"
  done
  echo "can0 and can1 are UP and configured for classic CAN at $BITRATE bit/s"
  exit 0
fi

for interface in "${INTERFACES[@]}"; do
  ip link set dev "$interface" down
  ip link set dev "$interface" type can bitrate "$BITRATE" restart-ms "$RESTART_MS" fd off
  ip link set dev "$interface" txqueuelen "$TX_QUEUE_LENGTH"
  ip link set dev "$interface" up
done

ip -details link show dev can0
ip -details link show dev can1
