#!/usr/bin/env bash
set -euo pipefail

# -------------------------
# Configuration
# -------------------------
HOTSPOT_NAME="${HOTSPOT_NAME:-StarTrackHotspot}"
HOTSPOT_IP="${HOTSPOT_IP:-192.168.4.1}"
INTERFACE="${INTERFACE:-wlan0}"
HOTSPOT_SSID="${HOTSPOT_SSID:-StarTrack_Unit1}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
APP_DIR="${APP_DIR:-$REPO_ROOT/Software/Raspberry Pi/WebUI}"
ENTRYPOINT="${ENTRYPOINT:-$APP_DIR/main.py}"
LOG_FILE="${LOG_FILE:-$APP_DIR/system.log}"
PID_FILE="${PID_FILE:-$APP_DIR/startrack.pid}"

VENV_CANDIDATES=(
    "${VENV_PATH:-$REPO_ROOT/.venv}"
    "$APP_DIR/.venv"
)

MODE_DEFAULT="${MODE_DEFAULT:-hardware}" # hardware | sim
SERVER_PORT="${SERVER_PORT:-443}"
SIM_ARDUINO_DEFAULT="${SIM_ARDUINO_DEFAULT:-auto}" # auto | on | off
ARDUINO_PORT_DEFAULT="${ARDUINO_PORT_DEFAULT:-auto}" # auto | /dev/ttyUSB0 | COM6 ...
ARDUINO_BAUD_DEFAULT="${ARDUINO_BAUD_DEFAULT:-115200}"

log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $*"
}

err() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] ERROR: $*" >&2
}

usage() {
    cat <<EOF
Usage:
  $0 offline [hardware|sim]
  $0 online  [hardware|sim]
  $0 sim
  $0 stop
  $0 status
  $0 logs

Examples:
  $0 offline hardware
  $0 offline sim
  $0 online
  $0 sim
EOF
}

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || {
        err "Missing required command: $1"
        exit 1
    }
}

resolve_python_bin() {
    local candidate
    for candidate in "${VENV_CANDIDATES[@]}"; do
        if [[ -x "$candidate/bin/python3" ]]; then
            echo "$candidate/bin/python3"
            return 0
        fi
    done
    err "Python virtualenv not found. Checked:"
    for candidate in "${VENV_CANDIDATES[@]}"; do
        err "  - $candidate"
    done
    exit 1
}

is_running() {
    if [[ -f "$PID_FILE" ]]; then
        local pid
        pid="$(cat "$PID_FILE" 2>/dev/null || true)"
        if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
            return 0
        fi
    fi
    return 1
}

stop_system() {
    log "Stopping StarTrack services..."
    if is_running; then
        local pid
        pid="$(cat "$PID_FILE")"
        kill "$pid" 2>/dev/null || true
        sleep 1
        kill -9 "$pid" 2>/dev/null || true
        rm -f "$PID_FILE"
        log "Stopped PID $pid"
    else
        rm -f "$PID_FILE" 2>/dev/null || true
        pkill -f "WebUI/main.py" 2>/dev/null || true
        log "No PID file process running."
    fi

    # Free configured port if something else still owns it.
    sudo fuser -k "${SERVER_PORT}/tcp" 2>/dev/null || true
}

start_app() {
    local mode="${1:-$MODE_DEFAULT}"
    if [[ "$mode" != "hardware" && "$mode" != "sim" ]]; then
        err "Invalid mode '$mode'. Use hardware or sim."
        exit 1
    fi

    if [[ ! -f "$ENTRYPOINT" ]]; then
        err "Entrypoint not found: $ENTRYPOINT"
        exit 1
    fi

    if is_running; then
        local pid
        pid="$(cat "$PID_FILE")"
        err "StarTrack already running (PID $pid). Stop it first."
        exit 1
    fi

    mkdir -p "$APP_DIR"
    local py_bin
    py_bin="$(resolve_python_bin)"
    local sim_arduino
    case "$SIM_ARDUINO_DEFAULT" in
        auto)
            if [[ "$mode" == "sim" ]]; then
                sim_arduino="1"
            else
                sim_arduino="0"
            fi
            ;;
        on|true|1|yes)
            sim_arduino="1"
            ;;
        off|false|0|no)
            sim_arduino="0"
            ;;
        *)
            err "Invalid SIM_ARDUINO_DEFAULT='$SIM_ARDUINO_DEFAULT' (use auto|on|off)."
            exit 1
            ;;
    esac

    log "Starting backend in '$mode' mode..."
    log "App dir: $APP_DIR"
    log "Log file: $LOG_FILE"
    log "Port: $SERVER_PORT"
    log "Sim Arduino: $sim_arduino"
    log "Arduino Port: $ARDUINO_PORT_DEFAULT"
    log "Arduino Baud: $ARDUINO_BAUD_DEFAULT"

    (
        cd "$APP_DIR"
        STARTRACK_SIM_ARDUINO="$sim_arduino" \
        STARTRACK_ARDUINO_PORT="$ARDUINO_PORT_DEFAULT" \
        STARTRACK_ARDUINO_BAUD="$ARDUINO_BAUD_DEFAULT" \
        nohup "$py_bin" -u "$ENTRYPOINT" --mode "$mode" --port "$SERVER_PORT" >>"$LOG_FILE" 2>&1 &
        echo $! >"$PID_FILE"
    )

    sleep 1
    if is_running; then
        log "System is live (PID $(cat "$PID_FILE"))."
    else
        err "Startup failed. Check logs: $LOG_FILE"
        exit 1
    fi
}

setup_offline_network() {
    require_cmd nmcli
    log "Initializing OFFLINE (AP) mode on $INTERFACE..."

    sudo nmcli device disconnect "$INTERFACE" || true

    if ! nmcli connection show "$HOTSPOT_NAME" >/dev/null 2>&1; then
        log "Creating hotspot profile '$HOTSPOT_NAME'..."
        sudo nmcli con add \
            type wifi \
            ifname "$INTERFACE" \
            con-name "$HOTSPOT_NAME" \
            autoconnect yes \
            ssid "$HOTSPOT_SSID"
        sudo nmcli con modify \
            "$HOTSPOT_NAME" \
            802-11-wireless.mode ap \
            802-11-wireless.band bg \
            ipv4.method shared \
            ipv4.addresses "$HOTSPOT_IP/24"
    fi

    sudo nmcli connection modify "$HOTSPOT_NAME" connection.autoconnect yes
    sudo nmcli connection up "$HOTSPOT_NAME"
}

setup_online_network() {
    require_cmd nmcli
    log "Initializing ONLINE (home Wi-Fi) mode..."

    sudo nmcli connection modify "$HOTSPOT_NAME" connection.autoconnect no || true
    sudo nmcli connection down "$HOTSPOT_NAME" || true

    log "Scanning Wi-Fi..."
    sudo nmcli device wifi rescan || true
    sleep 3

    local home_net
    home_net="$(nmcli -t -f NAME,TYPE connection show \
        | grep ":802-11-wireless" \
        | grep -v "$HOTSPOT_NAME" \
        | head -n 1 \
        | cut -d: -f1)"

    if [[ -z "${home_net:-}" ]]; then
        err "No saved home Wi-Fi profile found."
        err "Connect once manually to save profile:"
        err "  sudo nmcli device wifi connect '<SSID>' password '<PASSWORD>'"
        exit 1
    fi

    log "Connecting to '$home_net'..."
    sudo nmcli connection up "$home_net"
}

show_status() {
    if is_running; then
        log "RUNNING (PID $(cat "$PID_FILE"))"
    else
        log "STOPPED"
    fi
    log "Entrypoint: $ENTRYPOINT"
    log "Log file: $LOG_FILE"
}

case "${1:-}" in
    offline)
        stop_system
        setup_offline_network
        start_app "${2:-$MODE_DEFAULT}"
        ;;
    online)
        stop_system
        setup_online_network
        start_app "${2:-$MODE_DEFAULT}"
        ;;
    sim)
        stop_system
        log "Starting simulation shortcut..."
        start_app "sim"
        ;;
    stop)
        stop_system
        ;;
    status)
        show_status
        ;;
    logs)
        touch "$LOG_FILE"
        tail -n 200 -f "$LOG_FILE"
        ;;
    *)
        usage
        exit 1
        ;;
esac
