#!/bin/bash

# Configuration
HOTSPOT_NAME="StarTrackHotspot"
HOTSPOT_IP="192.168.4.1"
INTERFACE="wlan0"
# Note: Adjust APP_DIR and ENV_NAME as needed for your setup
APP_DIR="/home/francish/Documents/startrack"
ENV_NAME=".venv" 

function log {
    echo "[$(date +'%H:%M:%S')] $1"
}

function stop_system {
    log "Stopping all StarTrack services..."
    sudo pkill -f "main.py"
    sudo fuser -k 443/tcp 2>/dev/null
}

function start_app {
    MODE=$1
    log "Starting Backend in $MODE mode..."
    cd $APP_DIR
    
    # Activate Virtual Env
    if [ -f "$APP_DIR/$ENV_NAME/bin/activate" ]; then
        source "$APP_DIR/$ENV_NAME/bin/activate"
    else
        log "ERROR: Virtual Environment not found at $APP_DIR/$ENV_NAME"
        exit 1
    fi
    
    # Run Python with unbuffered output
    sudo "$APP_DIR/$ENV_NAME/bin/python3" -u main.py --mode $MODE > system.log 2>&1 &
    
    log "System is live. Logs at $APP_DIR/system.log"
}

case "$1" in
    offline)
        stop_system
        log "Initializing OFFLINE (AP) Mode..."
        
        # 1. Force disconnect current connections to free the device
        sudo nmcli device disconnect $INTERFACE

        # 2. Create Hotspot if it doesn't exist
        if ! nmcli connection show "$HOTSPOT_NAME" >/dev/null 2>&1; then
            log "Creating Hotspot Profile..."
            sudo nmcli con add type wifi ifname $INTERFACE con-name "$HOTSPOT_NAME" autoconnect yes ssid "StarTrack_Unit1"
            sudo nmcli con modify "$HOTSPOT_NAME" 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared ipv4.addresses $HOTSPOT_IP/24
        fi

        # 3. CRITICAL: Enable Autoconnect for Field Mode
        # This ensures if the Pi reboots in the field, it comes back as a Hotspot
        sudo nmcli connection modify "$HOTSPOT_NAME" connection.autoconnect yes

        # 4. Bring up the Hotspot
        sudo nmcli connection up "$HOTSPOT_NAME"
        
        start_app "hardware"
        ;;
        
    online)
        stop_system
        log "Initializing ONLINE (Home WiFi) Mode..."

        # 1. CRITICAL: Disable Hotspot Autoconnect
        # This prevents NetworkManager from jumping back to the hotspot immediately
        sudo nmcli connection modify "$HOTSPOT_NAME" connection.autoconnect no
        
        # 2. Stop the hotspot
        sudo nmcli connection down "$HOTSPOT_NAME"
        
        # 3. Rescan for Home Networks
        log "Scanning for known home networks..."
        sudo nmcli device wifi rescan
        sleep 3 # Give it a moment to find the router

        # 4. Connect to best available known network
        # We look for a connection profile that is wireless but NOT the hotspot
        # and try to bring it up.
        
        # Get list of connection names that are Wifi and NOT the Hotspot
        HOME_NET=$(nmcli -t -f NAME,TYPE connection show | grep ":802-11-wireless" | grep -v "$HOTSPOT_NAME" | head -n 1 | cut -d: -f1)

        if [ -z "$HOME_NET" ]; then
            log "WARNING: No Home WiFi profile found!"
            log "Please connect manually once to save the profile:"
            log "sudo nmcli device wifi connect 'YOUR_SSID' password 'YOUR_PASS'"
            exit 1
        fi

        log "Found Home Profile: '$HOME_NET'. Connecting..."
        
        if sudo nmcli connection up "$HOME_NET"; then
             log "Successfully connected to $HOME_NET"
        else
             log "ERROR: Failed to connect to $HOME_NET"
             exit 1
        fi
        
        start_app "hardware"
        ;;
        
    sim)
        stop_system
        log "Initializing SIMULATION Mode (Looping Video)..."
        start_app "sim"
        ;;
        
    stop)
        stop_system
        ;;
        
    *)
        echo "Usage: $0 {offline|online|sim|stop}"
        exit 1
        ;;
esac