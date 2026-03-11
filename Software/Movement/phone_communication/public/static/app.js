// Dynamic Host Resolution (Works for AP and Home WiFi)
const HOST = window.location.hostname;
const PORT = window.location.port || 443;
const WS_URL = `wss://${HOST}:${PORT}/ws`;
const STREAM_URL = `https://${HOST}:${PORT}/stream`;

const ui = {
    stream: document.getElementById('camera-stream'),
    status: document.getElementById('conn-status'),
    joystickBtn: document.getElementById('btn-joystick'),
    joystickZone: document.getElementById('joystick-zone'),
    log: document.getElementById('log-output')
};

let ws;
let joystickManager = null;

// --- 1. VIDEO HANDLING ---
function startStream() {
    // Add timestamp to prevent caching
    ui.stream.src = `${STREAM_URL}?t=${Date.now()}`;
    ui.stream.onerror = () => {
        console.log("Stream lost. Retrying...");
        setTimeout(startStream, 2000);
    };
}

// --- 2. TELEMETRY HANDLING ---
function connectWs() {
    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        ui.status.className = 'online';
        log("System Connected");
        
        // If we are on mobile (with GPS), send coordinates
        if (navigator.geolocation) {
            navigator.geolocation.watchPosition(pos => {
                send('SENSOR_GPS', {
                    lat: pos.coords.latitude,
                    lon: pos.coords.longitude,
                    alt: pos.coords.altitude
                });
            }, null, { enableHighAccuracy: true });
        }
    };

    ws.onmessage = (e) => {
        const msg = JSON.parse(e.data);
        handleUpdate(msg);
    };

    ws.onclose = () => {
        ui.status.className = 'offline';
        log("Connection Lost. Reconnecting...");
        setTimeout(connectWs, 1000);
    };
}

function send(type, data) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type, data }));
    }
}

function handleUpdate(packet) {
    // Sync UI with incoming data
    if (packet.type === 'SYNC' || packet.type === 'SENSOR_GPS') {
        const data = packet.data.gps || packet.data;
        if(data.lat) {
            document.getElementById('gps-lat').innerText = `LAT: ${data.lat.toFixed(4)}`;
            document.getElementById('gps-lon').innerText = `LON: ${data.lon.toFixed(4)}`;
        }
    }
    // Bridge logic: If we receive Joystick data from another client, visualize it here?
    // (Optional implementation)
}

function log(msg) {
    console.log(msg);
    if (ui.log) {
        const line = document.createElement('div');
        line.innerText = `[${new Date().toLocaleTimeString()}] ${msg}`;
        ui.log.prepend(line);
    }
}

// --- 3. INPUT HANDLING ---
ui.joystickBtn.addEventListener('click', () => {
    ui.joystickZone.classList.toggle('active');
    
    if (ui.joystickZone.classList.contains('active')) {
        if (!joystickManager) {
            joystickManager = nipplejs.create({
                zone: ui.joystickZone,
                mode: 'static',
                position: { left: '50%', top: '50%' },
                color: '#ff3333'
            });

            joystickManager.on('move', (evt, data) => {
                if (data.vector) {
                    send('JOYSTICK', { x: data.vector.x, y: data.vector.y });
                }
            });

            joystickManager.on('end', () => {
                send('JOYSTICK', { x: 0, y: 0 });
            });
        }
    }
});

document.getElementById('btn-capture').addEventListener('click', () => {
    send('COMMAND', { action: 'PHOTO' });
});

// Init
window.onload = () => {
    startStream();
    connectWs();
};