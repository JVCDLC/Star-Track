import asyncio
import websockets
import json

# Configuration
HOST = "127.0.0.1"
PORT = 7000

# --- CATALOGUE CENTRALISÉ ---
# C'est ici que vous modifiez vos cibles !
CATALOG = {
    "tab-solar": [
        {"id": "SUN", "name": "SOLEIL", "icon": "☀️"},
        {"id": "MOON", "name": "LUNE", "icon": "🌙"},
        {"id": "JUPITER", "name": "JUPITER", "icon": "🪐"},
        {"id": "SATURN", "name": "SATURNE", "icon": "🪐"},
        {"id": "MARS", "name": "MARS", "icon": "🔴"},
        {"id": "VENUS", "name": "VENUS", "icon": "🟡"}
    ],
    "tab-stars": [
        {"id": "SIRIUS", "name": "SIRIUS", "icon": "⭐"},
        {"id": "POLARIS", "name": "POLARIS", "icon": "⭐"},
        {"id": "BETELGEUSE", "name": "BETELGEUSE", "icon": "⭐"},
        {"id": "RIGEL", "name": "RIGEL", "icon": "⭐"},
        {"id": "VEGA", "name": "VEGA", "icon": "⭐"}
    ],
    "tab-deep": [
        {"id": "M42", "name": "ORION NEBULA", "icon": "✨"},
        {"id": "M31", "name": "ANDROMEDA", "icon": "🌀"},
        {"id": "M45", "name": "PLEIADES", "icon": "✨"}
    ],
    "tab-sat": [
        {"id": "ISS", "name": "ISS", "icon": "🛰️"},
        {"id": "HST", "name": "HUBBLE", "icon": "🔭"}
    ]
}

print("Initialisation du Cerveau StarTrack...")

async def handler(websocket):
    print(f"[CONNEXION] Client Bridge connecté.")
    try:
        # Envoi automatique du catalogue des cibles au Remote lors de la connexion
        print("[SYSTEM] Envoi automatique du catalogue au Remote...")
        response = {
            "starTrackType": "config_targets",
            "starTrackData": CATALOG
        }
        await websocket.send(json.dumps(response))
        
        async for message in websocket:
            try:
                packet = json.loads(message)
                msg_type = packet.get('starTrackType')
                data = packet.get('starTrackData', {})
                
                # --- LOGIQUE DE TRAITEMENT ---
                
                if msg_type == 'request_targets':
                    print("[SYSTEM] Envoi du catalogue au Remote...")
                    response = {
                        "starTrackType": "config_targets",
                        "starTrackData": CATALOG
                    }
                    await websocket.send(json.dumps(response))

                elif msg_type == 'sensor_data':
                    # Traitement GPS (Existant)
                    lat = data['gps'].get('lat')
                    # print(f"[GPS] {lat}, ...") 
                    pass
                    
                    # Envoi du catalogue après réception des données de localisation
                    print("[SYSTEM] Envoi du catalogue au Remote après localisation...")
                    response = {
                        "starTrackType": "config_targets",
                        "starTrackData": CATALOG
                    }
                    await websocket.send(json.dumps(response))

                elif msg_type == 'goto_target':
                    target = data.get('target')
                    print(f"\n>>> [MISSION] GOTO : {target}")
                    
                elif msg_type == 'command':
                    print(f"[ACTION] {data.get('action')}")

            except json.JSONDecodeError:
                pass
            except Exception as e:
                print(f"[ERREUR] {e}")

    except websockets.exceptions.ConnectionClosed:
        print("[DECONNEXION] Le Bridge a coupé le lien.")

async def main():
    async with websockets.serve(handler, HOST, PORT):
        print(f"✅ Serveur Brain prêt sur ws://{HOST}:{PORT}")
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nArrêt.")