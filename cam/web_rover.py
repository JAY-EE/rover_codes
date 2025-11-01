import asyncio
import websockets
import json
import logging

# --- Configuration ---
HOST = '0.0.0.0'  # Listen on all available network interfaces
PORT = 8765       # Port for the signaling server

# --- Server State ---
clients = {}
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def handle_client(websocket, path):
    """Handles connections from GStreamer pipelines and browsers."""
    client_type = None
    camera_id = None
    
    try:
        message = await websocket.recv()
        data = json.loads(message)

        if data.get('type') == 'register':
            client_type = data['client_type']
            camera_id = data['camera_id']

            if camera_id not in clients:
                clients[camera_id] = {'gstreamer': None, 'browsers': set()}

            if client_type == 'gstreamer':
                logger.info(f"GStreamer for '{camera_id}' registered.")
                clients[camera_id]['gstreamer'] = websocket
            elif client_type == 'browser':
                logger.info(f"Browser for '{camera_id}' registered.")
                clients[camera_id]['browsers'].add(websocket)
            
            async for message in websocket:
                await route_message(message, sender_websocket=websocket, camera_id=camera_id, sender_type=client_type)
        else:
            logger.warning("First message was not a 'register' message. Closing.")

    except websockets.exceptions.ConnectionClosed:
        logger.info(f"Connection closed for {client_type} '{camera_id}'.")
    finally:
        if camera_id and client_type:
            if client_type == 'gstreamer' and clients.get(camera_id):
                clients[camera_id]['gstreamer'] = None
            elif client_type == 'browser' and clients.get(camera_id):
                clients[camera_id]['browsers'].discard(websocket)

async def route_message(message, sender_websocket, camera_id, sender_type):
    """Routes WebRTC signaling messages between GStreamer and browsers."""
    if camera_id not in clients:
        return

    gstreamer_ws = clients[camera_id]['gstreamer']
    browser_set = clients[camera_id]['browsers']

    if sender_type == 'gstreamer':
        # GStreamer -> Forward to all browsers for this camera
        for browser_ws in browser_set:
            if browser_ws != sender_websocket:
                await browser_ws.send(message)
    elif sender_type == 'browser':
        # Browser -> Forward to GStreamer for this camera
        if gstreamer_ws and gstreamer_ws != sender_websocket:
            await gstreamer_ws.send(message)

async def main():
    logger.info(f"Starting signaling server on ws://{HOST}:{PORT}")
    async with websockets.serve(handle_client, HOST, PORT):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())

