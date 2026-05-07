import asyncio
import websockets
import json

# Dictionary to hold rooms: room_id -> { "robot": websocket, "clients": set(websockets) }
rooms = {}

async def handler(websocket):
    # The path will be something like /?room=room1&role=robot or /?room=room1&role=client
    # websockets v11+ makes the path available via websocket.request.path
    path = websocket.request.path
    
    room_id = "default"
    role = "client"
    
    if "?" in path:
        query_string = path.split("?")[1]
        params = dict(q.split("=") for q in query_string.split("&") if "=" in q)
        room_id = params.get("room", "default")
        role = params.get("role", "client")

    if room_id not in rooms:
        rooms[room_id] = {"robot": None, "clients": set()}

    room = rooms[room_id]
    
    if role == "robot":
        if room["robot"] is not None:
            # Kick old robot if a new one connects
            try:
                await room["robot"].close()
            except:
                pass
        room["robot"] = websocket
        print(f"[Room {room_id}] Robot connected.")
    else:
        room["clients"].add(websocket)
        print(f"[Room {room_id}] Client connected. Total clients: {len(room['clients'])}")

    try:
        async for message in websocket:
            data = json.loads(message)
            
            # If a client sends a message, route it only to the robot
            if role == "client" and room["robot"]:
                try:
                    await room["robot"].send(json.dumps(data))
                except websockets.exceptions.ConnectionClosed:
                    pass
                    
            # If the robot sends a message, broadcast to all its clients
            elif role == "robot":
                disconnected = set()
                for client in room["clients"]:
                    try:
                        await client.send(json.dumps(data))
                    except websockets.exceptions.ConnectionClosed:
                        disconnected.add(client)
                room["clients"] -= disconnected

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        if role == "robot" and room["robot"] == websocket:
            room["robot"] = None
            print(f"[Room {room_id}] Robot disconnected.")
        elif role == "client":
            room["clients"].discard(websocket)
            print(f"[Room {room_id}] Client disconnected.")
            
        if room["robot"] is None and not room["clients"]:
            del rooms[room_id]

async def main():
    print("Signaling server running on ws://0.0.0.0:8765")
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())
