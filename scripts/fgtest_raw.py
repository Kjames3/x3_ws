import socket, base64, os

host = 'localhost'
port = 8765

def ws_handshake(subprotocol=None):
    key = base64.b64encode(os.urandom(16)).decode()
    headers = (
        f"GET / HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        f"Upgrade: websocket\r\n"
        f"Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        f"Sec-WebSocket-Version: 13\r\n"
    )
    if subprotocol:
        headers += f"Sec-WebSocket-Protocol: {subprotocol}\r\n"
    headers += "\r\n"

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((host, port))
    s.send(headers.encode())
    response = s.recv(4096).decode(errors='replace')
    s.close()
    return response

print("=== Test 1: foxglove.websocket.v1 ===")
print(ws_handshake('foxglove.websocket.v1'))

print("=== Test 2: no subprotocol ===")
print(ws_handshake(None))

print("=== Test 3: foxglove.websocket.v2 ===")
print(ws_handshake('foxglove.websocket.v2'))
