import socket, base64, os, subprocess, re

host = 'localhost'
port = 8765

def ws_handshake(subprotocol=None, extra_headers=None):
    key = base64.b64encode(os.urandom(16)).decode()
    lines = [
        f"GET / HTTP/1.1",
        f"Host: {host}:{port}",
        f"Upgrade: websocket",
        f"Connection: Upgrade",
        f"Sec-WebSocket-Key: {key}",
        f"Sec-WebSocket-Version: 13",
    ]
    if subprotocol:
        lines.append(f"Sec-WebSocket-Protocol: {subprotocol}")
    if extra_headers:
        lines.extend(extra_headers)
    request = "\r\n".join(lines) + "\r\n\r\n"

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((host, port))
    s.send(request.encode())
    response = s.recv(4096).decode(errors='replace')
    s.close()
    # Return just status line + body
    parts = response.split("\r\n\r\n", 1)
    status = response.split("\r\n")[0]
    body = parts[1].strip() if len(parts) > 1 else ""
    return f"{status} | {body}"

# --- Find subprotocol string in installed binary ---
print("=== Searching installed binary for protocol strings ===")
try:
    result = subprocess.run(
        ["find", "/opt/ros/humble", "-name", "foxglove_bridge", "-type", "f"],
        capture_output=True, text=True
    )
    binaries = result.stdout.strip().split("\n")
    for b in binaries:
        if b:
            r = subprocess.run(["strings", b], capture_output=True, text=True)
            for line in r.stdout.splitlines():
                if "foxglove" in line.lower() or "websocket" in line.lower():
                    print(f"  {line}")
except Exception as e:
    print(f"  (binary search failed: {e})")

# --- Also check shared libs ---
try:
    result = subprocess.run(
        ["find", "/opt/ros/humble", "-name", "libfoxglove*"],
        capture_output=True, text=True
    )
    for lib in result.stdout.strip().split("\n"):
        if lib:
            r = subprocess.run(["strings", lib], capture_output=True, text=True)
            for line in r.stdout.splitlines():
                if ("foxglove" in line.lower() and "websocket" in line.lower()) or "subprotocol" in line.lower():
                    print(f"  [{os.path.basename(lib)}] {line}")
except Exception as e:
    print(f"  (lib search failed: {e})")

print()

# --- Handshake tests ---
candidates = [
    "foxglove.websocket.v1",
    "foxglove.websocket.v2",
    "foxglove-websocket-v1",
    "foxglove-websocket-v2",
    "ros.foxglove.v1",
    None,
]

for proto in candidates:
    label = proto if proto else "(none)"
    print(f"=== {label} ===")
    print(ws_handshake(proto))
    print()
