import socket

HOST = '127.0.0.1'
PORT = 60002

def start_tcp_client():
    print(f"Connecting to Ultraleap TCP server at {HOST}:{PORT}...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((HOST, PORT))
        print("Connected.")

        buffer = ''
        while True:
            data = client_socket.recv(4096).decode('utf-8')
            if not data:
                break

            buffer += data
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                if line:
                    numbers = list(map(float, line.strip().split(',')))
                    print(f"Received {len(numbers)} floats:")
                    print(numbers)   # <-- print all floats nicely
                    print()          # Add a blank line between frames for readability

if __name__ == '__main__':
    start_tcp_client()
