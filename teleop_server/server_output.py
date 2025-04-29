import socket

# Use the same HOST and PORT as defined in the server
HOST = '127.0.0.1'  # Localhost, assuming server.py is running locally
PORT = 60002         # The same port the server is listening on

def start_tcp_client():
    # Create a TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        # Connect the socket to the server's address and port
        client_socket.connect((HOST, PORT))
        print(f"Connected to server at {HOST}:{PORT}")

        while True:
            try:
                # Receive data from the server
                data = client_socket.recv(1024)  # Buffer size of 1024 bytes
                if not data:
                    break  # If no data, the connection has been closed
                print(f"Received data: {data.decode('utf-8')}")
            except KeyboardInterrupt:
                print("\nConnection closed by user.")
                break

if __name__ == '__main__':
    start_tcp_client()

