import leap
import time
import socket
import threading
from constants import *

class Runner(leap.Listener):
    def __init__(self):
        self.last_update = None
        self.network_lock = threading.Lock()
        self.connections = []

    def on_connection_event(self, event):
        print("Device connected")

    def on_device_event(self, event):
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()
        print(f"Found device {info.serial}")

    def on_tracking_event(self, event):
        all_positions = []
        if event.hands:
            hand = event.hands[0]
            for digit in [hand.thumb, hand.index, hand.middle, hand.ring, hand.pinky]:
                for bone in [digit.metacarpal, digit.proximal, digit.intermediate, digit.distal]:
                    vec_x = bone.next_joint.x - bone.prev_joint.x
                    vec_y = bone.next_joint.y - bone.prev_joint.y
                    vec_z = bone.next_joint.z - bone.prev_joint.z
                    all_positions.append(f"{vec_x},{vec_y},{vec_z}")

            palm = hand.palm.position
            all_positions.append(f"{palm.x},{palm.y},{palm.z}")

        if all_positions:
            self.last_update = ",".join(all_positions) + "\n"

    def run_server_on_port(self, server_socket):
        while True:
            print(f"[PORT {server_socket.getsockname()[1]}] Waiting for connection...")
            conn, addr = server_socket.accept()
            conn.setblocking(False)
            print(f"[PORT {server_socket.getsockname()[1]}] Connected by {addr}")

            while True:
                try:
                    _ = conn.recv(1024)
                except BlockingIOError:
                    pass
                except ConnectionResetError:
                    print(f"[PORT {server_socket.getsockname()[1]}] Client disconnected")
                    break

                with self.network_lock:
                    if self.last_update is not None:
                        try:
                            conn.sendall(self.last_update.encode('utf-8'))
                        except (BlockingIOError, BrokenPipeError):
                            print(f"[PORT {server_socket.getsockname()[1]}] Send failed")
                            break
                        self.last_update = None

                time.sleep(0.01)

            conn.close()

def main():
    runner = Runner()
    connection = leap.Connection()
    connection.add_listener(runner)
    connection.set_tracking_mode(leap.TrackingMode.Desktop)

    with connection.open(), \
         socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server1, \
         socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server2:

        server1.bind((HOST, PORT1))
        server2.bind((HOST, PORT2))
        server1.listen()
        server2.listen()

        # Start two threads for two TCP clients
        t1 = threading.Thread(target=runner.run_server_on_port, args=(server1,))
        t2 = threading.Thread(target=runner.run_server_on_port, args=(server2,))

        t1.start()
        t2.start()

        t1.join()
        t2.join()

if __name__ == "__main__":
    main()
