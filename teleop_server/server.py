import leap
import time
import socket
import threading

from constants import *

class Runner(leap.Listener):

    def __init__(self):
        self.conn = None
        self.network_lock = threading.Lock()
        self.last_update = None

    # Leap tracking callbacks

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
            for digit in hand.digits:
                for bone in digit.bones:
                    pos = bone.next_joint
                    all_positions.append(f"{pos.x},{pos.y},{pos.z}")

            # Append the palm position at the end (landmark 20)
            # Append the palm position (landmark 20)
            palm = hand.palm.position
            all_positions.append(f"{palm.x},{palm.y},{palm.z}")

            # Append the wrist position (landmark 21)
            wrist = hand.arm.next_joint  # or hand.wrist.position if available
            all_positions.append(f"{wrist.x},{wrist.y},{wrist.z}")

        if all_positions:
            self.last_update = ",".join(all_positions) + "\n"


    # Networking

    def run(self, server):
        while True:
            print(f"TCP server waiting to connect")
            self.conn, addr = server.accept()
            self.conn.setblocking(False)
            with self.conn:
                print(f"TCP server connected by {addr}")
                while True:
                    try:
                        data = self.conn.recv(1024)
                        if not data:
                            print("TCP server disconnect")
                            break
                    except BlockingIOError:
                        pass

                    # Send tracking update to the client
                    with self.network_lock:
                        if self.conn is not None and self.last_update is not None:
                            try:
                                self.conn.sendall(self.last_update.encode('utf-8'))
                            except (BlockingIOError, BrokenPipeError):
                                break
                            self.last_update = None

                    time.sleep(0.01)
            self.conn = None


def main():

    runner = Runner()
    connection = leap.Connection()
    connection.add_listener(runner)

    with connection.open(), \
            socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        connection.set_tracking_mode(leap.TrackingMode.Desktop)
        server.bind((HOST, PORT))
        server.listen()
        runner.run(server)


if __name__ == "__main__":
    main()
