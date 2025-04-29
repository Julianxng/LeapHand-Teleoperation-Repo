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
        # print(f"Frame {event.tracking_frame_id} with {len(event.hands)} hands.")
        for hand in event.hands:
            # hand_type = "left" if str(hand.type) == "HandType.Left" else "right"
            # output_str = f"Hand id {hand.id} is a {hand_type} hand with position ({hand.palm.position.x}, {hand.palm.position.y - 300}, {hand.palm.position.z})."
            self.last_update = f"{hand.palm.position.x},{hand.palm.position.y - 300},{hand.palm.position.z}\n"
        with self.network_lock: 
            if self.conn is not None and not self.last_update is None:
                self.conn.sendall(self.last_update.encode('utf-8'))
                self.last_update = None


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
                    time.sleep(0.1)
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
