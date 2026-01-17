import socket
import struct
import numpy as np
import cv2

#code to get image from webots to opencv format
class CameraClient:
    def __init__(self, tcp_ip='127.0.0.1', tcp_port=5599):
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.sock = None
        self.header_size = struct.calcsize("=HH")
        
        # Connect immediately upon initialization
        self.connect()

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print(f"Connecting to Camera at {self.tcp_ip}:{self.tcp_port}...")
            self.sock.connect((self.tcp_ip, self.tcp_port))
            print("Camera Connected!")
        except Exception as e:
            print(f"Failed to connect to camera: {e}")
            self.sock = None

    def get_frame(self):
        if self.sock is None:
            return None

        try:
            # 1. Receive Header (Width, Height)
            header = b''
            while len(header) < self.header_size:
                chunk = self.sock.recv(self.header_size - len(header))
                if not chunk: 
                    raise ConnectionError("Socket closed during header recv")
                header += chunk
            
            width, height = struct.unpack("=HH", header)

            # 2. Calculate Payload Size
            bytes_to_read = width * height * 3
        
            # 3. Receive Image Data
            img_data = b''
            while len(img_data) < bytes_to_read:
                # Read in larger chunks (4096) for speed
                remaining = bytes_to_read - len(img_data)
                chunk = self.sock.recv(min(remaining, 4096))
                if not chunk: 
                    raise ConnectionError("Socket closed during payload recv")
                img_data += chunk
            
            # 4. Convert to Numpy
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 3))
            
            # 5. Convert RGB to BGR for OpenCV
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img

        except Exception as e:
            print(f"Camera Error: {e}")
            # Optional: Try to reconnect automatically?
            # self.connect() 
            return None

    def close(self):
        if self.sock:
            self.sock.close()