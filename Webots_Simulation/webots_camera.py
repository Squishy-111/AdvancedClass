import socket
import struct
import numpy as np
import cv2

# --- Configuration ---
TCP_IP = '127.0.0.1'
TCP_PORT = 5599
# ---------------------
def grab_image(tcp_port, tcp_ip):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        print(f"Connecting to {tcp_ip}:{tcp_port}...")
        s.connect((tcp_ip, tcp_port))
        print("Connected!")

        # Corresponds to your screenshot's header size
        header_size = struct.calcsize("=HH")

        while True:
            # 1. Receive Header
            header = b''
            while len(header) < header_size:
                chunk = s.recv(header_size - len(header))
                if not chunk: break
                header += chunk
            
            if len(header) != header_size:
                print("Connection lost (header mismatch)")
                break

            # 2. Parse Header (Width, Height)
            width, height = struct.unpack("=HH", header)

            # 3. Calculate expected payload size (RGB = w * h * 3)
            bytes_to_read = width * height * 3
        
            # 4. Receive Image Data
            img_data = b''
            while len(img_data) < bytes_to_read:
                # Request up to 4096 bytes at a time, or whatever is left
                remaining = bytes_to_read - len(img_data)
                chunk = s.recv(min(remaining, 4096))
                if not chunk: break
                img_data += chunk
            
            if len(img_data) != bytes_to_read:
                break

            # 5. Convert to Numpy Array
            # Reshape directly to (height, width, 3) for RGB
            img = np.frombuffer(img_data, np.uint8).reshape((height, width,3))

            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            # 6. Display
            return img

    except Exception as e:
        print(f"Error fetching image: {e}")
        return None
        
    finally:
        # This ensures the socket is closed, but doesn't override the return value
        s.close()