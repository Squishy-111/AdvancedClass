import socket
import struct
import numpy as np
import cv2

# --- Configuration ---
TCP_IP = '127.0.0.1'
TCP_PORT = 5599
# ---------------------

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    print(f"Connecting to {TCP_IP}:{TCP_PORT}...")
    s.connect((TCP_IP, TCP_PORT))
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

        # 6. Display
        cv2.imshow("Webots RGB Feed", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"Error: {e}")
finally:
    s.close()
    cv2.destroyAllWindows()