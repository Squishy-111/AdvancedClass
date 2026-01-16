import cv2
import numpy as np
from webots_camera import grab_image

# --- Configuration ---
tcp_ip = '127.0.0.1'
tcp_port = 5599
# ---------------------

while True:
    img = grab_image(tcp_port, tcp_ip)

    if img is not None:
    
        cv2.imshow('webots camera feed', img)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cv2.destroyAllWindows()