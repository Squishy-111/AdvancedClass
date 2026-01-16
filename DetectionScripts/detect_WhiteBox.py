from ultralytics import YOLO
import cv2

# Load your trained model
model = YOLO(r"C:\Users\camel\Desktop\AdvancedClass\AdvancedClass\box_best.pt")

# Open webcam (CAP_DSHOW is for windows users, delete second arg for MAC)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) 
if not cap.isOpened():
    print("Failed to open camera")
    raise SystemExit

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO model on the current frame
    results = model(frame, stream=True, verbose=False)

    # Draw all boxes, regardless of confidence
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])*100 # get percentage value instead of decimal
            cls = int(box.cls[0]) # grabs the class index but I have only one class which is the whitebox
            label = model.names.get(cls) #grabs the id of the class using the class index

            # Draw bounding box and label
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame, f"{label} {conf:.1f}%",(x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Show webcam feed
    cv2.imshow("WhiteBox Detection", frame)

    # Press ESC to quit
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()




                