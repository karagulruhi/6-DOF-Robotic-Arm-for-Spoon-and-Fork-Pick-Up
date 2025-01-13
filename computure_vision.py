from picamera2 import Picamera2
import numpy as np
import torch
import cv2
import serial
import time
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)

# Attempt to establish a connection with the Arduino
try:
    arduino = serial.Serial('/dev/ttyUSB0', 9600)  # Port and baud rate
    arduino_connected = True
except serial.SerialException:
    print("Arduino not connected. Default value '0' will be used.")
    arduino = None
    arduino_connected = False

# Load the YOLOv5-Tiny model
model = torch.hub.load("ultralytics/yolov5", "custom", path="best.pt")
model.eval()

# Set confidence and IoU thresholds
model.conf = 0.6
model.iou = 0.45

# Initialize the PiCamera2
picam2 = Picamera2()
picam2.start()

# Define relevant classes and ROI coordinates
relevant_classes = ['fork', 'spoon']
region_x_min, region_y_min = 340, 60
region_x_max, region_y_max = 600, 300

while True:
    # Capture a frame from the camera
    frame = picam2.capture_array()

    # Perform inference with the YOLOv5 model
    results = model(frame)
    detections = results.xyxy[0].cpu().numpy()  # Bounding boxes
    labels = results.names

    # Default data to send to Arduino
    coords = "0\n"  # If no objects are detected or Arduino is not connected

    for det in detections:
        xmin, ymin, xmax, ymax, conf, class_id = det
        if (
            xmin >= region_x_min and ymin >= region_y_min and
            xmax <= region_x_max and ymax <= region_y_max
        ):
            label = labels[int(class_id)]
            if label in relevant_classes:
                # Assign object type: "1" for spoon, "0" for fork
                object_type = "1" if label == "spoon" else "0"
                coords = f"{int(xmin)},{int(ymin)},{int(xmax)},{int(ymax)},{object_type}\n"

                # Visualize the detection
                color = (0, 255, 0) if object_type == "1" else (0, 0, 255)
                cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), color, 2)
                cv2.putText(frame, label, (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # Send data to Arduino if connected
    if arduino_connected:
        arduino.write(coords.encode())
        print(f"Arduino qSent value: {coords.strip()}")
        time.sleep(0.1)  # Small delay between transmissions
    else:
        print(f"Arduino not connected. Sent value: {coords.strip()}")

    # Draw the ROI on the frame
    roi_color = (255, 0, 0)  # Blue for the ROI
    cv2.rectangle(frame, (region_x_min, region_y_min), (region_x_max, region_y_max), roi_color, 2)
    cv2.putText(frame, "ROI", (region_x_min, region_y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, roi_color, 2)

    # Display the frame with detections
    cv2.imshow("Detection", frame)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the camera and close all OpenCV windows
picam2.stop()
cv2.destroyAllWindows()
if arduino_connected:
    arduino.close()
