
import warnings
from picamera2 import Picamera2
import numpy as np
import torch
import cv2
import serial
import time
import threading
warnings.filterwarnings("ignore", category=FutureWarning)
picam2 = Picamera2()
picam2.start()
model = torch.hub.load("ultralytics/yolov5", "custom", path="best.pt")
model.conf = 0.6
model.iou = 0.45

# Serial communication setup
arduino = None
arduino_connected = False
arduino_busy = False
serial_lock = threading.Lock()
relevant_classes = ['fork', 'spoon']
ROI = (250, 0, 560, 260)  # x_min, y_min, x_max, y_max

def arduino_reader():
    """Read status messages from Arduino"""
    global arduino_busy, arduino_connected, arduino
    while True:
        try:
            if arduino and arduino.in_waiting > 0:
                with serial_lock:
                    message = arduino.readline().decode().strip()
                    print(message)
                    if message == "PROCESSING_START":
                        arduino_busy = True
                        print("[ARDUINO] Busy processing")
                    elif message == "READY":
                        arduino_busy = False
                        print("[ARDUINO] Ready for new data")
        except Exception as e:
            print(f"[ERROR] Serial read: {str(e)}")
            arduino_connected = False
        time.sleep(0.1)

def initialize_arduino():
    """Initialize Arduino connection"""
    global arduino, arduino_connected
    try:
        arduino = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=9600,
            timeout=1,
            write_timeout=1
        )
        arduino_connected = True
        print("[ARDUINO] Connection established")
        threading.Thread(target=arduino_reader, daemon=True).start()
        return True
    except serial.SerialException:
        print("[ARDUINO] Connection failed")
        arduino_connected = False
        return False

def send_to_arduino(data):
    """Send data to Arduino with safety checks"""
    global arduino, arduino_connected
    if not arduino_connected:
        initialize_arduino()
    
    if arduino_connected and not arduino_busy:
        try:
            with serial_lock:
                arduino.write(f"{data}\n".encode())
                print(f"[SENT] {data.strip()}")
        except Exception as e:
            print(f"[ERROR] Send failed: {str(e)}")
            arduino_connected = initialize_arduino()
    elif arduino_busy:
        print("[WARNING] Arduino busy - skipping")
    else:
        print("[ERROR] Arduino not connected")

# Initialize connection
initialize_arduino()


try:
    while True:
        frame = picam2.capture_array()
        results = model(frame)
        detections = results.xyxy[0].cpu().numpy()
        
        # Default coordinates
        coords = "0\n"
        
        # Process detectionsq
        for det in detections:
            xmin, ymin, xmax, ymax, conf, cls = det
            label = model.names[int(cls)]
            
            if (xmin >= ROI[0] and ymin >= ROI[1] and
                xmax <= ROI[2] and ymax <= ROI[3] and
                label in relevant_classes):
                
                obj_type = "1" if label == "spoon" else "0"
                coords = f"{int(xmin)},{int(ymin)},{int(xmax)},{int(ymax)},{obj_type}\n"
                
                # Visualization
                color = (0, 255, 0) if obj_type == "1" else (0, 0, 255)
                cv2.rectangle(frame, (int(xmin), int(ymin)), 
                            (int(xmax), int(ymax)), color, 2)
                cv2.putText(frame, f"{label} {conf:.2f}", 
                          (int(xmin), int(ymin)-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                break  # Process only first valid detection

        # Draw ROI and send data
        cv2.rectangle(frame, (ROI[0], ROI[1]), (ROI[2], ROI[3]), (255,0,0), 2)
        send_to_arduino(coords)
        
        # Display and exit control
        cv2.imshow("Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    if arduino_connected:
        arduino.close()
