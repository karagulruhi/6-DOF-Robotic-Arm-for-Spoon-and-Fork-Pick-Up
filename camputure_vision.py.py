import numpy as np
import cv2
from tflite_runtime.interpreter import Interpreter
from picamera2 import Picamera2, Preview

# Load the TFLite model
interpreter = Interpreter(model_path="model1.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Initialize PiCamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()

print("PiCamera2 started. Press 'q' to exit.")

# Define the region of interest (ROI) (x1, y1) and (x2, y2) for the area you want to process
x1, y1 = 369, 30   # Starting point of the region
x2, y2 = 509, 355   # Ending point of the region

while True:
    # Capture frame from the camera
    frame = picam2.capture_array()

    # Crop the frame to the defined region
    roi = frame[y1:y2, x1:x2]

    # Preprocess the cropped frame for the model
    img = cv2.resize(roi, (224, 224))  # Resize to model input size
    img = np.expand_dims(img, axis=0)
    img = img / 255.0  # Normalize the image

    # Perform prediction with the TFLite model
    interpreter.set_tensor(input_details[0]['index'], img.astype(np.float32))
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])

    # Get the confidence scores for both classes (spoon and fork)
    spoon_confidence = float(output_data[0][0])  # Confidence for "spoon"
    fork_confidence = 1 - spoon_confidence  # Confidence for "fork"

    # Determine predicted class based on confidence threshold of 85%
    if spoon_confidence >= 0.85:
        label = "spoon"
        confidence = spoon_confidence
    elif fork_confidence >= 0.85:
        label = "fork"
        confidence = fork_confidence
    else:
        label = "Uncertain"
        confidence = None

    # Display the result on the frame
    if confidence is not None:
        cv2.putText(frame, f"{label} ({confidence*100:.2f}%)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "Uncertain", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Draw a rectangle around the region of interest (ROI)
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Show the frame with the processed region
    cv2.imshow("Detection", frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
picam2.stop()
cv2.destroyAllWindows()
