#!/usr/bin/env python3
import cv2
import time
import smbus
from ultralytics import YOLO

# ===== GoPiGo3 I2C Setup =====
BUS = 1
ADDR = 0x08
bus = smbus.SMBus(BUS)

FWD       = 115
BWD       = 119
LEFT      = 97
RIGHT     = 100
STOP      = 120
SET_SPEED = 105

def send(cmd, data=None):
    if data is None:
        data = [0, 0, 0]
    elif isinstance(data, int):
        data = [data, 0, 0]
    else:
        data = list(data)[:3] + [0] * (3 - len(data))
    bus.write_i2c_block_data(ADDR, cmd, data)
    time.sleep(0.005)

def set_speed(speed):
    send(SET_SPEED, speed)

def forward():
    send(FWD)

def backward():
    send(BWD)

def turn_left():
    send(LEFT)

def turn_right():
    send(RIGHT)

def stop():
    send(STOP)

# ===== YOLO Setup =====
model = YOLO('/home/nealgoogs/GUnitFinalProject/epoch150_nano_sgd/weights/best.pt')

def detect():
    """Capture image and run detection."""
    cap = cv2.VideoCapture(0)
    time.sleep(0.5)
    for _ in range(5):
        ret, frame = cap.read()
    cap.release()

    if not ret:
        return []

    results = model(frame, conf=0.3)[0]

    detections = []
    for box in results.boxes:
        cls_name = model.names[int(box.cls[0])]
        conf = float(box.conf[0])
        detections.append({'class': cls_name, 'confidence': conf})

    return detections

# ===== Main Loop =====
print("Starting detection + movement...")
print("Press Ctrl+C to stop")

set_speed(150)

try:
    while True:
        detections = detect()

        if not detections:
            print("Nothing detected - spinning to search")
            turn_left()
            time.sleep(0.3)
            stop()
        else:
            for d in detections:
                print(f"Detected: {d['class']} ({d['confidence']:.2f})")

            # Get highest confidence detection
            best = max(detections, key=lambda x: x['confidence'])

            if best['class'] == 'bad_guy':
                print("BAD GUY! Backing up!")
                backward()
                time.sleep(0.5)
                stop()

            elif best['class'] == 'vehicle':
                print("Vehicle - turning right")
                turn_right()
                time.sleep(0.3)
                stop()

        time.sleep(0.5)  # Pause between detections

except KeyboardInterrupt:
    print("\nStopping...")
    stop()