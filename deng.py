# Edge Impulse - OpenMV FOMO Object Detection Example with 4 vertical zones and 16 grid zones
#
# This work is licensed under the MIT license.
# Copyright (c) 2013-2024 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

import sensor, image, time, ml, math, uos, gc
from machine import UART
import display

# Initialize sensor
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((320, 240))
sensor.skip_frames(time=2000)
sensor.set_vflip(True)
lcd = display.SPIDisplay()

# Initialize UART
uart = UART(3, 115200)  # Use UART(1) for OpenMV RT

# Mode flags
CLASS_MODE = 0  # 0: idle, 1: class mode, 2: self-study mode

# Load model
net = None
labels = None
min_confidence = 0.5

try:
    # load the model, alloc the model file on the heap if we have at least 64K free after loading
    net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64 * 1024)))
except Exception as e:
    raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

colors = [ # Add more colors if you are detecting more than 7 types of classes at once.
    (255,   0,   0),
    (  0, 255,   0),
    (255, 255,   0),
    (  0,   0, 255),
    (255,   0, 255),
    (  0, 255, 255),
    (255, 255, 255),
]

threshold_list = [(math.ceil(min_confidence * 255), 255)]

# Define 4 vertical zones (each 80 pixels wide)
ZONE_WIDTH = 80
ZONE_COUNT = 4
zones = [(i * ZONE_WIDTH, 0, ZONE_WIDTH, 240) for i in range(ZONE_COUNT)]

# Define 12 square zones for self-study mode (3 rows, 4 columns)
GRID_COLS = 4
GRID_ROWS = 3
GRID_WIDTH = 320 // GRID_COLS
GRID_HEIGHT = 240 // GRID_ROWS
grid_zones = [(col * GRID_WIDTH, row * GRID_HEIGHT, GRID_WIDTH, GRID_HEIGHT)
             for row in range(GRID_ROWS) for col in range(GRID_COLS)]

# Detection stabilization variables
DETECTION_HISTORY_LENGTH = 5  # Number of frames to consider for stabilization
detection_history = []  # Will store recent detection results

# Kalman Filter implementation
class KalmanFilter:
    def __init__(self, initial_x, initial_y):
        # State vector: [x, y, vx, vy]
        self.state = [initial_x, initial_y, 0, 0]  # Position and velocity

        # State covariance matrix
        self.P = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]

        # Process noise covariance
        self.Q = [
            [0.01, 0, 0, 0],
            [0, 0.01, 0, 0],
            [0, 0, 0.01, 0],
            [0, 0, 0, 0.01]
        ]

        # Measurement noise covariance
        self.R = [
            [1, 0],
            [0, 1]
        ]

        # State transition matrix (assuming constant velocity model)
        self.F = [
            [1, 0, 1, 0],  # x = x + vx
            [0, 1, 0, 1],  # y = y + vy
            [0, 0, 1, 0],  # vx = vx
            [0, 0, 0, 1]   # vy = vy
        ]

        # Observation matrix (we only observe position)
        self.H = [
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ]

    def predict(self):
        # Predict state
        self.state = [
            self.F[0][0]*self.state[0] + self.F[0][2]*self.state[2],
            self.F[1][1]*self.state[1] + self.F[1][3]*self.state[3],
            self.F[2][2]*self.state[2],
            self.F[3][3]*self.state[3]
        ]

        # Predict state covariance
        self.P = self.matrix_mult(self.F, self.matrix_mult(self.P, self.matrix_transpose(self.F)))
        self.P = self.matrix_add(self.P, self.Q)

        return (self.state[0], self.state[1])

    def update(self, measured_x, measured_y):
        # Measurement vector
        z = [measured_x, measured_y]

        # Calculate innovation
        y = [
            z[0] - (self.H[0][0]*self.state[0] + self.H[0][1]*self.state[1]),
            z[1] - (self.H[1][0]*self.state[0] + self.H[1][1]*self.state[1])
        ]

        # Calculate innovation covariance
        S = self.matrix_add(
            self.matrix_mult(self.H, self.matrix_mult(self.P, self.matrix_transpose(self.H))),
            self.R
        )

        # Calculate Kalman gain
        K = self.matrix_mult(
            self.matrix_mult(self.P, self.matrix_transpose(self.H)),
            self.matrix_inv(S)
        )

        # Update state estimate
        self.state = [
            self.state[0] + K[0][0]*y[0] + K[0][1]*y[1],
            self.state[1] + K[1][0]*y[0] + K[1][1]*y[1],
            self.state[2] + K[2][0]*y[0] + K[2][1]*y[1],
            self.state[3] + K[3][0]*y[0] + K[3][1]*y[1]
        ]

        # Update state covariance
        I = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]
        KH = self.matrix_mult(K, self.H)
        I_KH = self.matrix_sub(I, KH)
        self.P = self.matrix_mult(I_KH, self.P)

        return (self.state[0], self.state[1])

    def matrix_mult(self, A, B):
        # Matrix multiplication
        result = [[0 for _ in range(len(B[0]))] for _ in range(len(A))]
        for i in range(len(A)):
            for j in range(len(B[0])):
                for k in range(len(B)):
                    result[i][j] += A[i][k] * B[k][j]
        return result

    def matrix_transpose(self, A):
        # Matrix transpose
        return [[A[j][i] for j in range(len(A))] for i in range(len(A[0]))]

    def matrix_add(self, A, B):
        # Matrix addition
        return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

    def matrix_sub(self, A, B):
        # Matrix subtraction
        return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

    def matrix_inv(self, A):
        # Matrix inversion (2x2 only)
        det = A[0][0]*A[1][1] - A[0][1]*A[1][0]
        return [
            [A[1][1]/det, -A[0][1]/det],
            [-A[1][0]/det, A[0][0]/det]
        ]

# Dictionary to store Kalman filters for each detected object
object_filters = {}

def fomo_post_process(model, inputs, outputs):
    ob, oh, ow, oc = model.output_shape[0]

    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh

    scale = min(x_scale, y_scale)

    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (ow * scale)) / 2) + inputs[0].roi[1]

    l = [[] for i in range(oc)]

    for i in range(oc):
        img = image.Image(outputs[0][0, :, :, i] * 255)
        blobs = img.find_blobs(
            threshold_list, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1
        )
        for b in blobs:
            rect = b.rect()
            x, y, w, h = rect
            score = (
                img.get_statistics(thresholds=threshold_list, roi=rect).l_mean() / 255.0
            )
            x = int((x * scale) + x_offset)
            y = int((y * scale) + y_offset)
            w = int(w * scale)
            h = int(h * scale)
            l[i].append((x, y, w, h, score))
    return l

def check_uart_command():
    global CLASS_MODE
    if uart.any():
        data = uart.read()
        hex_data = [hex(b) for b in data]
        print("Received UART data:", hex_data)

        # Check for class mode command: 0x55 0x02 0x02 0x00 0xDD
        if len(data) >= 5 and data[0] == 0x55 and data[1] == 0x02 and data[2] == 0x02 and data[3] == 0x00 and data[4] == 0xDD:
            CLASS_MODE = 1
            print("Entering Class Mode")

        # Check for self-study mode command: 0x55 0x02 0x03 0x00 0xDD
        elif len(data) >= 5 and data[0] == 0x55 and data[1] == 0x02 and data[2] == 0x03 and data[3] == 0x00 and data[4] == 0xDD:
            CLASS_MODE = 2
            print("Entering Self-Study Mode")

        # Check for exit command: 0x55 0x02 0x01 0x00 0xDD
        elif len(data) >= 5 and data[0] == 0x55 and data[1] == 0x02 and data[2] == 0x01 and data[3] == 0x00 and data[4] == 0xDD:
            if CLASS_MODE in [1, 2]:  # Exit either class or self-study mode
                CLASS_MODE = 0
                print("Exiting Current Mode")

def get_zone(x_position):
    """Determine which zone the x position falls into"""
    return min(int(x_position / ZONE_WIDTH), ZONE_COUNT - 1)

def get_grid_zone(x, y):
    """Determine which grid zone the position falls into (1-12)"""
    col = min(int(x / GRID_WIDTH), GRID_COLS - 1)
    row = min(int(y / GRID_HEIGHT), GRID_ROWS - 1)
    return row * GRID_COLS + col + 1  # Return 1-12

def find_min_detection_zones(detection_counts):
    """Find zones with minimum detection counts (only zones 1-3), including zeros"""
    # Get counts for zones 1-3 only
    zone_counts = detection_counts[1:4]

    # Find minimum count (including zeros)
    min_count = min(zone_counts)

    # Get zones with minimum count (zones 1-3)
    min_zones = []
    for i in range(1, 4):
        if detection_counts[i] == min_count:
            min_zones.append(i)

    return min_zones

def send_zone_report(min_zones):
    """Send the appropriate UART message based on which zones have minimum detections"""
    if not min_zones:  # Shouldn't happen as we're always checking zones 1-3
        message = bytes([0x55, 0x01, 0x00, 0x00, 0xDD])
        print("No zones found (error case)")
    elif len(min_zones) == 1:  # Single zone with minimum detections
        zone = min_zones[0]
        if zone == 1:
            message = bytes([0x55, 0x01, 0xBB, 0xBB, 0xDD])
        elif zone == 2:
            message = bytes([0x55, 0x01, 0xDD, 0xDD, 0xDD])
        elif zone == 3:
            message = bytes([0x55, 0x01, 0xEE, 0xEE, 0xDD])
        print(f"Zone {zone} has minimum detections")
    else:  # Multiple zones with same minimum count
        if set(min_zones) == {1, 2}:
            message = bytes([0x55, 0x99, 0x99, 0xCC, 0xDD])
        elif set(min_zones) == {1, 3}:
            message = bytes([0x55, 0x01, 0xAA, 0xAA, 0xDD])
        elif set(min_zones) == {2, 3}:
            message = bytes([0x55, 0x01, 0xCC, 0xCC, 0xDD])
        else:  # All three zones have same count
            message = bytes([0x55, 0x01, 0xFF, 0xFF, 0xDD])
        print(f"Zones {min_zones} tied for minimum detections")

    uart.write(message)
    print(f"Sent message: {[hex(b) for b in message]}")

def process_grid_detections(img, detections):
    """Process detections in the 12 grid zones and send UART data"""
    # Initialize ceshi message
    ceshi = bytearray([0x55, 0x01, 0x90, 0x09, 0xDD])

    # Initialize grid detection status (1-12)
    grid_detected = [False] * 12

    # Process all detections
    for i, detection_list in enumerate(detections):
        if i == 0: continue  # background class
        if len(detection_list) == 0: continue  # no detections for this class?

        for x, y, w, h, score in detection_list:
            # Use Kalman filter for stabilization in self-study mode
            center_x = math.floor(x + (w / 2))
            center_y = math.floor(y + (h / 2))

            # Create a unique ID for this object based on position and class
            object_id = f"{i}_{center_x}_{center_y}"

            # Get or create Kalman filter for this object
            if object_id not in object_filters:
                object_filters[object_id] = KalmanFilter(center_x, center_y)

            # Update Kalman filter with current measurement
            kf = object_filters[object_id]
            predicted_x, predicted_y = kf.predict()
            filtered_x, filtered_y = kf.update(center_x, center_y)

            # Use filtered position for zone determination
            zone_num = get_grid_zone(filtered_x, filtered_y) - 1  # Convert to 0-11 index
            if 0 <= zone_num < 12:
                grid_detected[zone_num] = True

    # Draw grid zones and numbers
    for i, zone in enumerate(grid_zones):
        img.draw_rectangle(zone, color=(128, 128, 128), thickness=1)
        img.draw_string(zone[0] + 10, zone[1] + 10, str(i+1), color=(255, 255, 255))

        # Mark detected zones
        if grid_detected[i]:
            img.draw_rectangle(zone, color=(0, 255, 0), thickness=2)

    # Update ceshi message based on grid detections
    for i in range(12):
        zone_num = i + 1  # 1-12
        detected = grid_detected[i]
        if zone_num == 1:
            if detected:
                ceshi[2] |= 0x08
            else:
                ceshi[2] &= 0xF7
        elif zone_num == 2:
            if detected:
                ceshi[2] |= 0x04
            else:
                ceshi[2] &= 0xFB
        elif zone_num == 3:
            if detected:
                ceshi[2] |= 0x02
            else:
                ceshi[2] &= 0xFD
        elif zone_num == 4:
            if detected:
                ceshi[2] |= 0x01
            else:
                ceshi[2] &= 0xFE
        elif zone_num == 5:
            if detected:
                ceshi[3] |= 0x80
            else:
                ceshi[3] &= 0x7F
        elif zone_num == 6:
            if detected:
                ceshi[3] |= 0x40
            else:
                ceshi[3] &= 0xBF
        elif zone_num == 7:
            if detected:
                ceshi[3] |= 0x20
            else:
                ceshi[3] &= 0xDF
        elif zone_num == 8:
            if detected:
                ceshi[3] |= 0x10
            else:
                ceshi[3] &= 0xEF
        elif zone_num == 10:
            if detected:
                ceshi[3] |= 0x04
            else:
                ceshi[3] &= 0xFB
        elif zone_num == 11:
            if detected:
                ceshi[3] |= 0x02
            else:
                ceshi[3] &= 0xFD
    # Send the message
    uart.write(ceshi)
    print(f"Sent grid message: {[hex(b) for b in ceshi]}")

def stabilize_detections(current_detections):
    """Apply stabilization to detections using a simple moving average"""
    global detection_history

    # Add current detections to history
    detection_history.append(current_detections)

    # Keep only the most recent frames
    if len(detection_history) > DETECTION_HISTORY_LENGTH:
        detection_history.pop(0)

    # If we don't have enough history yet, return current detections
    if len(detection_history) < DETECTION_HISTORY_LENGTH:
        return current_detections

    # Calculate average positions for each detection
    stabilized_detections = []
    for class_idx in range(len(current_detections)):
        class_detections = []

        # Collect all detections for this class from history
        all_class_detections = []
        for frame in detection_history:
            if class_idx < len(frame):
                all_class_detections.extend(frame[class_idx])

        # Group nearby detections (simple clustering)
        clusters = []
        for det in all_class_detections:
            x, y, w, h, score = det
            matched = False
            for cluster in clusters:
                cx, cy, cw, ch, cscore, count = cluster
                # Check if this detection is close to the cluster center
                if abs(x - cx) < 20 and abs(y - cy) < 20:
                    # Update cluster with new detection
                    new_count = count + 1
                    new_x = (cx * count + x) / new_count
                    new_y = (cy * count + y) / new_count
                    new_w = (cw * count + w) / new_count
                    new_h = (ch * count + h) / new_count
                    new_score = (cscore * count + score) / new_count
                    cluster[:] = [new_x, new_y, new_w, new_h, new_score, new_count]
                    matched = True
                    break
            if not matched:
                clusters.append([x, y, w, h, score, 1])

        # Only keep clusters that appear in at least half of the frames
        min_cluster_count = DETECTION_HISTORY_LENGTH // 2
        for cluster in clusters:
            x, y, w, h, score, count = cluster
            if count >= min_cluster_count:
                class_detections.append((int(x), int(y), int(w), int(h), score))

        stabilized_detections.append(class_detections)

    return stabilized_detections

clock = time.clock()
while(True):
    clock.tick()

    # Check for UART commands
    check_uart_command()

    img = sensor.snapshot()

    # Draw zone boundaries for visualization
    for i, zone in enumerate(zones):
        img.draw_rectangle(zone, color=(128, 128, 128), thickness=1)
        img.draw_string(zone[0] + 5, 10, f"Zone {i}", color=(255, 255, 255))

    # Get all detections first
    raw_detections = net.predict([img], callback=fomo_post_process)

    # Apply stabilization to detections
    stabilized_detections = stabilize_detections(raw_detections)

    # Track detections in each zone (initialize counts to 0)
    zone_detection_counts = [0] * ZONE_COUNT

    # Clear old object filters (we'll recreate them for current detections)
    current_object_ids = set()

    for i, detection_list in enumerate(stabilized_detections):
        if i == 0: continue  # background class
        if len(detection_list) == 0: continue  # no detections for this class?

        print(" %s " % labels[i])

        for x, y, w, h, score in detection_list:
            center_x = math.floor(x + (w / 2))
            center_y = math.floor(y + (h / 2))
            print(f"x {center_x}\ty {center_y}\tsscore {score}")

            # Create a unique ID for this object based on position and class
            object_id = f"{i}_{center_x}_{center_y}"
            current_object_ids.add(object_id)

            # Get or create Kalman filter for this object
            if object_id not in object_filters:
                object_filters[object_id] = KalmanFilter(center_x, center_y)

            # Update Kalman filter with current measurement
            kf = object_filters[object_id]
            predicted_x, predicted_y = kf.predict()
            filtered_x, filtered_y = kf.update(center_x, center_y)

            # Draw original detection (red)
            img.draw_circle((center_x, center_y, 12), color=colors[i])
            img.draw_rectangle((x, y, w, h), color=colors[i], thickness=1)

            # Draw predicted position (blue)
            img.draw_circle((int(predicted_x), int(predicted_y), 8), color=(0, 0, 255))

            # Draw filtered position (green)
            img.draw_circle((int(filtered_x), int(filtered_y), 4), color=(0, 255, 0))

            # Use filtered position for zone determination
            zone_idx = get_zone(filtered_x)
            zone_detection_counts[zone_idx] += 1

            # Draw zone indicator
            img.draw_string(int(filtered_x) - 10, int(filtered_y) - 20, f"Z{zone_idx}", color=colors[i])

    # Clean up object filters (remove those not detected in this frame)
    for object_id in list(object_filters.keys()):
        if object_id not in current_object_ids:
            del object_filters[object_id]

    # Handle different modes
    if CLASS_MODE == 1:  # Class mode - now runs continuously until exit command
        print(f"Zone detection counts: {zone_detection_counts}")

        # Find zones with minimum detections (only zones 1-3), including zeros
        min_zones = find_min_detection_zones(zone_detection_counts)

        # Check if all zones have zero detections
        if all(count == 0 for count in zone_detection_counts[1:4]):
            min_zones = []  # This will trigger the "all zeros" case

        # Send appropriate UART message
        send_zone_report(min_zones)

        # Don't exit CLASS_MODE here - it will continue running until exit command is received

    elif CLASS_MODE == 2:  # Self-study mode
        # Process the 12 grid zones with stabilized detections
        process_grid_detections(img, stabilized_detections)
        # Note: We don't automatically exit self-study mode here anymore
        # It will only exit when receiving the specific UART command

    # Display the image
    lcd.write(img)

    print(clock.fps(), "fps", end="\n\n")
