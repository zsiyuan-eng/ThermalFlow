"""
MLX90640 Body Detection - MINIMAL Version (NO GUI)
Pure counting functionality - lightweight and fast
Only outputs when people enter/exit
"""

import serial
import numpy as np
import time
import sys
import threading
from scipy.ndimage import label

# ==================== CONFIGURATION ====================
SERIAL_PORT = 'COM12'
BAUD_RATE = 460800
ROWS = 24
COLS = 32

HEAD_TEMP_THRESHOLD = 25.5
MIN_HEAD_SIZE = 30
MAX_HEAD_SIZE = 300
MULTI_PERSON_THRESHOLD = 180

DOOR_LEFT_EDGE = 11
DOOR_RIGHT_EDGE = 21
TRACKING_TIMEOUT = 10
MIN_TRACKING_FRAMES = 5
MATCHING_DISTANCE = 30.0

# ==================== GLOBAL VARIABLES ====================
frame_count = 0
running = True

detected_heads = []
tracked_heads = {}
next_head_id = 0

people_inside = 0
total_entered = 0
total_exited = 0

# ==================== SERIAL MANAGER ====================
class SerialManager:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.raw_buffer = bytearray()
        self.lock = threading.Lock()
        
    def connect(self):
        max_retries = 3
        for attempt in range(max_retries):
            try:
                print(f"Connecting to {self.port}...")
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=0.1,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False
                )
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                time.sleep(0.5)
                print(f"[OK] Connected to {self.port}")
                return True
            except Exception as e:
                print(f"[ERROR] {e}")
                if attempt < max_retries - 1:
                    time.sleep(2)
        return False
    
    def read_data(self):
        with self.lock:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    new_data = self.ser.read(self.ser.in_waiting)
                    self.raw_buffer.extend(new_data)
                    return len(new_data)
                except:
                    return 0
        return 0
    
    def extract_frame(self):
        with self.lock:
            if len(self.raw_buffer) == 0:
                return None
            try:
                buffer_str = self.raw_buffer.decode('ascii', errors='ignore')
            except:
                self.raw_buffer.clear()
                return None
            
            start_idx = buffer_str.find("--- Start of Frame")
            if start_idx == -1:
                return None
            end_idx = buffer_str.find("--- End of Frame ---", start_idx)
            if end_idx == -1:
                return None
            
            frame_content = buffer_str[start_idx:end_idx]
            bytes_to_remove = len(buffer_str[:end_idx + len("--- End of Frame ---")].encode('ascii'))
            self.raw_buffer = self.raw_buffer[bytes_to_remove:]
            return frame_content
    
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

serial_mgr = SerialManager(SERIAL_PORT, BAUD_RATE)

# ==================== FRAME PARSER ====================
def parse_frame_data(frame_str):
    try:
        lines = frame_str.strip().split('\n')
        if len(lines) < ROWS + 1:
            return None
        
        temp_frame = []
        for i in range(1, min(len(lines), ROWS + 1)):
            line = lines[i].strip()
            if not line:
                continue
            numbers = []
            for part in line.split():
                try:
                    numbers.append(float(part))
                except ValueError:
                    break
            if len(numbers) >= COLS:
                temp_frame.append(numbers[:COLS])
            else:
                temp_frame.append(numbers + [np.nan] * (COLS - len(numbers)))
        
        if len(temp_frame) == ROWS:
            return np.array(temp_frame)
        else:
            return None
    except:
        return None

# ==================== DETECTION ====================
def detect_heads(frame):
    heads = []
    try:
        binary = frame > HEAD_TEMP_THRESHOLD
        labeled, num_features = label(binary)
        
        if num_features == 0:
            return []
        
        for blob_id in range(1, num_features + 1):
            mask = (labeled == blob_id)
            size = np.sum(mask)
            
            if size < MIN_HEAD_SIZE:
                continue
            
            # Estimate people count
            num_people = 2 if size >= MULTI_PERSON_THRESHOLD else 1
            
            y_coords, x_coords = np.where(mask)
            
            if num_people == 1:
                heads.append({
                    'pos': (np.mean(x_coords), np.mean(y_coords)),
                    'size': size
                })
            else:
                # Split horizontally
                x_min, x_max = np.min(x_coords), np.max(x_coords)
                blob_width = x_max - x_min + 1
                segment_width = blob_width / 2
                
                for i in range(2):
                    seg_x_min = x_min + i * segment_width
                    seg_x_max = x_min + (i + 1) * segment_width
                    seg_mask = (x_coords >= seg_x_min) & (x_coords < seg_x_max)
                    
                    if np.sum(seg_mask) > 0:
                        heads.append({
                            'pos': (np.mean(x_coords[seg_mask]), np.mean(y_coords[seg_mask])),
                            'size': size // 2
                        })
    except:
        pass
    
    return heads

# ==================== TRACKING ====================
def track_and_count(current_heads):
    global tracked_heads, next_head_id, people_inside, total_entered, total_exited, frame_count
    
    current_head_ids = set()
    
    # Match current heads
    for head in current_heads:
        pos = head['pos']
        matched_id = None
        min_distance = float('inf')
        
        for head_id, tracked in tracked_heads.items():
            if head_id in current_head_ids:
                continue
            last_pos = tracked['pos']
            distance = np.sqrt((pos[0] - last_pos[0])**2 + (pos[1] - last_pos[1])**2)
            
            if distance < MATCHING_DISTANCE and distance < min_distance:
                min_distance = distance
                matched_id = head_id
        
        if matched_id is not None:
            tracked_heads[matched_id]['pos'] = pos
            tracked_heads[matched_id]['history'].append(pos)
            if len(tracked_heads[matched_id]['history']) > 2:  # Keep only 2 points
                tracked_heads[matched_id]['history'].pop(0)
            tracked_heads[matched_id]['frames_since_seen'] = 0
            current_head_ids.add(matched_id)
        else:
            tracked_heads[next_head_id] = {
                'pos': pos,
                'history': [pos],
                'frames_since_seen': 0,
                'first_frame': frame_count
            }
            current_head_ids.add(next_head_id)
            next_head_id += 1
    
    # Check for crossing and remove
    to_remove = []
    for head_id, tracked in list(tracked_heads.items()):
        history = tracked['history']
        frames_tracked = frame_count - tracked.get('first_frame', frame_count)
        counted = False
        
        if frames_tracked >= MIN_TRACKING_FRAMES and len(history) >= 2:
            prev_x = history[-2][0]
            curr_x = history[-1][0]
            
            if prev_x >= DOOR_LEFT_EDGE and curr_x < DOOR_LEFT_EDGE:
                people_inside += 1
                total_entered += 1
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] ENTER: Inside={people_inside} (Total Entered={total_entered})")
                counted = True
            
            elif prev_x <= DOOR_RIGHT_EDGE and curr_x > DOOR_RIGHT_EDGE:
                people_inside = max(0, people_inside - 1)
                total_exited += 1
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] EXIT: Inside={people_inside} (Total Exited={total_exited})")
                counted = True
        
        if counted:
            to_remove.append(head_id)
            continue
        
        if head_id not in current_head_ids:
            tracked['frames_since_seen'] += 1
            if tracked['frames_since_seen'] >= TRACKING_TIMEOUT:
                to_remove.append(head_id)
    
    for head_id in to_remove:
        if head_id in tracked_heads:
            del tracked_heads[head_id]

# ==================== DATA PROCESSING ====================
def data_processing_thread():
    global frame_count, running, detected_heads
    
    while running:
        try:
            serial_mgr.read_data()
            
            while True:
                frame_str = serial_mgr.extract_frame()
                if frame_str is None:
                    break
                
                parsed_frame = parse_frame_data(frame_str)
                if parsed_frame is not None:
                    frame_count += 1
                    detected_heads = detect_heads(parsed_frame)
                    track_and_count(detected_heads)
            
            time.sleep(0.001)
        except:
            time.sleep(0.1)

# ==================== MAIN ====================
def main():
    global running
    
    print("\n" + "="*60)
    print("MLX90640 - MINIMAL Version (NO GUI)")
    print("="*60)
    print("Features:")
    print("  - No graphics/visualization")
    print("  - Minimal CPU/memory usage")
    print("  - Only outputs when people enter/exit")
    print("  - Press Ctrl+C to stop")
    print("="*60)
    
    if not serial_mgr.connect():
        sys.exit(1)
    
    processing_thread = threading.Thread(target=data_processing_thread, daemon=True)
    processing_thread.start()
    
    print("\n[READY] Monitoring... (will output when people enter/exit)")
    print("="*60 + "\n")
    
    try:
        while running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("STOPPED")
        running = False
        processing_thread.join(timeout=2)
        serial_mgr.close()
        
        print("="*60)
        print("FINAL STATISTICS")
        print("="*60)
        print(f"People Inside: {people_inside}")
        print(f"Total Entered: {total_entered}")
        print(f"Total Exited: {total_exited}")
        print(f"Total Frames Processed: {frame_count}")
        print("="*60)

if __name__ == "__main__":
    main()