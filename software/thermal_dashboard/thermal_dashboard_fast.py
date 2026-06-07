"""
MLX90640 Body Detection - OPTIMIZED Version
Performance improvements for high pixel count scenarios
"""

import serial
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use faster backend
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
import time
import sys
from collections import deque
import threading
from scipy.ndimage import label

# ==================== CONFIGURATION ====================
SERIAL_PORT = 'COM12'
BAUD_RATE = 460800
ROWS = 24
COLS = 32

TEMP_MIN = 20.0
TEMP_MAX = 40.0

HEAD_TEMP_THRESHOLD = 25.5
MIN_HEAD_SIZE = 15
MAX_HEAD_SIZE = 300
SINGLE_PERSON_SIZE = 50
MULTI_PERSON_THRESHOLD = 180
MAX_HEADS = 10

DOOR_LEFT_EDGE = 12
DOOR_RIGHT_EDGE = 20
DOOR_CENTER = 16
TRACKING_TIMEOUT = 10
MIN_TRACKING_FRAMES = 5
MATCHING_DISTANCE = 30.0

# Performance optimizations
MAX_TRAJECTORY_POINTS = 5  # Reduced from 10
UPDATE_INTERVAL = 100      # Slower refresh (100ms instead of 50ms)

# ==================== GLOBAL VARIABLES ====================
frame_data = np.zeros((ROWS, COLS))
frame_available = False
frame_count = 0
sensor_fps = 0.0
running = True
parse_stats = {'success': 0, 'errors': 0, 'incomplete': 0}

detected_heads = []
tracked_heads = {}
next_head_id = 0

people_inside = 0
total_entered = 0
total_exited = 0
recent_events = deque(maxlen=5)

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
                print(f"Connecting to {self.port} ({attempt+1}/{max_retries})...")
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
                print(f"Connected to {self.port}")
                return True
            except Exception as e:
                print(f"Failed: {e}")
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
                except Exception as e:
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
            print("Serial closed")

serial_mgr = SerialManager(SERIAL_PORT, BAUD_RATE)

# ==================== FRAME PARSER ====================
def parse_frame_data(frame_str):
    global parse_stats, sensor_fps
    
    try:
        lines = frame_str.strip().split('\n')
        if len(lines) < ROWS + 1:
            parse_stats['incomplete'] += 1
            return None
        
        if "FPS:" in lines[0]:
            try:
                fps_part = lines[0].split("FPS:")[1].split(")")[0]
                sensor_fps = float(fps_part)
            except:
                pass
        
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
            parse_stats['success'] += 1
            return np.array(temp_frame)
        else:
            parse_stats['incomplete'] += 1
            return None
    except Exception as e:
        parse_stats['errors'] += 1
        return None

# ==================== OPTIMIZED DETECTION ====================
def estimate_people_count(size):
    if size < MIN_HEAD_SIZE:
        return 0
    if size < MULTI_PERSON_THRESHOLD:
        return 1
    else:
        return 2

def split_blob_into_people(frame, mask, num_people):
    y_coords, x_coords = np.where(mask)
    
    if len(x_coords) == 0:
        return []
    
    x_min, x_max = np.min(x_coords), np.max(x_coords)
    y_min, y_max = np.min(y_coords), np.max(y_coords)
    blob_width = x_max - x_min + 1
    temps = frame[mask]
    max_temp = np.max(temps)
    
    positions = []
    
    if num_people == 1:
        center_x = np.mean(x_coords)
        center_y = np.mean(y_coords)
        positions.append({
            'pos': (center_x, center_y),
            'temp': max_temp,
            'size': len(x_coords)
        })
    else:
        segment_width = blob_width / num_people
        for i in range(num_people):
            seg_x_min = x_min + i * segment_width
            seg_x_max = x_min + (i + 1) * segment_width
            seg_mask = (x_coords >= seg_x_min) & (x_coords < seg_x_max)
            
            if np.sum(seg_mask) > 0:
                seg_x = x_coords[seg_mask]
                seg_y = y_coords[seg_mask]
                center_x = np.mean(seg_x)
                center_y = np.mean(seg_y)
                positions.append({
                    'pos': (center_x, center_y),
                    'temp': max_temp,
                    'size': len(x_coords) // num_people
                })
    
    return positions

def detect_heads(frame):
    heads = []
    try:
        binary = frame > HEAD_TEMP_THRESHOLD
        labeled, num_features = label(binary)
        
        if num_features == 0:
            return []
        
        for blob_id in range(1, min(num_features + 1, MAX_HEADS + 1)):
            mask = (labeled == blob_id)
            size = np.sum(mask)
            
            if size < MIN_HEAD_SIZE:
                continue
            
            num_people = estimate_people_count(size)
            if num_people == 0:
                continue
            
            people_in_blob = split_blob_into_people(frame, mask, num_people)
            heads.extend(people_in_blob)
    
    except Exception as e:
        pass
    
    return heads

def track_and_count(current_heads):
    global tracked_heads, next_head_id, people_inside, total_entered, total_exited, recent_events
    
    current_head_ids = set()
    
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
            # Keep only last N points
            if len(tracked_heads[matched_id]['history']) > MAX_TRAJECTORY_POINTS:
                tracked_heads[matched_id]['history'].pop(0)
            tracked_heads[matched_id]['frames_since_seen'] = 0
            tracked_heads[matched_id]['temp'] = head['temp']
            current_head_ids.add(matched_id)
        else:
            tracked_heads[next_head_id] = {
                'pos': pos,
                'history': [pos],
                'frames_since_seen': 0,
                'temp': head['temp'],
                'first_frame': frame_count
            }
            current_head_ids.add(next_head_id)
            next_head_id += 1
    
    to_remove = []
    for head_id, tracked in list(tracked_heads.items()):
        current_x = tracked['pos'][0]
        history = tracked['history']
        frames_tracked = frame_count - tracked.get('first_frame', frame_count)
        counted = False
        
        if (frames_tracked >= MIN_TRACKING_FRAMES and len(history) >= 2):
            prev_x = history[-2][0]
            curr_x = history[-1][0]
            
            if prev_x >= DOOR_LEFT_EDGE and curr_x < DOOR_LEFT_EDGE:
                people_inside += 1
                total_entered += 1
                print(f"[ENTER] Inside: {people_inside}")
                recent_events.append(('ENTER', time.strftime("%H:%M:%S"), people_inside))
                counted = True
            
            elif prev_x <= DOOR_RIGHT_EDGE and curr_x > DOOR_RIGHT_EDGE:
                people_inside = max(0, people_inside - 1)
                total_exited += 1
                print(f"[EXIT] Inside: {people_inside}")
                recent_events.append(('EXIT', time.strftime("%H:%M:%S"), people_inside))
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

# ==================== DATA PROCESSING THREAD ====================
def data_processing_thread():
    global frame_data, frame_available, frame_count, running, detected_heads
    
    print("Data processing thread started")
    
    while running:
        try:
            serial_mgr.read_data()
            
            frame_extracted = False
            while True:
                frame_str = serial_mgr.extract_frame()
                if frame_str is None:
                    break
                
                parsed_frame = parse_frame_data(frame_str)
                if parsed_frame is not None:
                    frame_data = parsed_frame
                    frame_count += 1
                    frame_available = True
                    frame_extracted = True
                    
                    detected_heads = detect_heads(parsed_frame)
                    track_and_count(detected_heads)
            
            if not frame_extracted:
                time.sleep(0.001)
        except Exception as e:
            time.sleep(0.1)
    
    print("Data processing thread stopped")

# ==================== OPTIMIZED VISUALIZATION ====================
plt.ioff()  # Turn off interactive mode
fig = plt.figure(figsize=(14, 7))
gs = fig.add_gridspec(1, 2, width_ratios=[3, 1])

ax_thermal = fig.add_subplot(gs[0])
thermal_img = ax_thermal.imshow(
    frame_data, cmap='hot', interpolation='nearest',  # nearest is faster than bilinear
    vmin=TEMP_MIN, vmax=TEMP_MAX, aspect='auto', origin='upper'
)

ax_thermal.set_title('MLX90640 - OPTIMIZED', fontsize=14, fontweight='bold')
ax_thermal.set_xlabel('X', fontsize=10)
ax_thermal.set_ylabel('Y', fontsize=10)
ax_thermal.grid(True, alpha=0.2)

left_line = ax_thermal.axvline(x=DOOR_LEFT_EDGE, color='cyan', linestyle='--', 
                                linewidth=2, alpha=0.7)
center_line = ax_thermal.axvline(x=DOOR_CENTER, color='yellow', linestyle=':', 
                                 linewidth=2, alpha=0.5)
right_line = ax_thermal.axvline(x=DOOR_RIGHT_EDGE, color='magenta', linestyle='--', 
                                 linewidth=2, alpha=0.7)

cbar = fig.colorbar(thermal_img, ax=ax_thermal, fraction=0.046, pad=0.02)
cbar.set_label('Temperature (C)', rotation=270, labelpad=15)

ax_info = fig.add_subplot(gs[1])
ax_info.axis('off')

status_text = ax_info.text(0.05, 0.95, '', transform=ax_info.transAxes, fontsize=10, 
                          verticalalignment='top')
counter_text = ax_info.text(0.05, 0.70, '', transform=ax_info.transAxes, fontsize=12, 
                           verticalalignment='top', fontweight='bold')

def update_display(frame_num):
    global frame_available
    
    if frame_available:
        thermal_img.set_data(frame_data)
        frame_available = False
        
        # Clear old drawings
        for artist in ax_thermal.patches[:]:
            artist.remove()
        for artist in ax_thermal.texts[:]:
            artist.remove()
        
        boundary_lines = {left_line, center_line, right_line}
        for line in ax_thermal.lines[:]:
            if line not in boundary_lines:
                line.remove()
        
        # Draw detections (simplified)
        for head in detected_heads:
            x, y = head['pos']
            circle = Circle((x, y), radius=1.0, linewidth=2,
                          edgecolor='red', facecolor='none', zorder=10)
            ax_thermal.add_patch(circle)
        
        # Draw trajectories (simplified - no labels)
        for tracked in tracked_heads.values():
            history = tracked['history']
            if len(history) > 1:
                xs = [p[0] for p in history]
                ys = [p[1] for p in history]
                ax_thermal.plot(xs, ys, 'yellow', linewidth=1, alpha=0.5, zorder=5)
        
        # Update text (minimal)
        status_text.set_text(
            f"Frame: {frame_count}\n"
            f"Bodies: {len(detected_heads)}\n"
            f"Tracking: {len(tracked_heads)}"
        )
        
        counter_text.set_text(
            f"INSIDE: {people_inside}\n"
            f"================\n"
            f"Entered: {total_entered}\n"
            f"Exited: {total_exited}"
        )
    
    return [thermal_img, status_text, counter_text]

# ==================== MAIN ====================
def main():
    global running
    
    print("\n" + "="*60)
    print("MLX90640 - OPTIMIZED Version")
    print("="*60)
    print("Optimizations:")
    print("  - Faster matplotlib backend")
    print("  - Reduced trajectory points (5 instead of 10)")
    print("  - Slower refresh rate (100ms instead of 50ms)")
    print("  - Simplified rendering")
    print("="*60)
    
    if not serial_mgr.connect():
        sys.exit(1)
    
    processing_thread = threading.Thread(target=data_processing_thread, daemon=True)
    processing_thread.start()
    
    print("Waiting for data...")
    start_wait = time.time()
    while frame_count == 0 and (time.time() - start_wait) < 10:
        time.sleep(1)
    
    if frame_count == 0:
        print("ERROR: No data received")
        serial_mgr.close()
        sys.exit(1)
    
    print("Starting visualization...\n")
    
    ani = FuncAnimation(fig, update_display, interval=UPDATE_INTERVAL, 
                       blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        running = False
        processing_thread.join(timeout=2)
        serial_mgr.close()
        
        print(f"\nFinal: Inside={people_inside}, Entered={total_entered}, Exited={total_exited}")

if __name__ == "__main__":
    main()