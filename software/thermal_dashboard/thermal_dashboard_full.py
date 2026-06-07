"""
MLX90640 Body Detection & Entry/Exit Counter
NEW: Detect multiple people when large blob appears (side-by-side)
"""

import serial
import numpy as np
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
MIN_HEAD_SIZE = 25              # Minimum pixels for 1 person
MAX_HEAD_SIZE = 300             # Maximum pixels for 1 person
SINGLE_PERSON_SIZE = 100        # Average size for 1 person
MULTI_PERSON_THRESHOLD = 60    # If blob >= 180px, split into 2 people
MAX_HEADS = 10

DOOR_LEFT_EDGE = 9
DOOR_RIGHT_EDGE = 23
DOOR_CENTER = 16
TRACKING_TIMEOUT = 10
MIN_TRACKING_FRAMES = 5
MATCHING_DISTANCE = 30.0        # Relaxed distance for trajectory matching

BUFFER_SIZE = 5

# ==================== GLOBAL VARIABLES ====================
frame_data = np.zeros((ROWS, COLS))
frame_available = False
frame_count = 0
sensor_fps = 0.0
display_fps = 0.0
running = True
frame_times = deque(maxlen=100)
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
    """Manages serial communication with MLX90640 sensor"""
    
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.raw_buffer = bytearray()
        self.lock = threading.Lock()
        
    def connect(self):
        """Establish serial connection"""
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
        """Read available data from serial port"""
        with self.lock:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    new_data = self.ser.read(self.ser.in_waiting)
                    self.raw_buffer.extend(new_data)
                    return len(new_data)
                except Exception as e:
                    print(f"Read error: {e}")
                    return 0
        return 0
    
    def extract_frame(self):
        """Extract complete frame from buffer"""
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
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial closed")

serial_mgr = SerialManager(SERIAL_PORT, BAUD_RATE)

# ==================== FRAME PARSER ====================
def parse_frame_data(frame_str):
    """Parse temperature data from frame string"""
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
        print(f"Parse error: {e}")
        parse_stats['errors'] += 1
        return None

# ==================== MULTI-PERSON DETECTION ====================
def estimate_people_count(size):
    """
    Estimate number of people based on blob size
    MAX 2 people per blob (as requested)
    
    Logic:
    - < 180px  → 1 person
    - >= 180px → 2 people
    """
    if size < MIN_HEAD_SIZE:
        return 0
    
    if size < MULTI_PERSON_THRESHOLD:
        return 1
    else:
        return 2  # MAX 2 people per blob


def split_blob_into_people(frame, mask, num_people):
    """
    Split a large blob into multiple people positions
    
    Strategy: Divide horizontally (side-by-side people)
    """
    y_coords, x_coords = np.where(mask)
    
    if len(x_coords) == 0:
        return []
    
    # Get blob bounds
    x_min, x_max = np.min(x_coords), np.max(x_coords)
    y_min, y_max = np.min(y_coords), np.max(y_coords)
    
    blob_width = x_max - x_min + 1
    temps = frame[mask]
    max_temp = np.max(temps)
    
    positions = []
    
    if num_people == 1:
        # Single person - use centroid
        center_x = np.mean(x_coords)
        center_y = np.mean(y_coords)
        positions.append({
            'pos': (center_x, center_y),
            'temp': max_temp,
            'size': len(x_coords)
        })
    
    else:
        # Multiple people - divide horizontally
        segment_width = blob_width / num_people
        
        for i in range(num_people):
            # Calculate segment bounds
            seg_x_min = x_min + i * segment_width
            seg_x_max = x_min + (i + 1) * segment_width
            
            # Find pixels in this segment
            seg_mask = (x_coords >= seg_x_min) & (x_coords < seg_x_max)
            
            if np.sum(seg_mask) > 0:
                seg_x = x_coords[seg_mask]
                seg_y = y_coords[seg_mask]
                
                center_x = np.mean(seg_x)
                center_y = np.mean(seg_y)
                
                positions.append({
                    'pos': (center_x, center_y),
                    'temp': max_temp,
                    'size': len(x_coords) // num_people  # Divide size equally
                })
    
    return positions


def detect_heads(frame):
    """
    Detect bodies with multi-person support
    
    NEW: Large blobs are split into multiple people
    """
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
            
            # Estimate number of people in this blob
            num_people = estimate_people_count(size)
            
            if num_people == 0:
                continue
            
            # Split blob into individual people
            people_in_blob = split_blob_into_people(frame, mask, num_people)
            
            heads.extend(people_in_blob)
    
    except Exception as e:
        print(f"Detection error: {e}")
    
    return heads


def track_and_count(current_heads):
    """
    Track bodies and count entries/exits
    """
    global tracked_heads, next_head_id, people_inside, total_entered, total_exited, recent_events
    
    current_head_ids = set()
    
    # Match current heads with tracked ones
    for head in current_heads:
        pos = head['pos']
        
        matched_id = None
        min_distance = float('inf')
        
        for head_id, tracked in tracked_heads.items():
            if head_id in current_head_ids:
                continue
            
            last_pos = tracked['pos']
            distance = np.sqrt((pos[0] - last_pos[0])**2 + (pos[1] - last_pos[1])**2)
            
            if distance < 30.0 and distance < min_distance:  # Relaxed from 18.0 to 30.0
                min_distance = distance
                matched_id = head_id
        
        if matched_id is not None:
            tracked_heads[matched_id]['pos'] = pos
            tracked_heads[matched_id]['history'].append(pos)
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
    
    # Check all tracked heads
    to_remove = []
    for head_id, tracked in list(tracked_heads.items()):
        current_x = tracked['pos'][0]
        history = tracked['history']
        frames_tracked = frame_count - tracked.get('first_frame', frame_count)
        
        # Flag to mark if this head has been counted
        counted = False
        
        # Check for boundary crossing (counting)
        if (frames_tracked >= MIN_TRACKING_FRAMES and len(history) >= 2):
            prev_x = history[-2][0]
            curr_x = history[-1][0]
            
            if prev_x >= DOOR_LEFT_EDGE and curr_x < DOOR_LEFT_EDGE:
                people_inside += 1
                total_entered += 1
                event = f"[ENTER] Crossed X={DOOR_LEFT_EDGE} ({prev_x:.1f}->{curr_x:.1f}) Inside: {people_inside}"
                print(event)
                recent_events.append(('ENTER', time.strftime("%H:%M:%S"), people_inside))
                counted = True  # Mark as counted - NOW REMOVE TRAJECTORY
            
            elif prev_x <= DOOR_RIGHT_EDGE and curr_x > DOOR_RIGHT_EDGE:
                people_inside = max(0, people_inside - 1)
                total_exited += 1
                event = f"[EXIT] Crossed X={DOOR_RIGHT_EDGE} ({prev_x:.1f}->{curr_x:.1f}) Inside: {people_inside}"
                print(event)
                recent_events.append(('EXIT', time.strftime("%H:%M:%S"), people_inside))
                counted = True  # Mark as counted - NOW REMOVE TRAJECTORY
        
        # ONLY remove trajectory if counted (crossed boundary)
        if counted:
            to_remove.append(head_id)
            continue
        
        # Otherwise, only remove if disappeared for timeout
        if head_id not in current_head_ids:
            tracked['frames_since_seen'] += 1
            if tracked['frames_since_seen'] >= TRACKING_TIMEOUT:
                to_remove.append(head_id)
    
    # Remove marked heads
    for head_id in to_remove:
        if head_id in tracked_heads:
            del tracked_heads[head_id]


# ==================== DATA PROCESSING THREAD ====================
def data_processing_thread():
    """Background thread: reads data, detects bodies, updates counters"""
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
                    
                    frame_times.append(time.time())
            
            if not frame_extracted:
                time.sleep(0.001)
        except Exception as e:
            print(f"Processing error: {e}")
            time.sleep(0.1)
    
    print("Data processing thread stopped")

# ==================== VISUALIZATION ====================
fig = plt.figure(figsize=(16, 8))
gs = fig.add_gridspec(1, 2, width_ratios=[3, 1])

ax_thermal = fig.add_subplot(gs[0])
thermal_img = ax_thermal.imshow(
    frame_data, cmap='hot', interpolation='bilinear',
    vmin=TEMP_MIN, vmax=TEMP_MAX, aspect='auto', origin='upper'
)

ax_thermal.set_title('MLX90640 - Multi-Person Detection', 
                     fontsize=14, fontweight='bold', pad=12)
ax_thermal.set_xlabel('X [pixels]', fontsize=10)
ax_thermal.set_ylabel('Y [pixels]', fontsize=10)
ax_thermal.grid(True, alpha=0.2, linestyle='--', linewidth=0.5)

left_line = ax_thermal.axvline(x=DOOR_LEFT_EDGE, color='cyan', linestyle='--', 
                                linewidth=2, alpha=0.7, label='Left edge (Entry)')
center_line = ax_thermal.axvline(x=DOOR_CENTER, color='yellow', linestyle=':', 
                                 linewidth=2, alpha=0.5, label='Door center')
right_line = ax_thermal.axvline(x=DOOR_RIGHT_EDGE, color='magenta', linestyle='--', 
                                 linewidth=2, alpha=0.7, label='Right edge (Exit)')
ax_thermal.legend(loc='upper right', fontsize=8)

cbar = fig.colorbar(thermal_img, ax=ax_thermal, fraction=0.046, pad=0.02)
cbar.set_label('Temperature (C)', rotation=270, labelpad=20)

ax_info = fig.add_subplot(gs[1])
ax_info.axis('off')

status_text = ax_info.text(
    0.05, 0.95, '', transform=ax_info.transAxes, fontsize=10, 
    verticalalignment='top',
    bbox=dict(boxstyle='round,pad=0.5', facecolor='#f8f8f8', alpha=0.9)
)

counter_text = ax_info.text(
    0.05, 0.75, '', transform=ax_info.transAxes, fontsize=11, 
    verticalalignment='top', fontweight='bold',
    bbox=dict(boxstyle='round,pad=0.5', facecolor='#ffe6e6', alpha=0.9)
)

detection_text = ax_info.text(
    0.05, 0.50, '', transform=ax_info.transAxes, fontsize=9, 
    verticalalignment='top',
    bbox=dict(boxstyle='round,pad=0.5', facecolor='#e6f3ff', alpha=0.9)
)

events_text = ax_info.text(
    0.05, 0.25, '', transform=ax_info.transAxes, fontsize=8, 
    verticalalignment='top',
    bbox=dict(boxstyle='round,pad=0.5', facecolor='#f0fff0', alpha=0.9)
)

# ==================== ANIMATION UPDATE ====================
def update_display(frame_num):
    """Update visualization with new frame data"""
    global frame_available, display_fps
    
    if frame_available:
        thermal_img.set_data(frame_data)
        frame_available = False
        
        for artist in ax_thermal.patches[:]:
            artist.remove()
        for artist in ax_thermal.texts[:]:
            artist.remove()
        
        # Delete trajectory lines but keep boundary lines
        boundary_lines = {left_line, center_line, right_line}
        for line in ax_thermal.lines[:]:
            if line not in boundary_lines:
                line.remove()
        
        for head in detected_heads:
            x, y = head['pos']
            circle = Circle((x, y), radius=1.2, linewidth=2.5,
                          edgecolor='red', facecolor='none', zorder=10)
            ax_thermal.add_patch(circle)
            
            ax_thermal.text(x, y - 2.5, f"({x:.1f}, {y:.1f})",
                          color='white', fontsize=8, fontweight='bold',
                          ha='center', va='bottom',
                          bbox=dict(boxstyle='round,pad=0.2', 
                                   facecolor='red', alpha=0.8))
        
        for head_id, tracked in tracked_heads.items():
            pos = tracked['pos']
            history = tracked['history']
            
            if len(history) > 1:
                xs = [p[0] for p in history[-10:]]
                ys = [p[1] for p in history[-10:]]
                ax_thermal.plot(xs, ys, 'yellow', linewidth=1.5, 
                              alpha=0.6, zorder=5)
            
            x, y = pos
            ax_thermal.text(x, y + 2.5, f"ID:{head_id}",
                          color='yellow', fontsize=7,
                          ha='center', va='top',
                          bbox=dict(boxstyle='round,pad=0.2', 
                                   facecolor='black', alpha=0.6))
        
        if len(frame_times) >= 2:
            recent = list(frame_times)[-10:]
            if len(recent) >= 2:
                intervals = np.diff(recent)
                if len(intervals) > 0:
                    display_fps = 1.0 / np.mean(intervals)
        
        status_text.set_text(
            f"Status\n"
            f"Frame: {frame_count}\n"
            f"Time: {time.strftime('%H:%M:%S')}\n"
            f"Sensor: {sensor_fps:.1f} FPS\n"
            f"Display: {display_fps:.1f} FPS"
        )
        
        counter_text.set_text(
            f"PEOPLE COUNT\n"
            f"================\n"
            f"Inside: {people_inside} people\n"
            f"================\n"
            f"Total Entered: {total_entered}\n"
            f"Total Exited: {total_exited}"
        )
        
        detection_info = f"Multi-Person Detection\n"
        detection_info += f"Bodies: {len(detected_heads)}\n"
        detection_info += f"Tracking: {len(tracked_heads)}\n"
        detection_info += f"Threshold: {HEAD_TEMP_THRESHOLD}C\n"
        detection_info += f"Match dist: {MATCHING_DISTANCE}px\n"
        detection_info += f"Split at: {MULTI_PERSON_THRESHOLD}px\n\n"
        
        if detected_heads:
            detection_info += "Current Bodies:\n"
            for i, head in enumerate(detected_heads[:5]):
                x, y = head['pos']
                detection_info += f"  ({x:.1f}, {y:.1f}) {head['size']}px\n"
        
        detection_text.set_text(detection_info)
        
        events_info = "Recent Events\n================\n"
        if recent_events:
            for event_type, event_time, count in list(recent_events)[-5:]:
                if event_type == 'ENTER':
                    events_info += f"[+] {event_time} Enter -> {count}\n"
                else:
                    events_info += f"[-] {event_time} Exit -> {count}\n"
        else:
            events_info += "No events yet"
        
        events_text.set_text(events_info)
    
    return [thermal_img, status_text, counter_text, detection_text, events_text]

# ==================== MAIN ====================
def main():
    global running
    
    print("\n" + "="*60)
    print("MLX90640 - Multi-Person Detection System [FIXED]")
    print("="*60)
    print(f"Port: {SERIAL_PORT}")
    print(f"Detection threshold: {HEAD_TEMP_THRESHOLD}C")
    print(f"Single person: ~{SINGLE_PERSON_SIZE}px")
    print(f"Multi-person: >={MULTI_PERSON_THRESHOLD}px -> Split into 2")
    print(f"Matching distance: {MATCHING_DISTANCE}px (relaxed)")
    print("="*60)
    print("Features:")
    print("  - Relaxed trajectory matching (30px) - less line breaking")
    print("  - Large blobs split into MAX 2 people")
    print("  - Trajectory deleted ONLY after counting (crossing boundary)")
    print("  - <180px -> 1 person, >=180px -> 2 people")
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
    
    ani = FuncAnimation(fig, update_display, interval=50, 
                       blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        running = False
        processing_thread.join(timeout=2)
        serial_mgr.close()
        
        print("\n" + "="*60)
        print("FINAL STATISTICS")
        print("="*60)
        print(f"Total frames: {frame_count}")
        print(f"People inside: {people_inside}")
        print(f"Total entered: {total_entered}")
        print(f"Total exited: {total_exited}")
        print("="*60)

if __name__ == "__main__":
    main()