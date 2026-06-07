"""
MLX90640 Head Detection & Entry/Exit Counter - WiFi Version
Connect to ESP32 via WiFi instead of USB serial
"""

import socket
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
import time
import sys
from collections import deque
import threading
from scipy.ndimage import label

# ==================== WIFI CONFIGURATION ====================
ESP32_IP = "192.168.4.1"        # ESP32 AP的IP地址（通常是这个）
ESP32_PORT = 8888               # TCP端口
RECONNECT_DELAY = 5             # 重连延迟（秒）

# ==================== SENSOR CONFIGURATION ====================
ROWS = 24
COLS = 32

# Temperature settings
TEMP_MIN = 20.0
TEMP_MAX = 40.0

# Head detection parameters
HEAD_TEMP_THRESHOLD = 32.5
MIN_HEAD_SIZE = 15
MAX_HEAD_SIZE = 120
MAX_HEADS = 10

# Entry/Exit detection
DOOR_LEFT_EDGE = 5
DOOR_RIGHT_EDGE = 27
DOOR_CENTER = 16
TRACKING_TIMEOUT = 10
MIN_TRACKING_FRAMES = 5

# ==================== GLOBAL VARIABLES ====================
frame_data = np.zeros((ROWS, COLS))
frame_available = False
frame_count = 0
sensor_fps = 0.0
display_fps = 0.0
running = True
frame_times = deque(maxlen=100)
parse_stats = {'success': 0, 'errors': 0, 'incomplete': 0}

# Head tracking
detected_heads = []
tracked_heads = {}
next_head_id = 0

# Counting
people_inside = 0
total_entered = 0
total_exited = 0
recent_events = deque(maxlen=5)

# ==================== WIFI MANAGER ====================
class WiFiManager:
    """WiFi连接管理器 - 替代串口管理器"""
    
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.raw_buffer = bytearray()
        self.lock = threading.Lock()
        self.connected = False
        
    def connect(self):
        """连接到ESP32"""
        max_retries = 3
        for attempt in range(max_retries):
            try:
                print(f"Connecting to ESP32 at {self.ip}:{self.port} ({attempt+1}/{max_retries})...")
                
                # 创建TCP socket
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5.0)  # 5秒超时
                
                # 连接到ESP32
                self.sock.connect((self.ip, self.port))
                self.sock.settimeout(0.1)  # 连接后改为短超时
                
                self.connected = True
                print(f"✅ Connected to ESP32!")
                
                # 读取欢迎消息
                try:
                    welcome = self.sock.recv(1024).decode('utf-8', errors='ignore')
                    if welcome:
                        print(f"ESP32 says: {welcome.strip()}")
                except:
                    pass
                
                return True
                
            except Exception as e:
                print(f"❌ Connection failed: {e}")
                if self.sock:
                    self.sock.close()
                if attempt < max_retries - 1:
                    print(f"Retrying in 2 seconds...")
                    time.sleep(2)
        
        print(f"CRITICAL: Failed to connect after {max_retries} attempts")
        return False
    
    def read_data(self):
        """从WiFi读取数据"""
        with self.lock:
            if not self.sock or not self.connected:
                return 0
            
            try:
                # 尝试接收数据
                new_data = self.sock.recv(4096)  # 一次读取4KB
                if new_data:
                    self.raw_buffer.extend(new_data)
                    return len(new_data)
                else:
                    # 收到空数据，连接可能断开
                    self.connected = False
                    return 0
                    
            except socket.timeout:
                # 超时，正常情况
                return 0
            except Exception as e:
                print(f"Read error: {e}")
                self.connected = False
                return 0
    
    def extract_frame(self):
        """从缓冲区提取完整帧（与串口版本相同）"""
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
        """关闭连接"""
        self.connected = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            print("WiFi connection closed")

wifi_mgr = WiFiManager(ESP32_IP, ESP32_PORT)

# ==================== FRAME PARSER ====================
def parse_frame_data(frame_str):
    """解析帧数据（与串口版本完全相同）"""
    global parse_stats, sensor_fps
    
    try:
        lines = frame_str.strip().split('\n')
        if len(lines) < ROWS + 1:
            parse_stats['incomplete'] += 1
            return None
        
        # Extract FPS
        if "FPS:" in lines[0]:
            try:
                fps_part = lines[0].split("FPS:")[1].split(")")[0]
                sensor_fps = float(fps_part)
            except:
                pass
        
        # Parse temperatures
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

# ==================== HEAD DETECTION (相同) ====================
def detect_heads(frame):
    """检测头部"""
    heads = []
    try:
        binary = frame > HEAD_TEMP_THRESHOLD
        labeled, num_features = label(binary)
        if num_features == 0:
            return []
        
        for head_id in range(1, min(num_features + 1, MAX_HEADS + 1)):
            mask = (labeled == head_id)
            size = np.sum(mask)
            if size < MIN_HEAD_SIZE or size > MAX_HEAD_SIZE:
                continue
            
            y_coords, x_coords = np.where(mask)
            center_x = np.mean(x_coords)
            center_y = np.mean(y_coords)
            temps = frame[mask]
            max_temp = np.max(temps)
            
            heads.append({
                'pos': (center_x, center_y),
                'temp': max_temp,
                'size': size
            })
    except Exception as e:
        print(f"Detection error: {e}")
    return heads

def track_and_count(current_heads):
    """跟踪并统计（与串口版本完全相同）"""
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
            if distance < 8.0 and distance < min_distance:
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
    
    to_remove = []
    for head_id, tracked in list(tracked_heads.items()):
        if head_id not in current_head_ids:
            tracked['frames_since_seen'] += 1
            if tracked['frames_since_seen'] >= TRACKING_TIMEOUT:
                frames_tracked = frame_count - tracked.get('first_frame', frame_count)
                if frames_tracked >= MIN_TRACKING_FRAMES and len(tracked['history']) >= MIN_TRACKING_FRAMES:
                    last_x = tracked['pos'][0]
                    if last_x < DOOR_LEFT_EDGE:
                        people_inside += 1
                        total_entered += 1
                        event = f"🚶➡️ 进入！(X={last_x:.1f}) 房间人数: {people_inside}"
                        print(event)
                        recent_events.append(('ENTER', time.strftime("%H:%M:%S"), people_inside))
                    elif last_x > DOOR_RIGHT_EDGE:
                        people_inside = max(0, people_inside - 1)
                        total_exited += 1
                        event = f"🚶⬅️ 离开！(X={last_x:.1f}) 房间人数: {people_inside}"
                        print(event)
                        recent_events.append(('EXIT', time.strftime("%H:%M:%S"), people_inside))
                to_remove.append(head_id)
    
    for head_id in to_remove:
        del tracked_heads[head_id]

# ==================== DATA PROCESSING THREAD ====================
def data_processing_thread():
    """后台线程：通过WiFi读取数据"""
    global frame_data, frame_available, frame_count, running, detected_heads
    
    print("Data processing thread started")
    
    while running:
        try:
            # 检查连接状态
            if not wifi_mgr.connected:
                print("⚠️ Connection lost, attempting to reconnect...")
                if wifi_mgr.connect():
                    print("✅ Reconnected!")
                else:
                    print(f"❌ Reconnect failed, retrying in {RECONNECT_DELAY}s...")
                    time.sleep(RECONNECT_DELAY)
                    continue
            
            # 读取数据
            bytes_read = wifi_mgr.read_data()
            
            # 提取并解析帧
            frame_extracted = False
            while True:
                frame_str = wifi_mgr.extract_frame()
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

# ==================== VISUALIZATION (与串口版本完全相同) ====================
fig = plt.figure(figsize=(16, 8))
gs = fig.add_gridspec(1, 2, width_ratios=[3, 1])

ax_thermal = fig.add_subplot(gs[0])
thermal_img = ax_thermal.imshow(
    frame_data, cmap='hot', interpolation='bilinear',
    vmin=TEMP_MIN, vmax=TEMP_MAX, aspect='auto', origin='upper'
)

ax_thermal.set_title('MLX90640 - WiFi Version', fontsize=14, fontweight='bold', pad=12)
ax_thermal.set_xlabel('X [pixels]', fontsize=10)
ax_thermal.set_ylabel('Y [pixels]', fontsize=10)
ax_thermal.grid(True, alpha=0.2, linestyle='--', linewidth=0.5)

left_line = ax_thermal.axvline(x=DOOR_LEFT_EDGE, color='cyan', linestyle='--', 
                                linewidth=2, alpha=0.7, label='Left edge')
center_line = ax_thermal.axvline(x=DOOR_CENTER, color='yellow', linestyle=':', 
                                 linewidth=2, alpha=0.5, label='Center')
right_line = ax_thermal.axvline(x=DOOR_RIGHT_EDGE, color='magenta', linestyle='--', 
                                 linewidth=2, alpha=0.7, label='Right edge')
ax_thermal.legend(loc='upper right', fontsize=8)

cbar = fig.colorbar(thermal_img, ax=ax_thermal, fraction=0.046, pad=0.02)
cbar.set_label('Temperature (°C)', rotation=270, labelpad=20)

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

def update_display(frame_num):
    """更新显示"""
    global frame_available, display_fps
    
    if frame_available:
        thermal_img.set_data(frame_data)
        frame_available = False
        
        for artist in ax_thermal.patches[:]:
            artist.remove()
        for artist in ax_thermal.texts[:]:
            artist.remove()
        
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
        
        conn_status = "🟢 Connected" if wifi_mgr.connected else "🔴 Disconnected"
        status_text.set_text(
            f"📊 Status\n"
            f"Connection: {conn_status}\n"
            f"Frame: {frame_count}\n"
            f"Time: {time.strftime('%H:%M:%S')}\n"
            f"Sensor: {sensor_fps:.1f} FPS\n"
            f"Display: {display_fps:.1f} FPS"
        )
        
        counter_text.set_text(
            f"👥 PEOPLE COUNT\n"
            f"━━━━━━━━━━━━━━━━\n"
            f"Inside: {people_inside} people\n"
            f"━━━━━━━━━━━━━━━━\n"
            f"Total Entered: {total_entered}\n"
            f"Total Exited: {total_exited}"
        )
        
        detection_info = f"🎯 Detection\n"
        detection_info += f"Heads: {len(detected_heads)}\n"
        detection_info += f"Tracking: {len(tracked_heads)}\n"
        detection_info += f"Threshold: {HEAD_TEMP_THRESHOLD}°C\n\n"
        if detected_heads:
            detection_info += "Current Heads:\n"
            for i, head in enumerate(detected_heads[:5]):
                x, y = head['pos']
                detection_info += f"  ({x:.1f}, {y:.1f})\n"
        detection_text.set_text(detection_info)
        
        events_info = "📋 Recent Events\n━━━━━━━━━━━━━━\n"
        if recent_events:
            for event_type, event_time, count in list(recent_events)[-5:]:
                if event_type == 'ENTER':
                    events_info += f"🟢 {event_time} Enter → {count}\n"
                else:
                    events_info += f"🔴 {event_time} Exit → {count}\n"
        else:
            events_info += "No events yet"
        events_text.set_text(events_info)
    
    return [thermal_img, status_text, counter_text, detection_text, events_text]

# ==================== MAIN ====================
def main():
    global running
    
    print("\n" + "="*60)
    print("MLX90640 - WiFi Version")
    print("="*60)
    print(f"ESP32 IP: {ESP32_IP}")
    print(f"TCP Port: {ESP32_PORT}")
    print("="*60)
    print("\n⚠️ IMPORTANT:")
    print("1. Upload Arduino code to ESP32 first")
    print("2. Connect your computer to WiFi: MLX90640_AP")
    print("3. WiFi password: 12345678")
    print("4. Then run this program")
    print("="*60 + "\n")
    
    if not wifi_mgr.connect():
        print("\n❌ Failed to connect. Please check:")
        print("   1. ESP32 is powered on")
        print("   2. You're connected to 'MLX90640_AP' WiFi")
        print("   3. IP address is correct (usually 192.168.4.1)")
        sys.exit(1)
    
    processing_thread = threading.Thread(target=data_processing_thread, daemon=True)
    processing_thread.start()
    
    print("Waiting for data...")
    start_wait = time.time()
    while frame_count == 0 and (time.time() - start_wait) < 10:
        time.sleep(1)
    
    if frame_count == 0:
        print("ERROR: No data received")
        wifi_mgr.close()
        sys.exit(1)
    
    print("✅ Starting visualization...\n")
    
    ani = FuncAnimation(fig, update_display, interval=50, 
                       blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n⏹️ Stopped")
    finally:
        running = False
        processing_thread.join(timeout=2)
        wifi_mgr.close()
        
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
