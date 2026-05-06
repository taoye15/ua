import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import json
import os
from datetime import datetime
from typing import List, Dict, Optional, Tuple, Any
import pandas as pd
from dataclasses import dataclass, field

# ==================== 配置常量 ====================
@dataclass
class Config:
    """系统配置类"""
    SCHOOL_CENTER_GCJ: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A_GCJ: List[float] = field(default_factory=lambda: [118.746956, 32.232945])
    DEFAULT_B_GCJ: List[float] = field(default_factory=lambda: [118.751589, 32.235204])
    
    CONFIG_FILE: str = "obstacle_config.json"
    BACKUP_DIR: str = "backups"
    DEFAULT_SAFETY_RADIUS_METERS: int = 5
    MAX_BACKUP_FILES: int = 10
    
    BASE_SPEED_MPS: float = 5.0
    HEARTBEAT_INTERVAL: float = 0.2
    VOLTAGE_VARIATION: float = 0.5
    SAT_RANGE: Tuple[int, int] = (8, 14)
    
    GAODE_SATELLITE_URL: str = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    GAODE_VECTOR_URL: str = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
    
    VERTICAL_OFFSET_MULTIPLIER: float = 3.0
    WAYPOINT_OFFSET_FACTOR: float = 10.0

config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)

# ==================== 几何函数 ====================
def point_in_polygon(point: List[float], polygon: List[List[float]]) -> bool:
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

def on_segment(p: List[float], q: List[float], r: List[float]) -> bool:
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))

def orientation(p: List[float], q: List[float], r: List[float]) -> int:
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) < 1e-10:
        return 0
    return 1 if val > 0 else 2

def segments_intersect(p1: List[float], p2: List[float], p3: List[float], p4: List[float]) -> bool:
    o1 = orientation(p1, p2, p3)
    o2 = orientation(p1, p2, p4)
    o3 = orientation(p3, p4, p1)
    o4 = orientation(p3, p4, p2)
    
    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and on_segment(p1, p3, p2):
        return True
    if o2 == 0 and on_segment(p1, p4, p2):
        return True
    if o3 == 0 and on_segment(p3, p1, p4):
        return True
    if o4 == 0 and on_segment(p3, p2, p4):
        return True
    return False

def line_intersects_polygon(p1: List[float], p2: List[float], polygon: List[List[float]]) -> bool:
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i + 1) % n]
        if segments_intersect(p1, p2, p3, p4):
            return True
    return False

def distance(p1: List[float], p2: List[float]) -> float:
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def get_polygon_bounds(polygon: List[List[float]]) -> Optional[Dict]:
    if not polygon:
        return None
    min_lng = min(p[0] for p in polygon)
    max_lng = max(p[0] for p in polygon)
    min_lat = min(p[1] for p in polygon)
    max_lat = max(p[1] for p in polygon)
    return {
        'min_lng': min_lng, 'max_lng': max_lng,
        'min_lat': min_lat, 'max_lat': max_lat,
        'center_lng': (min_lng + max_lng) / 2,
        'center_lat': (min_lat + max_lat) / 2
    }

def validate_polygon(polygon: List[List[float]]) -> bool:
    return len(polygon) >= 3

def meters_to_deg(meters: float, lat: float = 32.23) -> Tuple[float, float]:
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg

def point_to_segment_distance_deg(point: List[float], seg_start: List[float], seg_end: List[float]) -> float:
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx*dx + dy*dy
    
    if len_sq == 0:
        return math.sqrt((px-x1)**2 + (py-y1)**2)
    
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0, min(1, t))
    
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    
    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)

def point_to_segment_distance_meters(point: List[float], seg_start: List[float], seg_end: List[float]) -> float:
    return point_to_segment_distance_deg(point, seg_start, seg_end) * 111000

def check_safety_radius(drone_pos: List[float], obstacles_gcj: List[Dict], flight_altitude: float, safety_radius: float) -> Tuple[bool, Optional[float], Optional[str]]:
    if not drone_pos:
        return True, None, None
    
    min_distance = float('inf')
    danger_name = None
    
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        obs_height = obs.get('height', 30)
        
        if obs_height <= flight_altitude:
            continue
        
        if coords and len(coords) >= 3:
            for i in range(len(coords)):
                p1 = coords[i]
                p2 = coords[(i + 1) % len(coords)]
                dist_m = point_to_segment_distance_meters(drone_pos, p1, p2)
                
                if dist_m < min_distance:
                    min_distance = dist_m
                    danger_name = obs.get('name', '障碍物')
    
    if min_distance < safety_radius:
        return False, min_distance, danger_name
    return True, min_distance if min_distance != float('inf') else None, None

# ==================== 障碍物管理 ====================
def cleanup_old_backups():
    try:
        backup_files = [f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE)]
        if len(backup_files) > config.MAX_BACKUP_FILES:
            backup_files.sort()
            for old_file in backup_files[:-config.MAX_BACKUP_FILES]:
                os.remove(os.path.join(config.BACKUP_DIR, old_file))
    except Exception as e:
        st.warning(f"清理备份文件时出错: {e}")

def backup_config() -> Optional[str]:
    if os.path.exists(config.CONFIG_FILE):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_name = f"{config.BACKUP_DIR}/{config.CONFIG_FILE}.{timestamp}.bak"
        try:
            import shutil
            shutil.copy(config.CONFIG_FILE, backup_name)
            cleanup_old_backups()
            return backup_name
        except Exception as e:
            st.error(f"备份失败: {e}")
    return None

def load_obstacles() -> List[Dict]:
    if os.path.exists(config.CONFIG_FILE):
        try:
            with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                obstacles = data.get('obstacles', [])
                for obs in obstacles:
                    if 'selected' not in obs:
                        obs['selected'] = False
                    if 'height' not in obs:
                        obs['height'] = 30
                return obstacles
        except (json.JSONDecodeError, IOError) as e:
            st.error(f"加载配置文件失败: {e}")
            return []
    return []

def save_obstacles(obstacles: List[Dict]) -> bool:
    try:
        backup_config()
        data = {
            'obstacles': obstacles,
            'count': len(obstacles),
            'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'version': 'v13.1'
        }
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        st.error(f"保存失败: {e}")
        return False

def get_latest_backup() -> Optional[str]:
    try:
        backup_files = [f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')]
        if backup_files:
            backup_files.sort(reverse=True)
            return os.path.join(config.BACKUP_DIR, backup_files[0])
    except Exception as e:
        st.error(f"获取备份文件失败: {e}")
    return None

def restore_from_backup(backup_path: str) -> bool:
    try:
        with open(backup_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            obstacles = data.get('obstacles', [])
            save_obstacles(obstacles)
            return True
    except Exception as e:
        st.error(f"恢复备份失败: {e}")
        return False

# ==================== 绕行算法 ====================
def get_blocking_obstacles(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float) -> List[Dict]:
    blocking = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking.append(obs)
    return blocking

def find_left_path(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    """
    向左绕行：从顶部绕过障碍物
    第1段（起点→点1）：最长，垂直向上飞到很高处
    第2段（点1→点2）：次长，水平向右飞过障碍物顶部
    第3段（点2→终点）：最短，垂直向下到终点
    """
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking_obs:
        return [start, end]
    
    # 计算所有阻挡障碍物的整体边界
    max_lng = -float('inf')
    max_lat = -float('inf')
    min_lat = float('inf')
    
    for obs in blocking_obs:
        coords = obs.get('polygon', [])
        if coords:
            for point in coords:
                max_lng = max(max_lng, point[0])
                max_lat = max(max_lat, point[1])
                min_lat = min(min_lat, point[1])
    
    if max_lng == -float('inf'):
        return [start, end]
    
    # 安全偏移距离（米转度）
    safe_lng, safe_lat = meters_to_deg(safety_radius * 3)
    
    # 计算障碍物的高度
    obstacle_height = max_lat - min_lat
    
    # 第1段：起点 → 点1（垂直向上，距离最长）
    # 向上飞到障碍物顶部上方很远处（障碍物高度的3倍 + 安全偏移）
    point1 = [
        start[0] + 0.0012,  # ⬅️ 经度增加 0.0012
        max_lat + obstacle_height * 3 + safe_lat * 5 + 0.0002
    ]
    
    # 第2段：点1 → 点2（水平向右，距离次长）
    # 向右飞过整个障碍物宽度 + 额外距离
    point2 = [
        max_lng + obstacle_height * 2 + safe_lng * 3,  # 向右很远
        point1[1]  # Y坐标不变（保持高度）
    ]
    
    # 第3段：点2 → 终点（垂直向下，距离最短）
    point3 = end
    
    # 构建路径
    path = [start, point1, point2, point3]
    
    return path
    
def find_right_path(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking_obs:
        return [start, end]
    
    mid_x = (start[0] + end[0]) / 2
    mid_y = (start[1] + end[1]) / 2
    
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.sqrt(dx*dx + dy*dy)
    
    if length == 0:
        return [start, end]
    
    perp_x = dy / length
    perp_y = -dx / length
    
    offset_dist = safety_radius * config.WAYPOINT_OFFSET_FACTOR
    lat_rad = math.radians(mid_y)
    lng_scale = 111000 * math.cos(lat_rad)
    lat_scale = 111000
    
    offset_x = perp_x * offset_dist / lng_scale
    offset_y = perp_y * offset_dist / lat_scale
    
    waypoint = [mid_x + offset_x, mid_y + offset_y]
    
    return [start, waypoint, end]

def calculate_path_length(path: List[List[float]]) -> float:
    total = 0.0
    for i in range(len(path) - 1):
        total += distance(path[i], path[i + 1])
    return total

def find_best_path(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    left_path = find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    right_path = find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    
    left_len = calculate_path_length(left_path)
    right_len = calculate_path_length(right_path)
    
    return left_path if left_len < right_len else right_path

def create_avoidance_path(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float, direction: str, safety_radius: float = 5) -> List[List[float]]:
    if direction == "向左绕行":
        return find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    elif direction == "向右绕行":
        return find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    else:
        return find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius)

# ==================== 心跳包模拟器 ====================
@dataclass
class HeartbeatData:
    timestamp: str
    flight_time: float
    lat: float
    lng: float
    altitude: float
    voltage: float
    satellites: int
    speed: float
    progress: float
    arrived: bool
    safety_violation: bool
    remaining_distance: float

class HeartbeatSimulator:
    def __init__(self, start_point_gcj: List[float]):
        self.history: List[HeartbeatData] = []
        self.current_pos: List[float] = start_point_gcj.copy()
        self.path: List[List[float]] = [start_point_gcj.copy()]
        self.path_index: int = 0
        self.simulating: bool = False
        self.flight_altitude: float = 50
        self.speed: int = 50
        self.progress: float = 0.0
        self.total_distance: float = 0.0
        self.distance_traveled: float = 0.0
        self.safety_radius: float = config.DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation: bool = False
        self.start_time: Optional[datetime] = None
        self.flight_log: List[HeartbeatData] = []
        self.last_update_time: Optional[float] = None
        
    def set_path(self, path: List[List[float]], altitude: float = 50, speed: int = 50, safety_radius: float = 5):
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.safety_radius = safety_radius
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.safety_violation = False
        self.start_time = datetime.now()
        self.last_update_time = None
        
        self.total_distance = 0.0
        for i in range(len(path) - 1):
            self.total_distance += distance(path[i], path[i + 1])
    
    def update_and_generate(self, obstacles_gcj: List[Dict]) -> Optional[HeartbeatData]:
        if not self.simulating or self.path_index >= len(self.path) - 1:
            if self.simulating:
                self.simulating = False
            return None
        
        current_time = time.time()
        if self.last_update_time is None:
            delta_time = config.HEARTBEAT_INTERVAL
        else:
            delta_time = min(0.5, current_time - self.last_update_time)
        self.last_update_time = current_time
        
        start = self.path[self.path_index]
        end = self.path[self.path_index + 1]
        segment_distance = distance(start, end)
        
        speed_m_per_s = config.BASE_SPEED_MPS * (self.speed / 100)
        move_distance = speed_m_per_s * delta_time
        
        self.distance_traveled += move_distance
        
        if self.total_distance > 0:
            self.progress = min(1.0, self.distance_traveled / self.total_distance)
        
        if self.distance_traveled >= segment_distance and self.distance_traveled > 0:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                return self._generate_heartbeat(True)
        else:
            if segment_distance > 0:
                t = min(1.0, max(0.0, self.distance_traveled / segment_distance))
                lng = start[0] + (end[0] - start[0]) * t
                lat = start[1] + (end[1] - start[1]) * t
                self.current_pos = [lng, lat]
        
        safe, _, _ = check_safety_radius(self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius)
        if not safe:
            self.safety_violation = True
        
        return self._generate_heartbeat(False)
    
    def _generate_heartbeat(self, arrived: bool = False) -> HeartbeatData:
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        
        heartbeat = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"),
            flight_time=flight_time,
            lat=self.current_pos[1],
            lng=self.current_pos[0],
            altitude=self.flight_altitude,
            voltage=round(22.2 + random.uniform(-config.VOLTAGE_VARIATION, config.VOLTAGE_VARIATION), 1),
            satellites=random.randint(*config.SAT_RANGE),
            speed=round(config.BASE_SPEED_MPS * (self.speed / 100), 1),
            progress=self.progress,
            arrived=arrived,
            safety_violation=self.safety_violation,
            remaining_distance=max(0, self.total_distance - self.distance_traveled) * 111000
        )
        
        self.history.insert(0, heartbeat)
        if len(self.history) > 100:
            self.history.pop()
        
        self.flight_log.append(heartbeat)
        if len(self.flight_log) > 1000:
            self.flight_log.pop(0)
        
        return heartbeat
    
    def export_flight_data(self) -> pd.DataFrame:
        if not self.flight_log:
            return pd.DataFrame()
        
        data = [{
            'timestamp': h.timestamp,
            'flight_time': h.flight_time,
            'lat': h.lat,
            'lng': h.lng,
            'altitude': h.altitude,
            'voltage': h.voltage,
            'satellites': h.satellites,
            'speed': h.speed,
            'progress': h.progress,
            'arrived': h.arrived,
            'safety_violation': h.safety_violation,
            'remaining_distance': h.remaining_distance
        } for h in self.flight_log]
        
        return pd.DataFrame(data)

# ==================== 地图创建 ====================
def create_planning_map(center_gcj: List[float], points_gcj: Dict, obstacles_gcj: List[Dict], flight_history: Optional[List] = None,
                       planned_path: Optional[List] = None, map_type: str = "satellite", straight_blocked: bool = True, 
                       flight_altitude: float = 50, drone_pos: Optional[List] = None, direction: str = "最佳航线", safety_radius: float = 5) -> folium.Map:
    if map_type == "satellite":
        tiles = config.GAODE_SATELLITE_URL
        attr = "高德卫星地图"
    else:
        tiles = config.GAODE_VECTOR_URL
        attr = "高德矢量地图"
    
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr=attr)
    
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4},
                      'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False},
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(m)
    
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        if "向左" in direction:
            line_color = "purple"
        elif "向右" in direction:
            line_color = "orange"
        else:
            line_color = "green"
        folium.PolyLine(path_locations, color=line_color, weight=5, opacity=0.9, popup=f"✈️ {direction}").add_to(m)
        
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=5, color=line_color, fill=True, fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    
    if points_gcj.get('A') and points_gcj.get('B'):
        if not straight_blocked:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="blue", weight=2, opacity=0.5, dash_array='5, 5', popup="直线航线").add_to(m)
        else:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="gray", weight=2, opacity=0.4, dash_array='5, 5', popup="⚠️ 直线被阻挡").add_to(m)
    
    if drone_pos:
        folium.Circle(radius=safety_radius, location=[drone_pos[1], drone_pos[0]], color="blue", weight=2, fill=True, fill_color="blue", fill_opacity=0.2, popup=f"🛡️ 安全半径: {safety_radius}米").add_to(m)
    
    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)
    
    return m

# ==================== 辅助UI函数 ====================
def init_session_state():
    defaults = {
        'points_gcj': {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()},
        'obstacles_gcj': load_obstacles(),
        'heartbeat_sim': HeartbeatSimulator(config.DEFAULT_A_GCJ.copy()),
        'last_hb_time': time.time(),
        'simulation_running': False,
        'flight_history': [],
        'planned_path': None,
        'last_flight_altitude': 50,
        'pending_obstacle': None,
        'current_direction': "最佳航线",
        'safety_radius': config.DEFAULT_SAFETY_RADIUS_METERS,
        'auto_backup': True,
        'show_rename_dialog': False
    }
    
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value
    
    for obs in st.session_state.obstacles_gcj:
        if 'height' not in obs:
            obs['height'] = 30
        if 'selected' not in obs:
            obs['selected'] = False

def check_straight_blocked(points_gcj: Dict, obstacles_gcj: List[Dict], flight_altitude: float) -> Tuple[bool, int]:
    blocked = False
    high_count = 0
    
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            high_count += 1
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(points_gcj['A'], points_gcj['B'], coords):
                blocked = True
    
    return blocked, high_count

def render_sidebar() -> Tuple[str, str, int, float, bool]:
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, value=50, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("✈️ 无人机飞行高度")
    flight_alt = st.sidebar.slider("飞行高度 (m)", min_value=10, max_value=200, value=50, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全半径设置")
    safety_radius = st.sidebar.slider("安全半径 (米)", min_value=1, max_value=20, value=st.session_state.safety_radius, step=1)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("💾 自动保存")
    auto_save = st.sidebar.checkbox("自动保存障碍物", value=st.session_state.auto_backup)
    
    return page, map_type, drone_speed, flight_alt, auto_save

# ==================== 页面渲染函数 ====================
def render_planning_page(map_type: str, drone_speed: int, flight_alt: float, auto_save: bool):
    st.header("🗺️ 航线规划 - 智能避障")
    
    straight_blocked, high_obstacles = check_straight_blocked(st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_alt)
    
    if straight_blocked:
        st.warning(f"⚠️ 有 {high_obstacles} 个障碍物高于飞行高度({flight_alt}m)，需要绕行")
    else:
        st.success("✅ 直线航线畅通无阻（所有障碍物高度 ≤ 飞行高度）")
    
    st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 输入高度并保存")
    
    col1, col2 = st.columns([1, 1.5])
    
    with col1:
        render_planning_controls(flight_alt, drone_speed, auto_save)
    
    with col2:
        render_planning_map_view(map_type, flight_alt, straight_blocked)

def render_planning_controls(flight_alt: float, drone_speed: int, auto_save: bool):
    st.subheader("🎮 控制面板")
    
    with st.expander("📍 起点/终点设置", expanded=True):
        render_point_settings()
    
    with st.expander("🤖 路径规划策略", expanded=True):
        render_path_strategy(flight_alt)
    
    with st.expander("✈️ 飞行控制", expanded=True):
        render_flight_controls(flight_alt, drone_speed)
    
    st.markdown("### 📍 当前坐标")
    st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
    st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
    
    a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
    dist = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2) * 111000
    st.caption(f"📏 直线距离: {dist:.0f} 米")

def render_point_settings():
    st.markdown("#### 🟢 起点 A")
    col_a1, col_a2 = st.columns(2)
    with col_a1:
        a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat", step=0.000001)
    with col_a2:
        a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng", step=0.000001)
    
    if st.button("📍 设置 A 点", use_container_width=True):
        st.session_state.points_gcj['A'] = [a_lng, a_lat]
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
            st.session_state.current_direction, st.session_state.safety_radius
        )
        st.rerun()
    
    st.markdown("#### 🔴 终点 B")
    col_b1, col_b2 = st.columns(2)
    with col_b1:
        b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat", step=0.000001)
    with col_b2:
        b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng", step=0.000001)
    
    if st.button("📍 设置 B 点", use_container_width=True):
        st.session_state.points_gcj['B'] = [b_lng, b_lat]
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
            st.session_state.current_direction, st.session_state.safety_radius
        )
        st.rerun()

def render_path_strategy(flight_alt: float):
    st.markdown("**选择绕行方向：**")
    col_dir1, col_dir2, col_dir3 = st.columns(3)
    
    with col_dir1:
        if st.button("🔄 最佳航线", use_container_width=True, type="primary" if st.session_state.current_direction == "最佳航线" else "secondary"):
            st.session_state.current_direction = "最佳航线"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "最佳航线",
                st.session_state.safety_radius
            )
            st.success("已切换到最佳航线模式")
            st.rerun()
    
    with col_dir2:
        if st.button("⬅️ 向左绕行", use_container_width=True, type="primary" if st.session_state.current_direction == "向左绕行" else "secondary"):
            st.session_state.current_direction = "向左绕行"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "向左绕行",
                st.session_state.safety_radius
            )
            st.success("已切换到向左绕行模式")
            st.rerun()
    
    with col_dir3:
        if st.button("➡️ 向右绕行", use_container_width=True, type="primary" if st.session_state.current_direction == "向右绕行" else "secondary"):
            st.session_state.current_direction = "向右绕行"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "向右绕行",
                st.session_state.safety_radius
            )
            st.success("已切换到向右绕行模式")
            st.rerun()
    
    st.info(f"📌 当前绕行策略: **{st.session_state.current_direction}**")
    
    if st.button("🔄 重新规划路径", use_container_width=True):
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt,
            st.session_state.current_direction, st.session_state.safety_radius
        )
        if st.session_state.planned_path:
            waypoint_count = len(st.session_state.planned_path) - 2
            st.success(f"已按照「{st.session_state.current_direction}」规划路径，{waypoint_count}个绕行点")
        st.rerun()

def render_flight_controls(flight_alt: float, drone_speed: int):
    col_met1, col_met2, col_met3 = st.columns(3)
    with col_met1:
        st.metric("当前飞行高度", f"{flight_alt} m")
    with col_met2:
        st.metric("速度系数", f"{drone_speed}%")
    with col_met3:
        st.metric("🛡️ 安全半径", f"{st.session_state.safety_radius} 米")
    
    if st.session_state.planned_path:
        waypoint_count = len(st.session_state.planned_path) - 2
        st.metric("🎯 绕行点数量", waypoint_count)
        
        total_dist = calculate_path_length(st.session_state.planned_path) * 111000
        st.caption(f"📏 规划路径总长: {total_dist:.0f} 米")
    
    col_btn1, col_btn2 = st.columns(2)
    with col_btn1:
        if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
            if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, st.session_state.safety_radius)
                st.session_state.simulation_running = True
                st.session_state.flight_history = []
                waypoint_count = len(path) - 2
                st.success(f"🚁 飞行已开始！{'路径中有' + str(waypoint_count) + '个绕行点' if waypoint_count > 0 else '直线飞行'}")
                st.rerun()
            else:
                st.error("请先设置起点和终点")
    
    with col_btn2:
        if st.button("⏹️ 停止飞行", use_container_width=True):
            st.session_state.simulation_running = False
            st.session_state.heartbeat_sim.simulating = False
            st.info("飞行已停止")

def render_planning_map_view(map_type: str, flight_alt: float, straight_blocked: bool):
    st.subheader("🗺️ 规划地图")
    if straight_blocked:
        st.caption(f"当前避障策略: {st.session_state.current_direction}")
    st.caption("🟢 绿色=最佳航线 | 🟣 紫色=向左绕行 | 🟠 橙色=向右绕行 | 🔵 蓝色圆圈=安全半径")
    
    flight_trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:20]]
    center = st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ
    
    if st.session_state.planned_path is None:
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt,
            st.session_state.current_direction, st.session_state.safety_radius
        )
    
    drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
    
    m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj,
                           flight_trail, st.session_state.planned_path, map_type,
                           straight_blocked, flight_alt, drone_pos,
                           st.session_state.current_direction, st.session_state.safety_radius)
    
    output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
    
    handle_drawing_output(output)

def handle_drawing_output(output: Any):
    if output and output.get("last_active_drawing"):
        last = output["last_active_drawing"]
        if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
            coords = last["geometry"].get("coordinates", [])
            if coords and len(coords) > 0:
                poly = [[p[0], p[1]] for p in coords[0]]
                if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                    if validate_polygon(poly):
                        st.session_state.pending_obstacle = poly
                        st.rerun()
    
    if st.session_state.pending_obstacle is not None:
        render_obstacle_dialog()

def render_obstacle_dialog():
    st.markdown("---")
    st.subheader("📝 添加新障碍物")
    st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle)} 个顶点")
    
    col_name1, col_name2 = st.columns(2)
    with col_name1:
        new_name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj) + 1}")
    with col_name2:
        new_height = st.number_input("障碍物高度 (米)", min_value=1, max_value=200, value=30, step=5, key="height_input")
    
    col_ok, col_cancel = st.columns(2)
    with col_ok:
        if st.button("✅ 确认添加", use_container_width=True, type="primary"):
            new_obstacle = {
                "name": new_name,
                "polygon": st.session_state.pending_obstacle,
                "height": new_height,
                "selected": False,
                "id": f"obs_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{len(st.session_state.obstacles_gcj)}",
                "created_time": datetime.now().isoformat()
            }
            st.session_state.obstacles_gcj.append(new_obstacle)
            if st.session_state.auto_backup:
                save_obstacles(st.session_state.obstacles_gcj)
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
                st.session_state.current_direction, st.session_state.safety_radius
            )
            st.session_state.pending_obstacle = None
            st.success(f"✅ 已添加 {new_name}，高度 {new_height} 米")
            st.rerun()
    with col_cancel:
        if st.button("❌ 取消", use_container_width=True):
            st.session_state.pending_obstacle = None
            st.rerun()

# ==================== 飞行监控页面（优化版） ====================
def render_flight_monitoring_page(map_type: str, flight_alt: float, drone_speed: int):
    """渲染飞行监控页面"""
    st.header("📡 飞行监控 - 实时心跳包")
    
    # 更新飞行模拟
    update_flight_simulation()
    
    if st.session_state.heartbeat_sim.history:
        latest = st.session_state.heartbeat_sim.history[0]
        
        # 飞行进度条
        st.markdown("### ✈️ 飞行进度")
        progress_percent = int(latest.progress * 100)
        st.progress(latest.progress, text=f"飞行进度：{progress_percent}%")
        
        # 主要指标卡片
        st.markdown("### 📊 实时数据")
        col1, col2, col3, col4, col5 = st.columns(5)
        
        with col1:
            st.metric(
                label="⏰ 飞行时间",
                value=f"{latest.flight_time:.1f}s",
                delta=None
            )
        
        with col2:
            st.metric(
                label="📍 当前位置",
                value=f"{latest.lat:.6f}, {latest.lng:.6f}",
                delta=None
            )
        
        with col3:
            st.metric(
                label="📏 飞行高度",
                value=f"{latest.altitude} m",
                delta=None
            )
        
        with col4:
            st.metric(
                label="💨 当前速度",
                value=f"{latest.speed} m/s",
                delta=f"{drone_speed}%"
            )
        
        with col5:
            remaining_dist = latest.remaining_distance
            st.metric(
                label="📏 剩余距离",
                value=f"{remaining_dist:.0f} m",
                delta=None
            )
        
        # 设备状态
        st.markdown("### 🔧 设备状态")
        col6, col7, col8, col9, col10 = st.columns(5)
        
        with col6:
            st.metric(
                label="🔋 电池电压",
                value=f"{latest.voltage} V",
                delta=None
            )
        
        with col7:
            st.metric(
                label="🛰️ 卫星数量",
                value=f"{latest.satellites} 颗",
                delta=None
            )
        
        with col8:
            st.metric(
                label="🎯 任务进度",
                value=f"{progress_percent}%",
                delta=None
            )
        
        with col9:
            st.metric(
                label="🛡️ 安全半径",
                value=f"{st.session_state.safety_radius} m",
                delta=None
            )
        
        with col10:
            if latest.arrived:
                status = "✅ 已完成"
            elif st.session_state.simulation_running:
                status = "✈️ 飞行中"
            else:
                status = "⏸️ 已停止"
            st.metric(
                label="📌 飞行状态",
                value=status,
                delta=None
            )
        
        # 安全警告
        if latest.safety_violation:
            st.error("⚠️ 警告：无人机进入安全半径危险区域！请立即检查！")
        
        if latest.arrived:
            st.success("🎉 无人机已到达目的地！飞行任务完成！")
        
        st.markdown("---")
        
        # 实时地图
        st.markdown("### 🗺️ 实时位置追踪")
        display_monitor_map(map_type, latest, flight_alt)
        
        st.markdown("---")
        
        # 数据图表
        st.markdown("### 📈 实时数据图表")
        col_ch1, col_ch2 = st.columns(2)
        
        with col_ch1:
            st.subheader("📊 飞行高度变化")
            if len(st.session_state.heartbeat_sim.history) > 1:
                alt_data = []
                for i, h in enumerate(st.session_state.heartbeat_sim.history[:30]):
                    alt_data.append({"时间": i, "高度(m)": h.altitude})
                alt_df = pd.DataFrame(alt_data)
                st.line_chart(alt_df, x="时间", y="高度(m)")
        
        with col_ch2:
            st.subheader("⚡ 速度变化趋势")
            if len(st.session_state.heartbeat_sim.history) > 1:
                speed_data = []
                for i, h in enumerate(st.session_state.heartbeat_sim.history[:30]):
                    speed_data.append({"时间": i, "速度(m/s)": h.speed})
                speed_df = pd.DataFrame(speed_data)
                st.line_chart(speed_df, x="时间", y="速度(m/s)")
        
        # 第三行图表
        col_ch3, col_ch4 = st.columns(2)
        
        with col_ch3:
            st.subheader("🔋 电压变化")
            if len(st.session_state.heartbeat_sim.history) > 1:
                voltage_data = []
                for i, h in enumerate(st.session_state.heartbeat_sim.history[:30]):
                    voltage_data.append({"时间": i, "电压(V)": h.voltage})
                voltage_df = pd.DataFrame(voltage_data)
                st.line_chart(voltage_df, x="时间", y="电压(V)")
        
        with col_ch4:
            st.subheader("🛰️ 卫星数量")
            if len(st.session_state.heartbeat_sim.history) > 1:
                sat_data = []
                for i, h in enumerate(st.session_state.heartbeat_sim.history[:30]):
                    sat_data.append({"时间": i, "卫星数": h.satellites})
                sat_df = pd.DataFrame(sat_data)
                st.line_chart(sat_df, x="时间", y="卫星数")
        
        st.markdown("---")
        
        # 飞行日志
        st.markdown("### 📋 飞行日志记录")
        display_flight_history()
        
        # 导出按钮
        st.markdown("---")
        col_export1, col_export2, col_export3 = st.columns(3)
        with col_export1:
            if st.button("📊 导出完整飞行数据", use_container_width=True, type="primary"):
                df = st.session_state.heartbeat_sim.export_flight_data()
                if not df.empty:
                    csv = df.to_csv(index=False)
                    st.download_button(
                        label="📥 下载CSV文件",
                        data=csv,
                        file_name=f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                        mime="text/csv",
                        use_container_width=True
                    )
        
        with col_export2:
            if st.button("🔄 刷新数据", use_container_width=True):
                st.rerun()
        
        with col_export3:
            if st.button("⏹️ 停止飞行", use_container_width=True):
                st.session_state.simulation_running = False
                st.session_state.heartbeat_sim.simulating = False
                st.success("飞行已停止")
                st.rerun()
                
    else:
        st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        
        # 显示提示卡片
        st.markdown("---")
        col_tip1, col_tip2, col_tip3 = st.columns(3)
        with col_tip1:
            st.info("💡 提示1：先在航线规划页面设置起点和终点")
        with col_tip2:
            st.info("💡 提示2：设置飞行高度和速度系数")
        with col_tip3:
            st.info("💡 提示3：点击「开始飞行」按钮启动模拟")


def display_monitor_map(map_type: str, latest, flight_alt: float):
    """显示监控地图"""
    tiles = config.GAODE_SATELLITE_URL if map_type == "satellite" else config.GAODE_VECTOR_URL
    monitor_map = folium.Map(location=[latest.lat, latest.lng], zoom_start=18, 
                             tiles=tiles, attr="高德地图")
    
    # 绘制障碍物
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_alt else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, 
                          weight=2, fill=True, fill_opacity=0.3,
                          popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(monitor_map)
    
    # 绘制规划路径
    if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
        if "向左" in st.session_state.current_direction:
            line_color = "purple"
        elif "向右" in st.session_state.current_direction:
            line_color = "orange"
        else:
            line_color = "green"
        folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], 
                       color=line_color, weight=3, opacity=0.7,
                       popup=f"规划航线 - {st.session_state.current_direction}").add_to(monitor_map)
    
    # 绘制安全半径圈
    folium.Circle(
        radius=st.session_state.safety_radius, 
        location=[latest.lat, latest.lng],
        color="blue", 
        weight=2, 
        fill=True,
        fill_color="blue", 
        fill_opacity=0.2,
        popup=f"🛡️ 安全半径: {st.session_state.safety_radius}米"
    ).add_to(monitor_map)
    
    # 绘制历史轨迹
    trail = [[hb.lat, hb.lng] for hb in st.session_state.heartbeat_sim.history[:50] 
             if hb.lat and hb.lng]
    if len(trail) > 1:
        folium.PolyLine(trail, color="orange", weight=2, opacity=0.6,
                       popup="历史飞行轨迹").add_to(monitor_map)
    
    # 绘制当前位置
    folium.Marker(
        [latest.lat, latest.lng], 
        popup=f"当前位置\n高度: {latest.altitude}m\n速度: {latest.speed}m/s", 
        icon=folium.Icon(color='red', icon='plane', prefix='fa')
    ).add_to(monitor_map)
    
    # 绘制起点和终点
    if st.session_state.points_gcj['A']:
        folium.Marker(
            [st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
            popup="起点 A", 
            icon=folium.Icon(color='green', icon='play', prefix='fa')
        ).add_to(monitor_map)
    
    if st.session_state.points_gcj['B']:
        folium.Marker(
            [st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
            popup="终点 B", 
            icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
        ).add_to(monitor_map)
    
    # 添加航点标记
    if st.session_state.planned_path and len(st.session_state.planned_path) > 2:
        for i, point in enumerate(st.session_state.planned_path[1:-1]):
            folium.CircleMarker(
                [point[1], point[0]], 
                radius=4, 
                color="yellow", 
                fill=True,
                fill_color="yellow",
                fill_opacity=0.8,
                popup=f"航点 {i+1}"
            ).add_to(monitor_map)
    
    folium_static(monitor_map, width=900, height=500)


def display_flight_history():
    """显示飞行历史记录"""
    history_df = st.session_state.heartbeat_sim.export_flight_data()
    
    if not history_df.empty:
        # 显示最近的10条记录
        display_cols = ['timestamp', 'flight_time', 'lat', 'lng', 'altitude', 'speed', 'voltage', 'satellites', 'remaining_distance']
        display_cols = [col for col in display_cols if col in history_df.columns]
        
        recent_df = history_df[display_cols].head(10)
        
        # 重命名列名为中文
        column_names = {
            'timestamp': '时间',
            'flight_time': '飞行时间(s)',
            'lat': '纬度',
            'lng': '经度',
            'altitude': '高度(m)',
            'speed': '速度(m/s)',
            'voltage': '电压(V)',
            'satellites': '卫星数',
            'remaining_distance': '剩余距离(m)'
        }
        recent_df = recent_df.rename(columns=column_names)
        
        st.dataframe(recent_df, use_container_width=True)
        
        # 显示统计信息
        st.markdown("### 📊 飞行统计")
        col_stat1, col_stat2, col_stat3, col_stat4 = st.columns(4)
        
        with col_stat1:
            max_speed = history_df['speed'].max() if 'speed' in history_df.columns else 0
            st.metric("🏁 最高速度", f"{max_speed:.1f} m/s")
        
        with col_stat2:
            avg_speed = history_df['speed'].mean() if 'speed' in history_df.columns else 0
            st.metric("📈 平均速度", f"{avg_speed:.1f} m/s")
        
        with col_stat3:
            max_alt = history_df['altitude'].max() if 'altitude' in history_df.columns else 0
            st.metric("⛰️ 最高高度", f"{max_alt:.0f} m")
        
        with col_stat4:
            total_time = history_df['flight_time'].max() if 'flight_time' in history_df.columns else 0
            st.metric("⏱️ 总飞行时间", f"{total_time:.1f} s")
    else:
        st.info("暂无飞行数据")


def update_flight_simulation():
    """更新飞行模拟"""
    current_time = time.time()
    if st.session_state.simulation_running:
        if current_time - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
            try:
                new_hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
                if new_hb:
                    st.session_state.last_hb_time = current_time
                    st.session_state.flight_history.append([new_hb.lng, new_hb.lat])
                    if len(st.session_state.flight_history) > 200:
                        st.session_state.flight_history.pop(0)
                    if not st.session_state.heartbeat_sim.simulating:
                        st.session_state.simulation_running = False
                        st.success("🏁 无人机已安全到达目的地！")
                    st.rerun()
            except Exception as e:
                st.error(f"更新心跳时出错: {e}")
    else:
        st.session_state.last_hb_time = current_time

# ==================== 障碍物管理页面 ====================
def render_obstacle_management_page(flight_alt: float):
    """渲染障碍物管理页面"""
    st.header("🚧 障碍物管理")
    
    # 状态信息
    col_status1, col_status2, col_status3, col_status4 = st.columns(4)
    with col_status1:
        st.info(f"📊 当前共 {len(st.session_state.obstacles_gcj)} 个障碍物")
    with col_status2:
        st.info(f"🛡️ 安全半径: {st.session_state.safety_radius}米")
    with col_status3:
        if os.path.exists(config.CONFIG_FILE):
            try:
                with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    save_time = data.get('save_time', '未知')
                    st.info(f"💾 最后保存: {save_time}")
            except:
                st.info("💾 未保存")
        else:
            st.info("💾 未保存")
    with col_status4:
        backup_count = len([f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')])
        st.info(f"📦 备份数量: {backup_count}")
    
    st.markdown("---")
    
    # 数据操作按钮组
    col_data1, col_data2, col_data3, col_data4, col_data5 = st.columns(5)
    
    with col_data1:
        if st.button("💾 保存配置", use_container_width=True, type="primary"):
            if save_obstacles(st.session_state.obstacles_gcj):
                st.success(f"✅ 已保存 {len(st.session_state.obstacles_gcj)} 个障碍物")
                st.balloons()
                time.sleep(0.5)
                st.rerun()
    
    with col_data2:
        if st.button("📂 加载配置", use_container_width=True):
            loaded = load_obstacles()
            if loaded:
                st.session_state.obstacles_gcj = loaded
                update_path_after_obstacle_change(flight_alt)
                st.success(f"✅ 已加载 {len(loaded)} 个障碍物")
                st.rerun()
            else:
                st.warning("⚠️ 未找到配置文件")
    
    with col_data3:
        if st.session_state.obstacles_gcj:
            config_data = {
                'obstacles': st.session_state.obstacles_gcj,
                'count': len(st.session_state.obstacles_gcj),
                'export_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'version': 'v13.1'
            }
            json_str = json.dumps(config_data, ensure_ascii=False, indent=2)
            st.download_button(
                label="📥 导出配置",
                data=json_str,
                file_name=f"obstacles_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
                mime="application/json",
                use_container_width=True
            )
        else:
            st.button("📥 导出配置", use_container_width=True, disabled=True)
    
    with col_data4:
        latest_backup = get_latest_backup()
        if latest_backup:
            if st.button("🔄 恢复备份", use_container_width=True):
                if restore_from_backup(latest_backup):
                    st.session_state.obstacles_gcj = load_obstacles()
                    update_path_after_obstacle_change(flight_alt)
                    st.success("✅ 已从备份恢复")
                    st.rerun()
                else:
                    st.error("❌ 恢复失败")
        else:
            st.button("🔄 恢复备份", use_container_width=True, disabled=True)
    
    with col_data5:
        if st.button("🗑️ 清除全部", use_container_width=True):
            if st.session_state.auto_backup:
                backup_config()
            st.session_state.obstacles_gcj = []
            save_obstacles([])
            update_path_after_obstacle_change(flight_alt)
            st.success("✅ 已清除所有障碍物")
            st.rerun()
    
    st.markdown("---")
    
    # 统计信息
    col_stats1, col_stats2, col_stats3, col_stats4 = st.columns(4)
    with col_stats1:
        high_obs = sum(1 for obs in st.session_state.obstacles_gcj if obs.get('height', 30) > flight_alt)
        st.metric("🔴 需避让障碍物", high_obs)
    with col_stats2:
        safe_obs = len(st.session_state.obstacles_gcj) - high_obs
        st.metric("🟠 安全障碍物", safe_obs)
    with col_stats3:
        total_vertices = sum(len(obs.get('polygon', [])) for obs in st.session_state.obstacles_gcj)
        st.metric("📍 总顶点数", total_vertices)
    with col_stats4:
        avg_height = sum(obs.get('height', 30) for obs in st.session_state.obstacles_gcj) / max(1, len(st.session_state.obstacles_gcj))
        st.metric("📏 平均高度", f"{avg_height:.1f}m")
    
    st.markdown("---")
    
    # 批量操作
    st.subheader("🎯 批量操作")
    
    for obs in st.session_state.obstacles_gcj:
        if 'selected' not in obs:
            obs['selected'] = False
    
    col_batch1, col_batch2, col_batch3, col_batch4 = st.columns(4)
    
    with col_batch1:
        select_all = st.checkbox("☑️ 全选所有障碍物")
        if select_all:
            for obs in st.session_state.obstacles_gcj:
                obs['selected'] = True
    
    with col_batch2:
        if st.button("🗑️ 批量删除", use_container_width=True, type="primary"):
            selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
            if selected_indices:
                for i in reversed(selected_indices):
                    st.session_state.obstacles_gcj.pop(i)
                if st.session_state.auto_backup:
                    save_obstacles(st.session_state.obstacles_gcj)
                update_path_after_obstacle_change(flight_alt)
                st.success(f"✅ 已删除 {len(selected_indices)} 个障碍物")
                st.rerun()
            else:
                st.warning("⚠️ 请先选择要删除的障碍物")
    
    with col_batch3:
        batch_height = st.number_input("批量高度(m)", min_value=1, max_value=200, value=30, step=5, key="batch_height")
        if st.button("📏 批量设置高度", use_container_width=True):
            selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
            if selected_indices:
                for i in selected_indices:
                    st.session_state.obstacles_gcj[i]['height'] = batch_height
                if st.session_state.auto_backup:
                    save_obstacles(st.session_state.obstacles_gcj)
                update_path_after_obstacle_change(flight_alt)
                st.success(f"✅ 已为 {len(selected_indices)} 个障碍物设置高度为 {batch_height}m")
                st.rerun()
            else:
                st.warning("⚠️ 请先选择要修改的障碍物")
    
    with col_batch4:
        if st.button("🏷️ 批量重命名", use_container_width=True):
            selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
            if selected_indices:
                st.session_state.show_rename_dialog = True
            else:
                st.warning("⚠️ 请先选择要重命名的障碍物")
    
    if st.session_state.get('show_rename_dialog', False):
        with st.expander("🏷️ 批量重命名", expanded=True):
            col_n1, col_n2 = st.columns(2)
            with col_n1:
                name_prefix = st.text_input("名称前缀", value="建筑物")
                start_number = st.number_input("起始编号", min_value=1, value=1, step=1)
            with col_n2:
                name_suffix = st.text_input("名称后缀", value="")
            
            col_confirm, col_cancel_r = st.columns(2)
            with col_confirm:
                if st.button("确认重命名", use_container_width=True, type="primary"):
                    selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
                    for idx, i in enumerate(selected_indices):
                        new_name = f"{name_prefix}{start_number + idx}{name_suffix}"
                        st.session_state.obstacles_gcj[i]['name'] = new_name
                    if st.session_state.auto_backup:
                        save_obstacles(st.session_state.obstacles_gcj)
                    st.session_state.show_rename_dialog = False
                    st.success(f"✅ 已重命名 {len(selected_indices)} 个障碍物")
                    st.rerun()
            with col_cancel_r:
                if st.button("取消"):
                    st.session_state.show_rename_dialog = False
                    st.rerun()
    
    st.markdown("---")
    
    # 标签页
    tab_list, tab_map = st.tabs(["📋 列表视图", "🗺️ 地图视图"])
    
    with tab_list:
        render_obstacle_list_view(flight_alt)
    
    with tab_map:
        render_obstacle_map_view(flight_alt)


def render_obstacle_list_view(flight_alt: float):
    st.subheader("📝 障碍物列表")
    st.caption("💡 提示：勾选复选框后可使用批量操作功能")
    
    if st.session_state.obstacles_gcj:
        items_per_row = 2
        rows = (len(st.session_state.obstacles_gcj) + items_per_row - 1) // items_per_row
        
        for row in range(rows):
            cols = st.columns(items_per_row)
            for col_idx in range(items_per_row):
                idx = row * items_per_row + col_idx
                if idx < len(st.session_state.obstacles_gcj):
                    render_obstacle_card(idx, flight_alt, cols[col_idx])
    else:
        st.info("📭 暂无任何障碍物，可以在「地图视图」中绘制添加")


def render_obstacle_card(idx: int, flight_alt: float, container):
    obs = st.session_state.obstacles_gcj[idx]
    with container:
        with st.container(border=True):
            height = obs.get('height', 30)
            color = "🔴" if height > flight_alt else "🟠"
            name = obs.get('name', f'障碍物{idx+1}')
            
            col_check, col_name = st.columns([1, 5])
            with col_check:
                checked = st.checkbox("", key=f"select_card_{idx}", value=obs.get('selected', False))
                st.session_state.obstacles_gcj[idx]['selected'] = checked
            with col_name:
                st.markdown(f"**{color} {name}**")
            
            col_h1, col_h2 = st.columns(2)
            with col_h1:
                st.caption(f"📏 高度: {height}m")
            with col_h2:
                st.caption(f"📍 顶点: {len(obs.get('polygon', []))}个")
            
            new_h = st.number_input("调整高度", value=height, min_value=1, max_value=200, step=5, key=f"quick_edit_{idx}", label_visibility="collapsed")
            if new_h != height:
                obs['height'] = new_h
                if st.session_state.auto_backup:
                    save_obstacles(st.session_state.obstacles_gcj)
                update_path_after_obstacle_change(flight_alt)
                st.rerun()
            
            if st.button("🗑️ 删除", key=f"delete_card_{idx}", use_container_width=True):
                st.session_state.obstacles_gcj.pop(idx)
                if st.session_state.auto_backup:
                    save_obstacles(st.session_state.obstacles_gcj)
                update_path_after_obstacle_change(flight_alt)
                st.rerun()


def render_obstacle_map_view(flight_alt: float):
    st.subheader("🗺️ 地图视图")
    st.caption("✏️ 使用左上角绘制工具绘制新障碍物 | 🖱️ 点击障碍物查看详细信息 | 🎨 红色=需避让，橙色=安全")
    
    map_view_type = st.radio("地图类型", ["卫星影像", "矢量街道"], index=0, horizontal=True)
    map_type_view = "satellite" if map_view_type == "卫星影像" else "vector"
    
    tiles = config.GAODE_SATELLITE_URL if map_type_view == "satellite" else config.GAODE_VECTOR_URL
    obs_map = folium.Map(location=[config.SCHOOL_CENTER_GCJ[1], config.SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=tiles, attr="高德地图")
    
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4},
                      'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False},
        edit_options={'edit': True, 'remove': True}
    )
    obs_map.add_child(draw)
    
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        color = "red" if height > flight_alt else "orange"
        if coords and len(coords) >= 3:
            popup_text = f"""
            <div style="font-family: sans-serif;">
                <b>🏢 {obs.get('name')}</b><br>
                高度: {height} 米<br>
                ID: {obs.get('id', 'N/A')}<br>
            </div>
            """
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.5, popup=folium.Popup(popup_text, max_width=300)).add_to(obs_map)
    
    folium.Marker([config.DEFAULT_A_GCJ[1], config.DEFAULT_A_GCJ[0]], popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(obs_map)
    folium.Marker([config.DEFAULT_B_GCJ[1], config.DEFAULT_B_GCJ[0]], popup="终点", icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')).add_to(obs_map)
    
    map_output = st_folium(obs_map, width=800, height=550, key="obstacle_map_view", returned_objects=["last_active_drawing"])
    
    if map_output and map_output.get("last_active_drawing"):
        last = map_output["last_active_drawing"]
        if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
            coords = last["geometry"].get("coordinates", [])
            if coords and len(coords) > 0:
                poly = [[p[0], p[1]] for p in coords[0]]
                if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                    if validate_polygon(poly):
                        st.session_state.pending_obstacle = poly
                        st.rerun()
    
    if st.session_state.pending_obstacle is not None:
        render_obstacle_dialog()


def update_path_after_obstacle_change(flight_alt: float):
    st.session_state.planned_path = create_avoidance_path(
        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, flight_alt,
        st.session_state.current_direction, st.session_state.safety_radius
    )


# ==================== 主程序 ====================
def main():
    st.set_page_config(page_title="南京科技职业学院 - 无人机地面站系统", layout="wide")
    
    init_session_state()
    
    st.title("🏫 南京科技职业学院 - 无人机地面站系统")
    st.markdown("---")
    
    page, map_type, drone_speed, flight_alt, auto_save = render_sidebar()
    st.session_state.auto_backup = auto_save
    
    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path is not None:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt,
                st.session_state.current_direction, st.session_state.safety_radius
            )
            st.rerun()
    
    if page == "🗺️ 航线规划":
        render_planning_page(map_type, drone_speed, flight_alt, auto_save)
    elif page == "📡 飞行监控":
        render_flight_monitoring_page(map_type, flight_alt, drone_speed)
    elif page == "🚧 障碍物管理":
        render_obstacle_management_page(flight_alt)


if __name__ == "__main__":
    main()
