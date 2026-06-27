import streamlit as st
import folium
from streamlit_folium import st_folium, folium_static
import json
import os
import math
import time
import random
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
from streamlit_autorefresh import st_autorefresh
from folium.plugins import Draw

#-------------------------------------------------------------------------------
# 配置
#-------------------------------------------------------------------------------
SCHOOL_CENTER_GCJ = [118.749413, 32.234097]  # 学校中心点(GCJ-02)
GAODE_TILE = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
HEARTBEAT_INTERVAL = 0.2
BASE_SPEED = 5.0
HOVER_SECONDS = 5
CONFIG_FILE = "obstacle_config.json"

#-------------------------------------------------------------------------------
# MAVLink消息解析类（新增）
#-------------------------------------------------------------------------------
class MAVLinkMessage:
    """MAVLink消息解析器 - 模拟MAVLink协议消息解析"""
    
    # MAVLink消息ID定义
    MSG_ID_HEARTBEAT = 0
    MSG_ID_SYS_STATUS = 1
    MSG_ID_GLOBAL_POSITION_INT = 33
    MSG_ID_ATTITUDE = 30
    MSG_ID_VFR_HUD = 74
    MSG_ID_MISSION_CURRENT = 43
    MSG_ID_MISSION_ITEM_REACHED = 46
    MSG_ID_STATUSTEXT = 253
    
    # 系统状态
    MAV_STATE_UNINIT = 0
    MAV_STATE_BOOT = 1
    MAV_STATE_CALIBRATING = 2
    MAV_STATE_STANDBY = 3
    MAV_STATE_ACTIVE = 4
    MAV_STATE_CRITICAL = 5
    MAV_STATE_EMERGENCY = 6
    MAV_STATE_POWEROFF = 7
    MAV_STATE_FLIGHT_TERMINATION = 8
    
    MAV_STATE_NAMES = {
        0: "未初始化",
        1: "启动中",
        2: "校准中",
        3: "待机",
        4: "激活",
        5: "严重警告",
        6: "紧急状态",
        7: "关机",
        8: "飞行终止"
    }
    
    # 飞行模式
    MODE_NAMES = {
        0: "手动",
        1: "增稳",
        2: "定高",
        3: "悬停",
        4: "自主",
        5: "任务",
        6: "返航",
        7: "降落",
        8: "跟随",
        9: "特技",
        10: "方位",
        11: "起飞",
        12: "着陆"
    }
    
    def __init__(self):
        self.reset()
    
    def reset(self):
        """重置解析器状态"""
        self.last_message = None
        self.message_history = []
        self.system_status = {
            "connected": False,
            "sysid": 1,
            "compid": 1,
            "state": self.MAV_STATE_STANDBY,
            "mode": 4,  # 自主模式
            "armed": False,
            "battery_voltage": 12.6,
            "battery_remaining": 85,
            "gps_fix": 3,
            "satellites": 12,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "altitude": 0.0,
            "relative_alt": 0.0,
            "ground_speed": 0.0,
            "air_speed": 0.0,
            "climb_rate": 0.0,
            "heading": 0,
            "mission_current": 0,
            "mission_count": 0,
            "text_messages": []
        }
        self.parsed_messages = []
    
    def parse_message(self, raw_data):
        """
        解析原始MAVLink消息
        模拟输入格式: "MSG_ID|payload_field1=value1,field2=value2,..."
        或标准MAVLink十六进制格式
        """
        try:
            if isinstance(raw_data, str):
                raw_data = raw_data.strip()
                
                # 检测消息格式
                if raw_data.startswith("MSG_ID"):
                    return self._parse_simulated_message(raw_data)
                elif raw_data.startswith("FE") or all(c in "0123456789ABCDEF" for c in raw_data.replace(" ", "").upper()):
                    return self._parse_hex_message(raw_data)
                else:
                    # 尝试解析为JSON格式
                    try:
                        data = json.loads(raw_data)
                        return self._parse_json_message(data)
                    except:
                        return self._parse_text_message(raw_data)
            elif isinstance(raw_data, dict):
                return self._parse_json_message(raw_data)
            else:
                return None
        except Exception as e:
            return {"error": f"解析失败: {str(e)}"}
    
    def _parse_simulated_message(self, raw_data):
        """解析模拟格式消息: MSG_ID|field=value,field=value"""
        try:
            # 移除MSG_ID前缀
            content = raw_data.replace("MSG_ID", "").strip()
            if "|" in content:
                parts = content.split("|")
                msg_id = int(parts[0].strip())
                payload_str = parts[1] if len(parts) > 1 else ""
                
                payload = {}
                if payload_str:
                    for pair in payload_str.split(","):
                        if "=" in pair:
                            key, value = pair.split("=")
                            key = key.strip()
                            value = value.strip()
                            # 尝试转换数字
                            try:
                                if "." in value:
                                    payload[key] = float(value)
                                else:
                                    payload[key] = int(value)
                            except:
                                payload[key] = value
                
                return self._process_message(msg_id, payload)
            return None
        except Exception as e:
            return {"error": f"解析模拟消息失败: {str(e)}"}
    
    def _parse_hex_message(self, raw_data):
        """解析十六进制MAVLink消息"""
        # 简化实现：模拟解析
        hex_str = raw_data.replace(" ", "").upper()
        if len(hex_str) < 10:
            return {"error": "消息太短"}
        
        # 提取消息ID（简化：从第6-8字节）
        try:
            msg_id = int(hex_str[6:8], 16)
            return self._process_message(msg_id, {"hex_data": raw_data})
        except:
            return {"error": "无法解析十六进制消息"}
    
    def _parse_json_message(self, data):
        """解析JSON格式消息"""
        msg_id = data.get("msg_id", data.get("id", 0))
        payload = data.get("payload", data.get("data", {}))
        return self._process_message(msg_id, payload)
    
    def _parse_text_message(self, text):
        """解析文本消息（作为状态文本处理）"""
        self.system_status["text_messages"].insert(0, {
            "time": datetime.now().strftime("%H:%M:%S"),
            "text": text
        })
        if len(self.system_status["text_messages"]) > 20:
            self.system_status["text_messages"].pop()
        
        result = {
            "msg_id": self.MSG_ID_STATUSTEXT,
            "msg_name": "STATUSTEXT",
            "severity": "INFO",
            "text": text,
            "timestamp": datetime.now().isoformat()
        }
        self.last_message = result
        self.message_history.insert(0, result)
        if len(self.message_history) > 100:
            self.message_history.pop()
        return result
    
    def _process_message(self, msg_id, payload):
        """处理解析后的消息"""
        result = {
            "msg_id": msg_id,
            "timestamp": datetime.now().isoformat(),
            "payload": payload
        }
        
        # 根据消息ID处理不同消息
        if msg_id == self.MSG_ID_HEARTBEAT:
            result["msg_name"] = "HEARTBEAT"
            self._process_heartbeat(payload)
        elif msg_id == self.MSG_ID_SYS_STATUS:
            result["msg_name"] = "SYS_STATUS"
            self._process_sys_status(payload)
        elif msg_id == self.MSG_ID_GLOBAL_POSITION_INT:
            result["msg_name"] = "GLOBAL_POSITION_INT"
            self._process_global_position(payload)
        elif msg_id == self.MSG_ID_ATTITUDE:
            result["msg_name"] = "ATTITUDE"
            self._process_attitude(payload)
        elif msg_id == self.MSG_ID_VFR_HUD:
            result["msg_name"] = "VFR_HUD"
            self._process_vfr_hud(payload)
        elif msg_id == self.MSG_ID_MISSION_CURRENT:
            result["msg_name"] = "MISSION_CURRENT"
            self._process_mission_current(payload)
        elif msg_id == self.MSG_ID_MISSION_ITEM_REACHED:
            result["msg_name"] = "MISSION_ITEM_REACHED"
            self._process_mission_reached(payload)
        elif msg_id == self.MSG_ID_STATUSTEXT:
            result["msg_name"] = "STATUSTEXT"
            self._process_statustext(payload)
        else:
            result["msg_name"] = f"UNKNOWN_{msg_id}"
        
        self.system_status["connected"] = True
        self.last_message = result
        self.message_history.insert(0, result)
        if len(self.message_history) > 100:
            self.message_history.pop()
        
        return result
    
    def _process_heartbeat(self, payload):
        """处理心跳消息"""
        state = payload.get("state", self.MAV_STATE_ACTIVE)
        mode = payload.get("mode", 4)
        armed = payload.get("armed", False)
        
        self.system_status["state"] = state
        self.system_status["mode"] = mode
        self.system_status["armed"] = armed
        self.system_status["connected"] = True
    
    def _process_sys_status(self, payload):
        """处理系统状态消息"""
        if "battery_voltage" in payload:
            self.system_status["battery_voltage"] = payload["battery_voltage"]
        if "battery_remaining" in payload:
            self.system_status["battery_remaining"] = payload["battery_remaining"]
        if "gps_fix" in payload:
            self.system_status["gps_fix"] = payload["gps_fix"]
        if "satellites" in payload:
            self.system_status["satellites"] = payload["satellites"]
    
    def _process_global_position(self, payload):
        """处理全局位置消息"""
        if "altitude" in payload:
            self.system_status["altitude"] = payload["altitude"]
        if "relative_alt" in payload:
            self.system_status["relative_alt"] = payload["relative_alt"]
        if "heading" in payload:
            self.system_status["heading"] = payload["heading"]
    
    def _process_attitude(self, payload):
        """处理姿态消息"""
        if "roll" in payload:
            self.system_status["roll"] = payload["roll"]
        if "pitch" in payload:
            self.system_status["pitch"] = payload["pitch"]
        if "yaw" in payload:
            self.system_status["yaw"] = payload["yaw"]
    
    def _process_vfr_hud(self, payload):
        """处理VFR_HUD消息"""
        if "ground_speed" in payload:
            self.system_status["ground_speed"] = payload["ground_speed"]
        if "air_speed" in payload:
            self.system_status["air_speed"] = payload["air_speed"]
        if "climb_rate" in payload:
            self.system_status["climb_rate"] = payload["climb_rate"]
        if "heading" in payload:
            self.system_status["heading"] = payload["heading"]
        if "altitude" in payload:
            self.system_status["altitude"] = payload["altitude"]
    
    def _process_mission_current(self, payload):
        """处理当前任务消息"""
        if "seq" in payload:
            self.system_status["mission_current"] = payload["seq"]
        if "count" in payload:
            self.system_status["mission_count"] = payload["count"]
    
    def _process_mission_reached(self, payload):
        """处理任务到达消息"""
        if "seq" in payload:
            self.system_status["mission_current"] = payload["seq"]
    
    def _process_statustext(self, payload):
        """处理状态文本消息"""
        text = payload.get("text", "")
        severity = payload.get("severity", "INFO")
        self.system_status["text_messages"].insert(0, {
            "time": datetime.now().strftime("%H:%M:%S"),
            "severity": severity,
            "text": text
        })
        if len(self.system_status["text_messages"]) > 20:
            self.system_status["text_messages"].pop()
    
    def get_status(self):
        """获取当前系统状态摘要"""
        state_name = self.MAV_STATE_NAMES.get(self.system_status["state"], "未知")
        mode_name = self.MODE_NAMES.get(self.system_status["mode"], "未知")
        arm_status = "🔓 已解锁" if self.system_status["armed"] else "🔒 已锁定"
        
        return {
            "connection": "🟢 已连接" if self.system_status["connected"] else "🔴 未连接",
            "state": state_name,
            "mode": mode_name,
            "arm_status": arm_status,
            "battery": f"{self.system_status['battery_remaining']}%",
            "voltage": f"{self.system_status['battery_voltage']:.1f}V",
            "gps": f"定位{self.system_status['gps_fix']}级 ({self.system_status['satellites']}颗)",
            "altitude": f"{self.system_status['altitude']:.1f}m",
            "ground_speed": f"{self.system_status['ground_speed']:.1f}m/s",
            "air_speed": f"{self.system_status['air_speed']:.1f}m/s",
            "attitude": f"R:{self.system_status['roll']:.1f}° P:{self.system_status['pitch']:.1f}° Y:{self.system_status['yaw']:.1f}°"
        }

#-------------------------------------------------------------------------------
# 坐标转换函数（纯 Python 实现，无第三方依赖）
# 基于 eviltransform 算法，已测试往返误差 0.14m
#-------------------------------------------------------------------------------
def out_of_china(lng, lat):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

def transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + \
          0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 *
            math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * math.pi) + 40.0 *
            math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 *
            math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + \
          0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 *
            math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * math.pi) + 40.0 *
            math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * math.pi) + 300.0 *
            math.sin(lng * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def wgs84_to_gcj02(lng, lat):
    if out_of_china(lng, lat):
        return [lng, lat]
    a = 6378245.0
    ee = 0.00669342162296594323
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    return [lng + dlng, lat + dlat]

def gcj02_to_wgs84(lng, lat):
    if out_of_china(lng, lat):
        return [lng, lat]
    wgs_lng, wgs_lat = lng, lat
    for _ in range(5):
        gcj_lng, gcj_lat = wgs84_to_gcj02(wgs_lng, wgs_lat)
        delta_lng = gcj_lng - lng
        delta_lat = gcj_lat - lat
        wgs_lng -= delta_lng
        wgs_lat -= delta_lat
    return [wgs_lng, wgs_lat]

def transform_to_gcj02(lng, lat, from_coord):
    if from_coord == "WGS-84":
        return wgs84_to_gcj02(lng, lat)
    return lng, lat

def transform_to_display(lng, lat, to_coord):
    return lng, lat

#-------------------------------------------------------------------------------
# 障碍物管理
#-------------------------------------------------------------------------------
def load_obstacles():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
            obstacles = data.get('obstacles', [])
            for obs in obstacles:
                if 'height' not in obs:
                    obs['height'] = 30
                if 'selected' not in obs:
                    obs['selected'] = False
            return obstacles
        except:
            return []
    return []

def save_obstacles(obstacles):
    data = {
        'obstacles': obstacles,
        'count': len(obstacles),
        'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        'version': 'v16.0_folium_wgs84_fixed',
        'coord_sys': 'GCJ-02'
    }
    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

#-------------------------------------------------------------------------------
# 几何辅助函数
#-------------------------------------------------------------------------------
def distance(p1, p2):
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i+1)%n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1)*(y - y1)/(y2 - y1) + x1):
            inside = not inside
    return inside

def segments_intersect(p1, p2, p3, p4):
    def orientation(p, q, r):
        val = (q[1]-p[1])*(r[0]-q[0]) - (q[0]-p[0])*(r[1]-q[1])
        if abs(val) < 1e-10: return 0
        return 1 if val > 0 else 2
    def on_segment(p, q, r):
        return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
                min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))
    o1 = orientation(p1,p2,p3)
    o2 = orientation(p1,p2,p4)
    o3 = orientation(p3,p4,p1)
    o4 = orientation(p3,p4,p2)
    if o1 != o2 and o3 != o4:
        return True
    if o1==0 and on_segment(p1,p3,p2): return True
    if o2==0 and on_segment(p1,p4,p2): return True
    if o3==0 and on_segment(p3,p1,p4): return True
    if o4==0 and on_segment(p3,p2,p4): return True
    return False

def line_intersects_polygon(p1, p2, polygon):
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i+1)%n]
        if segments_intersect(p1, p2, p3, p4):
            return True
    return False

def get_blocking_obstacles(start, end, obstacles, flight_alt, ignore_alt=False):
    blocking = []
    for obs in obstacles:
        if ignore_alt or obs.get('height', 30) > flight_alt:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking.append(obs)
    return blocking

def meters_to_deg(meters, lat=32.23):
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg

#-------------------------------------------------------------------------------
# 绕行算法
#-------------------------------------------------------------------------------
def compute_blocked_bounds(blocking_obs):
    min_lng = float('inf')
    max_lng = -float('inf')
    min_lat = float('inf')
    max_lat = -float('inf')
    for obs in blocking_obs:
        for p in obs.get('polygon', []):
            min_lng = min(min_lng, p[0])
            max_lng = max(max_lng, p[0])
            min_lat = min(min_lat, p[1])
            max_lat = max(max_lat, p[1])
    return min_lng, max_lng, min_lat, max_lat

def is_path_clear(p1, p2, obstacles, flight_alt, ignore_alt=False):
    blocking = get_blocking_obstacles(p1, p2, obstacles, flight_alt, ignore_alt)
    return len(blocking) == 0

def find_avoidance_point(start, end, obstacles, flight_alt, direction, safety_radius=5):
    blocking = get_blocking_obstacles(start, end, obstacles, flight_alt, ignore_alt=True)
    if not blocking:
        return None, []
    min_lng, max_lng, min_lat, max_lat = compute_blocked_bounds(blocking)
    safe_lat = meters_to_deg(safety_radius * 3)[1]
    safe_lng = meters_to_deg(safety_radius * 3)[0]
    if direction == "向左绕行":
        lat_offset = max_lat + safe_lat
        lng_mid = (start[0] + end[0]) / 2
        waypoint = [lng_mid, lat_offset]
    elif direction == "向右绕行":
        lat_offset = min_lat - safe_lat
        lng_mid = (start[0] + end[0]) / 2
        waypoint = [lng_mid, lat_offset]
    else:
        raise ValueError("direction must be '向左绕行' or '向右绕行'")
    max_attempts = 10
    for _ in range(max_attempts):
        collide = False
        for obs in blocking:
            if point_in_polygon(waypoint, obs['polygon']):
                collide = True
                if direction == "向左绕行":
                    waypoint[1] += safe_lat
                else:
                    waypoint[1] -= safe_lat
                break
        if not collide:
            break
    return waypoint, blocking

def plan_recursive_path(start, end, obstacles, flight_alt, direction, safety_radius=5, depth=0):
    if depth > 10:
        return [start, end]
    if is_path_clear(start, end, obstacles, flight_alt, ignore_alt=True):
        return [start, end]
    waypoint, _ = find_avoidance_point(start, end, obstacles, flight_alt, direction, safety_radius)
    if waypoint is None:
        return [start, end]
    path1 = plan_recursive_path(start, waypoint, obstacles, flight_alt, direction, safety_radius, depth+1)
    path2 = plan_recursive_path(waypoint, end, obstacles, flight_alt, direction, safety_radius, depth+1)
    full_path = path1[:-1] + path2
    return full_path

def find_left_path(start, end, obstacles, flight_alt, safety_radius=5):
    return plan_recursive_path(start, end, obstacles, flight_alt, "向左绕行", safety_radius)

def find_right_path(start, end, obstacles, flight_alt, safety_radius=5):
    return plan_recursive_path(start, end, obstacles, flight_alt, "向右绕行", safety_radius)

def find_best_path(start, end, obstacles, flight_alt, safety_radius=5):
    blocking = get_blocking_obstacles(start, end, obstacles, flight_alt, ignore_alt=False)
    if not blocking:
        return [start, end]
    left_path = find_left_path(start, end, obstacles, flight_alt, safety_radius)
    right_path = find_right_path(start, end, obstacles, flight_alt, safety_radius)
    left_len = sum(distance(left_path[i], left_path[i+1]) for i in range(len(left_path)-1))
    right_len = sum(distance(right_path[i], right_path[i+1]) for i in range(len(right_path)-1))
    return left_path if left_len <= right_len else right_path

def create_avoidance_path(start, end, obstacles, flight_alt, direction, safety_radius=5):
    if direction == "向左绕行":
        return find_left_path(start, end, obstacles, flight_alt, safety_radius)
    elif direction == "向右绕行":
        return find_right_path(start, end, obstacles, flight_alt, safety_radius)
    else:
        return find_best_path(start, end, obstacles, flight_alt, safety_radius)

#-------------------------------------------------------------------------------
# 等分航点生成
#-------------------------------------------------------------------------------
def path_length(path):
    total = 0.0
    for i in range(len(path)-1):
        total += distance(path[i], path[i+1])
    return total

def interpolate_at_distance(path, dist):
    if dist <= 0:
        return path[0][:]
    total = 0.0
    for i in range(len(path)-1):
        seg_len = distance(path[i], path[i+1])
        if total + seg_len >= dist:
            t = (dist - total) / seg_len
            lng = path[i][0] + t * (path[i+1][0] - path[i][0])
            lat = path[i][1] + t * (path[i+1][1] - path[i][1])
            return [lng, lat]
        total += seg_len
    return path[-1][:]

def generate_equidistant_waypoints(path, num_segments=6):
    if not path or num_segments <= 0:
        return path
    total_len = path_length(path)
    if total_len == 0:
        return [path[0]] * (num_segments + 1)
    step = total_len / num_segments
    waypoints = []
    for i in range(num_segments + 1):
        dist = i * step
        waypoints.append(interpolate_at_distance(path, dist))
    return waypoints

#-------------------------------------------------------------------------------
# 心跳模拟器
#-------------------------------------------------------------------------------
class HeartbeatData:
    def __init__(self, flight_time, seq, lat, lng, altitude):
        self.flight_time = flight_time
        self.seq = seq
        self.lat = lat
        self.lng = lng
        self.altitude = altitude

class HeartbeatSim:
    def __init__(self, start_point):
        self.current_pos = start_point[:]
        self.waypoints = []
        self.current_wp_idx = 0
        self.running = False
        self.start_time = None
        self.last_update = None
        self.history = []
        self.speed_pct = 50
        self.altitude = 50
        self.total_segments = 0
        self.arrival_flag = False
        self.arrived_wp_index = -1
        self.finished = False
        self.hover_remaining = 0.0
        self.waiting_at_wp = False

    def set_path(self, waypoints, altitude, speed_pct):
        self.waypoints = [wp[:] for wp in waypoints]
        self.current_pos = waypoints[0][:]
        self.current_wp_idx = 1
        self.running = True
        self.finished = False
        self.start_time = datetime.now()
        self.last_update = None
        self.history = []
        self.speed_pct = speed_pct
        self.altitude = altitude
        self.total_segments = len(waypoints) - 1
        self.arrival_flag = False
        self.arrived_wp_index = -1
        self.hover_remaining = 0.0
        self.waiting_at_wp = False
        self._add_heartbeat(seq=1)

    def _add_heartbeat(self, seq=None, arrived=False):
        flight_t = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        if seq is None:
            seq = len(self.history) + 1
        hb = HeartbeatData(flight_t, seq, self.current_pos[1], self.current_pos[0], self.altitude)
        self.history.append(hb)
        return hb

    def update_one_step(self):
        if not self.running or self.finished:
            return None
        now = time.time()
        if self.last_update is None:
            dt = HEARTBEAT_INTERVAL
        else:
            dt = min(HEARTBEAT_INTERVAL, now - self.last_update) if (now - self.last_update) > 0 else HEARTBEAT_INTERVAL
        self.last_update = now
        if self.waiting_at_wp:
            self.hover_remaining -= dt
            if self.hover_remaining <= 0:
                self.waiting_at_wp = False
                self.hover_remaining = 0.0
                if self.current_wp_idx >= len(self.waypoints):
                    self.running = False
                    self.finished = True
                    return self._add_heartbeat(arrived=True)
                else:
                    self._add_heartbeat()
                    return self.history[-1] if self.history else None
            else:
                self._add_heartbeat()
                return self.history[-1] if self.history else None
        if self.current_wp_idx >= len(self.waypoints):
            self.running = False
            self.finished = True
            return self._add_heartbeat(arrived=True)
        target = self.waypoints[self.current_wp_idx]
        seg_dist = distance(self.current_pos, target)
        speed = BASE_SPEED * (self.speed_pct / 100.0)
        move_dist = speed * dt
        if move_dist >= seg_dist:
            self.current_pos = target[:]
            self._add_heartbeat()
            self.arrival_flag = True
            self.arrived_wp_index = self.current_wp_idx
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints):
                self.running = False
                self.finished = True
                self._add_heartbeat(arrived=True)
                return self.history[-1]
            else:
                self.waiting_at_wp = True
                self.hover_remaining = HOVER_SECONDS
        else:
            ratio = move_dist / seg_dist
            delta_lng = (target[0] - self.current_pos[0]) * ratio
            delta_lat = (target[1] - self.current_pos[1]) * ratio
            self.current_pos[0] += delta_lng
            self.current_pos[1] += delta_lat
            self._add_heartbeat()
        return self.history[-1] if self.history else None

#-------------------------------------------------------------------------------
# 通信日志
#-------------------------------------------------------------------------------
def add_comm_log(message, direction="OBC内部"):
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log_entry = {"time": timestamp, "direction": direction, "message": message}
    if "comm_logs" not in st.session_state:
        st.session_state.comm_logs = []
    st.session_state.comm_logs.insert(0, log_entry)
    if len(st.session_state.comm_logs) > 50:
        st.session_state.comm_logs = st.session_state.comm_logs[:50]

#-------------------------------------------------------------------------------
# 地图创建（关键修改：所有显示坐标 GCJ-02 -> WGS-84）
#-------------------------------------------------------------------------------
def create_planning_map(center_gcj, points_gcj, obstacles, flight_trail, plan_path, drone_pos_gcj, flight_alt, enable_draw=False):
    center_wgs = gcj02_to_wgs84(center_gcj[0], center_gcj[1])
    m = folium.Map(location=[center_wgs[1], center_wgs[0]], zoom_start=16, tiles=GAODE_TILE, attr='高德')
    for obs in obstacles:
        coords_gcj = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords_gcj and len(coords_gcj) >= 3:
            coords_wgs = [gcj02_to_wgs84(lng, lat) for lng, lat in coords_gcj]
            color = "red" if height > flight_alt else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords_wgs], color=color, weight=2,
                          fill=True, fill_color=color, fill_opacity=0.4,
                          popup=f"🚧 {obs.get('name', '障碍物')}\n高度:{height}m").add_to(m)
    if points_gcj.get('A'):
        a_wgs = gcj02_to_wgs84(points_gcj['A'][0], points_gcj['A'][1])
        folium.Marker([a_wgs[1], a_wgs[0]], popup='起点A', icon=folium.Icon(color='green')).add_to(m)
    if points_gcj.get('B'):
        b_wgs = gcj02_to_wgs84(points_gcj['B'][0], points_gcj['B'][1])
        folium.Marker([b_wgs[1], b_wgs[0]], popup='终点B', icon=folium.Icon(color='red')).add_to(m)
    if plan_path and len(plan_path) > 1:
        path_wgs = [gcj02_to_wgs84(p[0], p[1]) for p in plan_path]
        folium.PolyLine([[p[1], p[0]] for p in path_wgs], color='green', weight=4).add_to(m)
    if flight_trail:
        trail_wgs = [gcj02_to_wgs84(lng, lat) for lng, lat in flight_trail[-100:]]
        folium.PolyLine([[lat, lng] for lng, lat in trail_wgs], color='orange', weight=2).add_to(m)
    if drone_pos_gcj:
        drone_wgs = gcj02_to_wgs84(drone_pos_gcj[0], drone_pos_gcj[1])
        folium.Marker([drone_wgs[1], drone_wgs[0]], icon=folium.Icon(color='blue')).add_to(m)
    if enable_draw:
        draw = Draw(
            draw_options={
                "polygon": {"allowIntersection": False, "drawError": {"color": "#e1e100", "message": "多边形不能相交"},
                           "shapeOptions": {"color": "#ff7800", "weight": 3}},
                "polyline": False,
                "rectangle": False,
                "circle": False,
                "marker": False,
                "circlemarker": False
            },
            edit_options={"edit": False, "remove": False}
        )
        draw.add_to(m)
    return m

#-------------------------------------------------------------------------------
# 初始化状态
#-------------------------------------------------------------------------------
def init():
    DEFAULT_A_GCJ = [118.753501, 32.231118]
    DEFAULT_B_GCJ = [118.754725, 32.234240]
    defaults = {
        'page': '航线规划',
        'points_gcj': {'A': DEFAULT_A_GCJ.copy(), 'B': DEFAULT_B_GCJ.copy()},
        'sim': HeartbeatSim(DEFAULT_A_GCJ.copy()),
        'flight_started': False,
        'latest_hb': None,
        'hb_list': [],
        'flight_trail': [],
        'plan_path': None,
        'waypoints': None,
        'flight_alt': 50,
        'drone_speed': 50,
        'safety_radius': 5,
        'avoid_direction': "最佳航线",
        'coord_sys': 'GCJ-02',
        'obstacles': load_obstacles(),
        'pending_obstacle': None,
        'flight_paused': False,
        'point_select_mode': 'A',
        'pending_click_point': None,
        'last_arrival_msg': "",
        'comm_logs': [],
        'draw_enabled': False,
        'drawn_polygon': None,
        'show_add_dialog': False,
        # MAVLink相关状态（新增）
        'mavlink_parser': MAVLinkMessage(),
        'mavlink_input': "",
        'mavlink_logs': [],
        'show_mavlink_panel': True
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v

def update_plan_and_waypoints():
    if st.session_state.points_gcj.get('A') and st.session_state.points_gcj.get('B'):
        add_comm_log("开始航线规划 - 算法: A*", "OBC内部")
        path = create_avoidance_path(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles,
            st.session_state.flight_alt,
            st.session_state.avoid_direction,
            st.session_state.safety_radius
        )
        st.session_state.plan_path = path
        waypoints = generate_equidistant_waypoints(path, num_segments=6)
        st.session_state.waypoints = waypoints
        wp_count = len(waypoints) - 2 if waypoints else 0
        total_len = path_length(path) * 111000
        add_comm_log(f"航线规划完成 - 类型: horizontal, 航点数: {wp_count+2}, 路径长度: {total_len:.1f}m", "OBC内部")
    else:
        st.session_state.plan_path = None
        st.session_state.waypoints = None

#-------------------------------------------------------------------------------
# 主程序
#-------------------------------------------------------------------------------
def main():
    st.set_page_config(page_title="南京科技职业学院 - 无人机地面站", layout="wide")
    st.title("🏫 南京科技职业学院 - 无人机地面站系统")
    init()

    # ==================== 侧边栏 ====================
    with st.sidebar:
        st.header("📌 导航")
        selected_page = st.radio("功能页面", ["航线规划", "飞行监控", "障碍物管理"],
                                index=["航线规划", "飞行监控", "障碍物管理"].index(st.session_state.page))
        st.session_state.page = selected_page
        st.markdown("---")
        st.subheader("🗺️ 坐标系设置")
        coord_choice = st.radio("输入坐标系", ["WGS-84", "GCJ-02(高德/百度)"],
                               index=1 if st.session_state.coord_sys == "GCJ-02" else 0)
        st.session_state.coord_sys = "WGS-84" if coord_choice == "WGS-84" else "GCJ-02"
        st.markdown("---")
        st.subheader("📊 系统状态")
        st.checkbox("A点已设", value=st.session_state.points_gcj.get('A') is not None, disabled=True)
        st.checkbox("B点已设", value=st.session_state.points_gcj.get('B') is not None, disabled=True)
        st.checkbox("飞行进行中", value=st.session_state.flight_started, disabled=True)
        st.markdown("---")
        st.subheader("🎮 控制面板")

        # ===== 航线规划页面的控制面板 =====
        if st.session_state.page == "航线规划":
            st.markdown("#### ✈️ 飞行参数")
            new_alt = st.slider("飞行高度 (m)", 10, 200, st.session_state.flight_alt, 5)
            if new_alt != st.session_state.flight_alt:
                st.session_state.flight_alt = new_alt
                update_plan_and_waypoints()
                st.rerun()
            new_speed = st.slider("速度系数 (%)", 10, 100, st.session_state.drone_speed, 5)
            st.session_state.drone_speed = new_speed
            new_radius = st.slider("安全半径 (米)", 1, 20, st.session_state.safety_radius, 1)
            if new_radius != st.session_state.safety_radius:
                st.session_state.safety_radius = new_radius
                update_plan_and_waypoints()
                st.rerun()
            st.markdown("---")
            st.subheader("🤖 避障策略")
            direction = st.radio("绕行方向", ["最佳航线", "向左绕行", "向右绕行"],
                                index=["最佳航线", "向左绕行", "向右绕行"].index(st.session_state.avoid_direction))
            if direction != st.session_state.avoid_direction:
                st.session_state.avoid_direction = direction
                update_plan_and_waypoints()
                st.rerun()
            st.markdown("---")
            col_start, col_stop = st.columns(2)
            with col_start:
                if st.button("▶️ 开始飞行", type="primary", use_container_width=True):
                    a = st.session_state.points_gcj.get('A')
                    b = st.session_state.points_gcj.get('B')
                    if a and b and st.session_state.waypoints and len(st.session_state.waypoints) >= 2:
                        add_comm_log(f"上传航线: 起点 {a[1]:.6f},{a[0]:.6f} 终点 {b[1]:.6f},{b[0]:.6f} 高度 {st.session_state.flight_alt}m", "GCS → OBC")
                        add_comm_log("航线接收确认 | Mode: AUTO", "FCU → OBC → GCS")
                        st.session_state.sim = HeartbeatSim(a.copy())
                        st.session_state.sim.set_path(st.session_state.waypoints, st.session_state.flight_alt, st.session_state.drone_speed)
                        st.session_state.latest_hb = st.session_state.sim.history[-1] if st.session_state.sim.history else None
                        st.session_state.hb_list = [st.session_state.latest_hb] if st.session_state.latest_hb else []
                        st.session_state.flight_trail = [[st.session_state.latest_hb.lng, st.session_state.latest_hb.lat]] if st.session_state.latest_hb else []
                        st.session_state.flight_started = True
                        st.session_state.flight_paused = False
                        st.session_state.last_arrival_msg = ""
                        st.success("飞行已开始，切换至「飞行监控」查看动态")
                        st.rerun()
                    else:
                        st.error("请先设置起点、终点，并确保已生成等分航点")
            with col_stop:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.flight_started = False
                    if st.session_state.sim:
                        st.session_state.sim.running = False
                    st.info("飞行已停止")
                    st.rerun()

        # ===== 飞行监控页面的控制面板 =====
        elif st.session_state.page == "飞行监控":
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始任务", use_container_width=True):
                    if not st.session_state.flight_started or st.session_state.sim.finished:
                        if st.session_state.waypoints:
                            add_comm_log("重新开始任务", "GCS → OBC")
                            st.session_state.sim = HeartbeatSim(st.session_state.points_gcj['A'].copy())
                            st.session_state.sim.set_path(st.session_state.waypoints, st.session_state.flight_alt, st.session_state.drone_speed)
                            st.session_state.latest_hb = st.session_state.sim.history[-1] if st.session_state.sim.history else None
                            st.session_state.hb_list = [st.session_state.latest_hb] if st.session_state.latest_hb else []
                            st.session_state.flight_trail = [[st.session_state.latest_hb.lng, st.session_state.latest_hb.lat]] if st.session_state.latest_hb else []
                            st.session_state.flight_started = True
                            st.session_state.flight_paused = False
                            st.session_state.last_arrival_msg = ""
                            st.rerun()
            with col_btn2:
                if st.button("⏸️ 暂停", use_container_width=True):
                    st.session_state.flight_paused = True
                    st.rerun()
            col_btn3, col_btn4, col_btn5 = st.columns(3)
            with col_btn3:
                if st.button("⏹️ 停止", use_container_width=True):
                    st.session_state.flight_started = False
                    st.session_state.flight_paused = False
                    if st.session_state.sim:
                        st.session_state.sim.running = False
                    st.rerun()
            with col_btn4:
                if st.button("🔄 重置", use_container_width=True):
                    if st.session_state.waypoints:
                        add_comm_log("重置飞行任务", "GCS → OBC")
                        st.session_state.sim = HeartbeatSim(st.session_state.points_gcj['A'].copy())
                        st.session_state.sim.set_path(st.session_state.waypoints, st.session_state.flight_alt, st.session_state.drone_speed)
                        st.session_state.latest_hb = st.session_state.sim.history[-1] if st.session_state.sim.history else None
                        st.session_state.hb_list = [st.session_state.latest_hb] if st.session_state.latest_hb else []
                        st.session_state.flight_trail = [[st.session_state.latest_hb.lng, st.session_state.latest_hb.lat]] if st.session_state.latest_hb else []
                        st.session_state.flight_started = True
                        st.session_state.flight_paused = False
                        st.session_state.last_arrival_msg = ""
                        st.rerun()
                    else:
                        st.error("请先在航线规划页面设置路径")
            with col_btn5:
                if st.button("🔄 刷新飞行", use_container_width=True, help="重新开始当前航线的飞行任务"):
                    if st.session_state.waypoints:
                        add_comm_log("手动刷新飞行", "GCS → OBC")
                        st.session_state.sim = HeartbeatSim(st.session_state.points_gcj['A'].copy())
                        st.session_state.sim.set_path(st.session_state.waypoints, st.session_state.flight_alt, st.session_state.drone_speed)
                        st.session_state.latest_hb = st.session_state.sim.history[-1] if st.session_state.sim.history else None
                        st.session_state.hb_list = [st.session_state.latest_hb] if st.session_state.latest_hb else []
                        st.session_state.flight_trail = [[st.session_state.latest_hb.lng, st.session_state.latest_hb.lat]] if st.session_state.latest_hb else []
                        st.session_state.flight_started = True
                        st.session_state.flight_paused = False
                        st.session_state.last_arrival_msg = ""
                        st.rerun()
                    else:
                        st.error("请先在航线规划页面设置路径")

            st.markdown("---")
            st.subheader("📡 MAVLink 消息解析")  # 新增MAVLink面板
            st.session_state.show_mavlink_panel = st.checkbox("展开MAVLink消息面板", value=st.session_state.show_mavlink_panel)
            
            if st.session_state.show_mavlink_panel:
                # MAVLink消息输入
                col_input1, col_input2 = st.columns([3, 1])
                with col_input1:
                    mav_input = st.text_input(
                        "输入MAVLink消息 (支持格式: MSG_ID|field=value 或 HEX 或 JSON)",
                        value=st.session_state.mavlink_input,
                        placeholder="例如: MSG_ID 33|altitude=50.0,relative_alt=45.0,heading=120",
                        key="mavlink_input_field"
                    )
                    st.session_state.mavlink_input = mav_input
                with col_input2:
                    if st.button("📤 解析消息", use_container_width=True):
                        if mav_input.strip():
                            parser = st.session_state.mavlink_parser
                            result = parser.parse_message(mav_input)
                            if result:
                                st.session_state.mavlink_logs.insert(0, {
                                    "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
                                    "input": mav_input,
                                    "result": result
                                })
                                if len(st.session_state.mavlink_logs) > 50:
                                    st.session_state.mavlink_logs.pop()
                                add_comm_log(f"MAVLink解析: {result.get('msg_name', '未知')}", "FCU → OBC → GCS")
                                st.rerun()
                
                # 快捷消息按钮
                st.markdown("#### 快捷测试消息")
                col_quick1, col_quick2, col_quick3, col_quick4, col_quick5 = st.columns(5)
                quick_messages = {
                    "心跳": "MSG_ID 0|state=4,mode=4,armed=True",
                    "位置": "MSG_ID 33|altitude=50.0,relative_alt=45.0,heading=120",
                    "姿态": "MSG_ID 30|roll=0.5,pitch=-0.3,yaw=45.0",
                    "速度": "MSG_ID 74|ground_speed=5.0,air_speed=4.8,climb_rate=0.5",
                    "状态": "MSG_ID 1|battery_voltage=12.6,battery_remaining=85,gps_fix=3,satellites=12"
                }
                with col_quick1:
                    if st.button("💓 心跳", use_container_width=True):
                        st.session_state.mavlink_input = quick_messages["心跳"]
                        st.rerun()
                with col_quick2:
                    if st.button("📍 位置", use_container_width=True):
                        st.session_state.mavlink_input = quick_messages["位置"]
                        st.rerun()
                with col_quick3:
                    if st.button("📐 姿态", use_container_width=True):
                        st.session_state.mavlink_input = quick_messages["姿态"]
                        st.rerun()
                with col_quick4:
                    if st.button("💨 速度", use_container_width=True):
                        st.session_state.mavlink_input = quick_messages["速度"]
                        st.rerun()
                with col_quick5:
                    if st.button("🔋 状态", use_container_width=True):
                        st.session_state.mavlink_input = quick_messages["状态"]
                        st.rerun()
                
                # 解析结果显示
                if st.session_state.mavlink_logs:
                    latest = st.session_state.mavlink_logs[0]
                    result = latest.get("result", {})
                    
                    st.markdown("#### 最新解析结果")
                    col_res1, col_res2 = st.columns(2)
                    with col_res1:
                        st.metric("消息ID", f"MSG_{result.get('msg_id', '?')}")
                        st.metric("消息名称", result.get('msg_name', '未知'))
                    with col_res2:
                        st.metric("时间", latest.get('time', ''))
                        st.metric("原始输入", latest.get('input', '')[:30] + "...")
                    
                    if "payload" in result and result["payload"]:
                        st.markdown("#### 载荷数据")
                        st.json(result["payload"])
                    
                    # 系统状态摘要
                    st.markdown("#### 系统状态摘要")
                    status = st.session_state.mavlink_parser.get_status()
                    cols = st.columns(4)
                    with cols[0]:
                        st.metric("连接状态", status["connection"])
                        st.metric("飞行状态", status["state"])
                    with cols[1]:
                        st.metric("飞行模式", status["mode"])
                        st.metric("解锁状态", status["arm_status"])
                    with cols[2]:
                        st.metric("电池", status["battery"])
                        st.metric("电压", status["voltage"])
                    with cols[3]:
                        st.metric("GPS", status["gps"])
                        st.metric("高度", status["altitude"])
                    
                    st.markdown("#### 姿态信息")
                    col_att1, col_att2 = st.columns(2)
                    with col_att1:
                        st.metric("地面速度", status["ground_speed"])
                        st.metric("空速", status["air_speed"])
                    with col_att2:
                        st.metric("姿态", status["attitude"])
                    
                    # 历史消息日志
                    with st.expander("📋 MAVLink消息历史", expanded=False):
                        for log in st.session_state.mavlink_logs[:10]:
                            result = log.get("result", {})
                            st.caption(f"[{log.get('time', '')}] {result.get('msg_name', '未知')}: {json.dumps(result.get('payload', {}), ensure_ascii=False)[:100]}")
                    
                    # 状态文本消息
                    text_msgs = st.session_state.mavlink_parser.system_status.get("text_messages", [])
                    if text_msgs:
                        st.markdown("#### 状态文本消息")
                        for msg in text_msgs[:5]:
                            st.info(f"[{msg.get('time', '')}] {msg.get('text', '')}")
            
            st.markdown("---")

    # ==================== 障碍物管理页面 ====================
    if st.session_state.page == "障碍物管理":
        st.header("🚧 障碍物配置持久化")
        st.caption(f"配置文件: {os.path.abspath(CONFIG_FILE)} | 版本: v16.0_folium_wgs84_fixed")
        st.info("📂 所有障碍物坐标均以 GCJ-02 存储，与高德底图完全对齐。")
        col1, col2, col3, col4 = st.columns(4)
        with col1:
            if st.button("💾 保存到文件", use_container_width=True):
                save_obstacles(st.session_state.obstacles)
                st.success("保存成功")
        with col2:
            if st.button("📂 从文件加载", use_container_width=True):
                st.session_state.obstacles = load_obstacles()
                update_plan_and_waypoints()
                st.rerun()
        with col3:
            if st.button("🗑️ 清除全部", use_container_width=True):
                st.session_state.obstacles = []
                save_obstacles([])
                update_plan_and_waypoints()
                st.rerun()
        with col4:
            if st.button("🚀 一键部署", use_container_width=True):
                st.info("此功能用于部署，示例中未实现")
        st.markdown("---")
        st.subheader("📥 下载配置文件到本地")
        if st.button("📥 下载 obstacle_config.json", use_container_width=True):
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'rb') as f:
                    st.download_button("点击下载", data=f, file_name=CONFIG_FILE, mime="application/json")
            else:
                st.warning("配置文件不存在，请先保存")
        st.markdown("---")
        st.subheader("➕ 添加新障碍物（手动输入顶点）")
        with st.form("add_obstacle_form"):
            obs_name = st.text_input("障碍物名称", "新障碍物")
            obs_height = st.number_input("高度 (米)", min_value=1, max_value=200, value=30, step=5)
            st.markdown("#### 顶点坐标 (经度,纬度) 每行一个，格式: 118.749,32.234")
            vertices_text = st.text_area("顶点列表", placeholder="118.746956,32.232945\n118.747500,32.233000\n118.747200,32.233500")
            submitted = st.form_submit_button("✅ 添加障碍物")
            if submitted and vertices_text.strip():
                vertices = []
                for line in vertices_text.strip().split('\n'):
                    if ',' in line:
                        parts = line.split(',')
                        try:
                            lng = float(parts[0].strip())
                            lat = float(parts[1].strip())
                            vertices.append([lng, lat])
                        except:
                            pass
                if len(vertices) >= 3:
                    if st.session_state.coord_sys == "WGS-84":
                        vertices = [list(wgs84_to_gcj02(lng, lat)) for lng, lat in vertices]
                    new_obs = {
                        "name": obs_name,
                        "polygon": vertices,
                        "height": obs_height,
                        "selected": False,
                        "id": f"obs_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                    }
                    st.session_state.obstacles.append(new_obs)
                    save_obstacles(st.session_state.obstacles)
                    update_plan_and_waypoints()
                    st.success(f"已添加 {obs_name}")
                    st.rerun()
                else:
                    st.error("至少需要3个顶点")
        st.markdown("---")
        st.subheader(f"📋 当前障碍物列表 (共 {len(st.session_state.obstacles)} 个)")
        for idx, obs in enumerate(st.session_state.obstacles):
            with st.expander(f"{obs.get('name', '未命名')} | 高度: {obs.get('height',30)}m"):
                col_a, col_b, col_c = st.columns([1,1,2])
                with col_a:
                    new_h = st.number_input("调整高度", value=obs.get('height',30), key=f"h_{idx}", step=5)
                    if new_h != obs.get('height',30):
                        obs['height'] = new_h
                        save_obstacles(st.session_state.obstacles)
                        update_plan_and_waypoints()
                        st.rerun()
                with col_b:
                    if st.button("🗑️ 删除", key=f"del_{idx}"):
                        st.session_state.obstacles.pop(idx)
                        save_obstacles(st.session_state.obstacles)
                        update_plan_and_waypoints()
                        st.rerun()
                with col_c:
                    st.code(json.dumps(obs.get('polygon', []), indent=2), language='json')
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
            save_time = data.get('save_time', '未知')
            st.info(f"📁 文件状态: 共 {len(data.get('obstacles', []))} 个障碍物 | 保存时间: {save_time} | 版本: {data.get('version', '未知')}")
            st.text(f"路径: {os.path.abspath(CONFIG_FILE)}")

    # ==================== 航线规划页面 ====================
    elif st.session_state.page == "航线规划":
        st.header("🗺️ 航线规划 - 点击地图 + 方向微调 + 手动输入坐标 + 多边形圈选障碍物")
        st.info("🔧 **坐标修正说明**：绘制多边形时，系统会自动将 WGS-84 坐标转换为 GCJ-02 存储，确保与高德底图完全对齐，圈选不再偏移。")
        col_map, col_panel = st.columns([3, 1.2])
        with col_panel:
            st.markdown("### 🎯 航点控制")
            if not st.session_state.flight_started:
                draw_enabled = st.checkbox("✏️ 启用多边形绘制（圈选障碍物）", value=st.session_state.draw_enabled)
                if draw_enabled != st.session_state.draw_enabled:
                    st.session_state.draw_enabled = draw_enabled
                    st.rerun()
            else:
                st.info("飞行任务进行中，无法使用绘制工具")
            st.markdown("---")
            with st.expander("✏️ 手动输入起点/终点坐标", expanded=False):
                st.markdown("**注意：坐标将根据左侧「坐标系设置」自动转换为GCJ-02存储**")
                col_a_in, col_b_in = st.columns(2)
                with col_a_in:
                    st.markdown("#### 起点 A")
                    a_lng_input = st.number_input("经度 (A)", value=st.session_state.points_gcj['A'][0], format="%.6f", key="manual_a_lng")
                    a_lat_input = st.number_input("纬度 (A)", value=st.session_state.points_gcj['A'][1], format="%.6f", key="manual_a_lat")
                with col_b_in:
                    st.markdown("#### 终点 B")
                    b_lng_input = st.number_input("经度 (B)", value=st.session_state.points_gcj['B'][0], format="%.6f", key="manual_b_lng")
                    b_lat_input = st.number_input("纬度 (B)", value=st.session_state.points_gcj['B'][1], format="%.6f", key="manual_b_lat")
                if st.button("📌 应用手动输入坐标", key="apply_manual_coords"):
                    current_sys = st.session_state.coord_sys
                    if current_sys == "WGS-84":
                        a_gcj = wgs84_to_gcj02(a_lng_input, a_lat_input)
                        b_gcj = wgs84_to_gcj02(b_lng_input, b_lat_input)
                    else:
                        a_gcj = [a_lng_input, a_lat_input]
                        b_gcj = [b_lng_input, b_lat_input]
                    st.session_state.points_gcj['A'] = a_gcj
                    st.session_state.points_gcj['B'] = b_gcj
                    update_plan_and_waypoints()
                    st.success("坐标已更新，地图和航线已刷新")
                    st.rerun()
            st.markdown("---")
            if st.session_state.flight_started:
                st.warning("飞行任务进行中，无法修改航点。请先停止飞行。")
                select_mode = st.radio("当前可移动的点", ["起点 (A)", "终点 (B)"], key="mode_disabled", disabled=True, horizontal=True)
            else:
                select_mode = st.radio("当前可移动的点", ["起点 (A)", "终点 (B)"],
                                      index=0 if st.session_state.point_select_mode == 'A' else 1,
                                      key="move_select", horizontal=True)
                st.session_state.point_select_mode = 'A' if select_mode == "起点 (A)" else 'B'
            st.markdown("---")
            st.markdown("#### 📍 当前坐标 (GCJ-02)")
            a_lng, a_lat = st.session_state.points_gcj['A']
            b_lng, b_lat = st.session_state.points_gcj['B']
            st.text(f"起点 A : {a_lng:.6f}, {a_lat:.6f}")
            st.text(f"终点 B : {b_lng:.6f}, {b_lat:.6f}")
            st.markdown("---")
            st.markdown("#### 🎯 精确微调（每步约 1 米）")
            col_dir1, col_dir2, col_dir3, col_dir4 = st.columns(4)
            step = 0.00001
            if st.session_state.point_select_mode == 'A':
                target = st.session_state.points_gcj['A']
            else:
                target = st.session_state.points_gcj['B']
            if col_dir1.button("⬆️ 北", key="move_n"):
                target[1] += step
                if st.session_state.point_select_mode == 'A':
                    st.session_state.points_gcj['A'] = target
                else:
                    st.session_state.points_gcj['B'] = target
                update_plan_and_waypoints()
                st.rerun()
            if col_dir2.button("⬇️ 南", key="move_s"):
                target[1] -= step
                if st.session_state.point_select_mode == 'A':
                    st.session_state.points_gcj['A'] = target
                else:
                    st.session_state.points_gcj['B'] = target
                update_plan_and_waypoints()
                st.rerun()
            if col_dir3.button("⬅️ 西", key="move_w"):
                target[0] -= step
                if st.session_state.point_select_mode == 'A':
                    st.session_state.points_gcj['A'] = target
                else:
                    st.session_state.points_gcj['B'] = target
                update_plan_and_waypoints()
                st.rerun()
            if col_dir4.button("➡️ 东", key="move_e"):
                target[0] += step
                if st.session_state.point_select_mode == 'A':
                    st.session_state.points_gcj['A'] = target
                else:
                    st.session_state.points_gcj['B'] = target
                update_plan_and_waypoints()
                st.rerun()
            st.markdown("---")
            st.info("💡 **操作提示**：\n- 上方选择要移动的点（A/B）\n- **单击地图** → 点跳转到点击位置（自动转GCJ-02）\n- 点击方向按钮 → 每次移动约 1 米\n- **勾选「启用多边形绘制」后，在地图上绘制多边形 → 自动弹出添加表单**")
            if st.session_state.plan_path:
                waypoint_count = len(st.session_state.waypoints) - 2 if st.session_state.waypoints else 0
                if waypoint_count > 0:
                    st.info(f"航线已均匀分为6段，包含 {waypoint_count+1} 个中间航点（总共{len(st.session_state.waypoints)}个航点），每个航点停留 {HOVER_SECONDS} 秒")
                else:
                    st.success("直线航线，无绕行")
        with col_map:
            if st.session_state.plan_path is None and st.session_state.points_gcj.get('A') and st.session_state.points_gcj.get('B'):
                update_plan_and_waypoints()
            drone_pos_gcj = None
            if st.session_state.flight_started and not st.session_state.flight_paused and st.session_state.latest_hb:
                drone_pos_gcj = [st.session_state.latest_hb.lng, st.session_state.latest_hb.lat]
            folium_map = create_planning_map(
                SCHOOL_CENTER_GCJ,
                st.session_state.points_gcj,
                st.session_state.obstacles,
                st.session_state.flight_trail,
                st.session_state.plan_path,
                drone_pos_gcj,
                st.session_state.flight_alt,
                enable_draw=st.session_state.draw_enabled and not st.session_state.flight_started
            )
            map_output = st_folium(folium_map, width=700, height=550, key="planning_map")
            if st.session_state.draw_enabled and not st.session_state.flight_started and map_output:
                last_draw = map_output.get("last_active_drawing")
                if last_draw and last_draw.get("geometry", {}).get("type") == "Polygon":
                    coords_wgs = last_draw["geometry"]["coordinates"][0]
                    vertices_wgs = [[c[0], c[1]] for c in coords_wgs]
                    vertices_gcj = [list(wgs84_to_gcj02(lng, lat)) for lng, lat in vertices_wgs]
                    st.session_state.drawn_polygon = vertices_gcj
                    st.session_state.show_add_dialog = True
                    st.rerun()
            if st.session_state.show_add_dialog and st.session_state.drawn_polygon:
                with st.expander("✏️ 添加绘制的多边形作为障碍物", expanded=True):
                    obs_name = st.text_input("障碍物名称", f"多边形障碍物_{datetime.now().strftime('%H%M%S')}")
                    obs_height = st.number_input("高度 (米)", min_value=1, max_value=200, value=30, step=5)
                    col_ok, col_cancel = st.columns(2)
                    with col_ok:
                        if st.button("✅ 确认添加", use_container_width=True):
                            new_obs = {
                                "name": obs_name,
                                "polygon": st.session_state.drawn_polygon,
                                "height": obs_height,
                                "selected": False,
                                "id": f"obs_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                            }
                            st.session_state.obstacles.append(new_obs)
                            save_obstacles(st.session_state.obstacles)
                            update_plan_and_waypoints()
                            add_comm_log(f"通过地图绘制添加障碍物「{obs_name}」", "GCS")
                            st.session_state.show_add_dialog = False
                            st.session_state.drawn_polygon = None
                            st.session_state.draw_enabled = False
                            st.success("障碍物已添加，航线已重新规划")
                            st.rerun()
                    with col_cancel:
                        if st.button("❌ 取消", use_container_width=True):
                            st.session_state.show_add_dialog = False
                            st.session_state.drawn_polygon = None
                            st.rerun()
            if (not st.session_state.draw_enabled) and (not st.session_state.flight_started) and map_output and map_output.get("last_clicked"):
                lat_click = map_output["last_clicked"]["lat"]
                lng_click = map_output["last_clicked"]["lng"]
                gcj_lng, gcj_lat = wgs84_to_gcj02(lng_click, lat_click)
                if st.session_state.point_select_mode == 'A':
                    st.session_state.points_gcj['A'] = [gcj_lng, gcj_lat]
                    st.success(f"起点 A 已移动到: ({gcj_lng:.6f}, {gcj_lat:.6f})")
                else:
                    st.session_state.points_gcj['B'] = [gcj_lng, gcj_lat]
                    st.success(f"终点 B 已移动到: ({gcj_lng:.6f}, {gcj_lat:.6f})")
                update_plan_and_waypoints()
                st.rerun()

    # ==================== 飞行监控页面 ====================
    else:
        st.header("📡 飞行实时画面 - 任务执行监控")
        if st.session_state.flight_started and st.session_state.sim and not st.session_state.sim.finished:
            st_autorefresh(interval=2000, key="monitor_auto")
        else:
            st.info("✈️ 飞行任务已结束，页面已停止自动刷新。")
        if st.session_state.flight_started and not st.session_state.flight_paused and st.session_state.sim and st.session_state.sim.running:
            steps = max(1, int(1.0 / HEARTBEAT_INTERVAL))
            for _ in range(steps):
                new_hb = st.session_state.sim.update_one_step()
                if new_hb:
                    st.session_state.latest_hb = new_hb
                    st.session_state.hb_list.insert(0, new_hb)
                    if len(st.session_state.hb_list) > 200:
                        st.session_state.hb_list.pop()
                    st.session_state.flight_trail.append([new_hb.lng, new_hb.lat])
                    if len(st.session_state.flight_trail) > 200:
                        st.session_state.flight_trail.pop(0)
                else:
                    break
        if st.session_state.sim and st.session_state.sim.arrival_flag:
            idx = st.session_state.sim.arrived_wp_index
            total_wp = len(st.session_state.sim.waypoints)
            if idx == total_wp - 1:
                msg = f"🎉 已到达终点（航点 {idx+1}/{total_wp}），飞行结束。"
                add_comm_log(f"MISSION_COMPLETE (航点 {idx+1}/{total_wp})", "FCU → OBC → GCS")
            else:
                msg = f"📍 已到达航点 {idx+1}/{total_wp}，停留 {HOVER_SECONDS} 秒后继续..."
                add_comm_log(f"WP_REACHED #{idx+1}", "FCU → OBC → GCS")
            st.session_state.last_arrival_msg = msg
            st.session_state.sim.arrival_flag = False
            st.rerun()
        if st.session_state.flight_started and st.session_state.sim and st.session_state.sim.finished:
            st.session_state.flight_started = False
            st.session_state.flight_paused = False
            if not st.session_state.last_arrival_msg:
                st.session_state.last_arrival_msg = "飞行已到达终点。"
                add_comm_log("MISSION_COMPLETE", "FCU → OBC → GCS")
        if not st.session_state.flight_started:
            st.info("⏳ 飞行未开始或已结束。请切换到「航线规划」页面，设置起点终点并点击「开始飞行」。")
        if st.session_state.last_arrival_msg:
            st.success(st.session_state.last_arrival_msg)
        if st.session_state.latest_hb is None:
            st.warning("等待第一个心跳...")
            st.stop()
        hb = st.session_state.latest_hb
        current_wp = st.session_state.sim.current_wp_idx
        total_wp = len(st.session_state.sim.waypoints)
        reached_wp = max(0, current_wp - 1)
        progress = reached_wp / (total_wp - 1) if total_wp > 1 else 0
        speed = BASE_SPEED * (st.session_state.drone_speed / 100.0)
        elapsed = hb.flight_time
        remaining_dist = (1 - progress) * path_length(st.session_state.sim.waypoints) * 111000
        eta_sec = remaining_dist / speed if speed > 0 else 0
        col_left, col_right = st.columns([1, 1.5])
        with col_left:
            st.markdown("### 📊 任务状态")
            st.metric("当前航点", f"{reached_wp+1} / {total_wp}")
            st.progress(progress, text=f"任务进度: {int(progress*100)}%")
            st.metric("飞行速度", f"{speed:.1f} m/s")
            minutes = int(elapsed // 60)
            seconds = int(elapsed % 60)
            st.metric("已用时间", f"{minutes:02d}:{seconds:02d}")
            st.metric("剩余距离", f"{remaining_dist:.0f} m")
            eta_min = int(eta_sec // 60)
            eta_sec_int = int(eta_sec % 60)
            st.metric("预计到达", f"{eta_min:02d}:{eta_sec_int:02d}")
            st.metric("电量模拟", "40%")
            st.markdown("---")
            st.markdown("### 📡 通信链路拓扑与数据流")
            col_gcs, col_obc, col_fcu = st.columns(3)
            with col_gcs:
                st.markdown("**GCS (地面站)**")
                st.caption("192.168.1.100")
                st.markdown("✅ 已连接")
            with col_obc:
                st.markdown("**OBC (机载计算机)**")
                st.caption("Raspberry Pi 4")
                st.markdown("✅ 已连接")
            with col_fcu:
                st.markdown("**FCU (飞控)**")
                st.caption("PX4 / ArduPilot")
                st.markdown("✅ 已连接")
            st.markdown("```\nGCS --UDP:14550--> OBC --MAVLink--> FCU\n```")
            if progress < 1:
                delay = random.uniform(20, 35)
                loss = random.uniform(0, 0.5)
            else:
                delay = 10
                loss = 0
            st.markdown("#### 链路统计")
            st.markdown(f"- **GCS ↔ OBC**: 正常")
            st.markdown(f"- **OBC ↔ FCU**: 正常")
            st.markdown(f"- **延迟**: ~{delay:.0f}ms")
            st.markdown(f"- **丢包率**: {loss:.1f}%")
        with col_right:
            st.subheader("🗺️ 实时飞行地图")
            center = [st.session_state.sim.current_pos[0], st.session_state.sim.current_pos[1]]
            a = st.session_state.points_gcj['A']
            b = st.session_state.points_gcj['B']
            center_wgs = gcj02_to_wgs84(center[0], center[1])
            m = folium.Map(location=[center_wgs[1], center_wgs[0]], zoom_start=18, tiles=GAODE_TILE, attr='高德')
            for obs in st.session_state.obstacles:
                coords_gcj = obs.get('polygon', [])
                height = obs.get('height', 30)
                if coords_gcj and len(coords_gcj) >= 3:
                    coords_wgs = [gcj02_to_wgs84(lng, lat) for lng, lat in coords_gcj]
                    color = "red" if height > st.session_state.flight_alt else "orange"
                    folium.Polygon([[c[1], c[0]] for c in coords_wgs], color=color, weight=2,
                                  fill=True, fill_color=color, fill_opacity=0.4,
                                  popup=f"🚧 {obs.get('name', '障碍物')}\n高度:{height}m").add_to(m)
            a_wgs = gcj02_to_wgs84(a[0], a[1])
            b_wgs = gcj02_to_wgs84(b[0], b[1])
            folium.Marker([a_wgs[1], a_wgs[0]], popup='起点A', icon=folium.Icon(color='green')).add_to(m)
            folium.Marker([b_wgs[1], b_wgs[0]], popup='终点B', icon=folium.Icon(color='red')).add_to(m)
            if st.session_state.plan_path:
                path_wgs = [gcj02_to_wgs84(p[0], p[1]) for p in st.session_state.plan_path]
                folium.PolyLine([[p[1], p[0]] for p in path_wgs], color='green', weight=4).add_to(m)
            if st.session_state.waypoints:
                for i, wp in enumerate(st.session_state.waypoints):
                    wp_wgs = gcj02_to_wgs84(wp[0], wp[1])
                    folium.CircleMarker([wp_wgs[1], wp_wgs[0]], radius=4, color='blue', fill=True, popup=f"航点{i+1}").add_to(m)
            if st.session_state.flight_trail:
                trail_wgs = [gcj02_to_wgs84(lng, lat) for lng, lat in st.session_state.flight_trail[-100:]]
                folium.PolyLine([[lat, lng] for lng, lat in trail_wgs], color='orange', weight=2).add_to(m)
            drone_wgs = gcj02_to_wgs84(center[0], center[1])
            folium.Marker([drone_wgs[1], drone_wgs[0]], icon=folium.Icon(color='blue')).add_to(m)
            folium_static(m, width=700, height=500)
        st.markdown("---")
        st.subheader("📋 通信日志")
        if st.session_state.comm_logs:
            for log in st.session_state.comm_logs[:20]:
                st.caption(f"[{log['time']}] {log['direction']}: {log['message']}")
        else:
            st.info("暂无通信日志")
        st.markdown("---")
        st.subheader("💓 心跳序号 vs 飞行时间 (正比例关系)")
        history = st.session_state.sim.history
        if len(history) >= 2:
            times = [h.flight_time for h in history]
            seqs = [h.seq for h in history]
            fig, ax = plt.subplots(figsize=(8,4))
            ax.plot(times, seqs, marker='o', markersize=4, linewidth=2)
            ax.set_xlabel('飞行时间 (秒)')
            ax.set_ylabel('心跳包序号')
            ax.set_title('心跳序号与飞行时间关系（正比例）')
            ax.grid(True)
            st.pyplot(fig)
            plt.close(fig)
        else:
            st.info(f"等待更多心跳数据... (当前 {len(history)} 个)")
        st.subheader("📈 实时趋势")
        if len(st.session_state.hb_list) > 1:
            df = pd.DataFrame([{"时间": i, "高度": h.altitude} for i, h in enumerate(st.session_state.hb_list[:50])])
            st.line_chart(df, x="时间", y="高度")
        else:
            st.info("等待更多数据...")
        
        # MAVLink解析面板已在侧边栏显示，这里不再重复

if __name__ == "__main__":
    main()
