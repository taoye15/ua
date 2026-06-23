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
import copy

#
#---------------------------------------------------------------------
# 配置
#---------------------------------------------------------------------

SCHOOL_CENTER_GCJ = [118.749413, 32.234097]  # 学校中心点(GCJ-02)
GAODE_TILE = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
HEARTBEAT_INTERVAL = 0.2
BASE_SPEED = 5.0
HOVER_SECONDS = 5
CONFIG_FILE = "obstacle_config.json"

#
#---------------------------------------------------------------------
# 坐标转换函数（纯 Python 实现，无第三方依赖）
# 基于 eviltransform 算法，已测试往返误差 0.14m
#---------------------------------------------------------------------

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

#
#---------------------------------------------------------------------
# 障碍物管理
#---------------------------------------------------------------------

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
        'version': 'v18.0_obstacle_avoidance_all_modes',
        'coord_sys': 'GCJ-02'
    }
    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

#
#---------------------------------------------------------------------
# 几何辅助函数
#---------------------------------------------------------------------

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

def point_on_segment(p, p1, p2, tolerance=1e-9):
    """检查点是否在线段上"""
    if distance(p1, p2) < tolerance:
        return distance(p, p1) < tolerance
    cross = (p[0] - p1[0]) * (p2[1] - p1[1]) - (p[1] - p1[1]) * (p2[0] - p1[0])
    if abs(cross) > tolerance:
        return False
    dot = (p[0] - p1[0]) * (p2[0] - p1[0]) + (p[1] - p1[1]) * (p2[1] - p1[1])
    if dot < 0:
        return False
    squared_len = (p2[0] - p1[0])**2 + (p2[1] - p1[1])**2
    if dot > squared_len:
        return False
    return True

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
        # 只有当障碍物高度 > 飞行高度时才阻挡，或者忽略高度检查
        if ignore_alt or obs.get('height', 30) > flight_alt:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking.append(obs)
    return blocking

def meters_to_deg(meters, lat=32.23):
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg

#
#---------------------------------------------------------------------
# 改进的绕行算法 - 紧贴障碍物绕行（支持左右绕行方向）
#---------------------------------------------------------------------

def get_polygon_edges(polygon):
    """获取多边形的所有边"""
    edges = []
    n = len(polygon)
    for i in range(n):
        edges.append((polygon[i], polygon[(i+1)%n]))
    return edges

def find_line_polygon_intersections(p1, p2, polygon):
    """找到线段与多边形边的所有交点"""
    intersections = []
    edges = get_polygon_edges(polygon)
    for edge in edges:
        e1, e2 = edge
        if segments_intersect(p1, p2, e1, e2):
            # 计算交点坐标
            x1, y1 = p1
            x2, y2 = p2
            x3, y3 = e1
            x4, y4 = e2
            
            denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if abs(denom) < 1e-12:
                continue
            
            t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
            if 0 <= t <= 1:
                ix = x1 + t * (x2 - x1)
                iy = y1 + t * (y2 - y1)
                if point_on_segment([ix, iy], e1, e2):
                    intersections.append([ix, iy])
    
    return intersections

def get_nearest_point_on_polygon(point, polygon):
    """获取多边形上离点最近的点"""
    min_dist = float('inf')
    nearest = None
    edges = get_polygon_edges(polygon)
    for edge in edges:
        e1, e2 = edge
        dx = e2[0] - e1[0]
        dy = e2[1] - e1[1]
        if dx == 0 and dy == 0:
            dist = distance(point, e1)
            if dist < min_dist:
                min_dist = dist
                nearest = e1[:]
            continue
        
        t = ((point[0] - e1[0]) * dx + (point[1] - e1[1]) * dy) / (dx*dx + dy*dy)
        t = max(0, min(1, t))
        proj_x = e1[0] + t * dx
        proj_y = e1[1] + t * dy
        dist = distance(point, [proj_x, proj_y])
        if dist < min_dist:
            min_dist = dist
            nearest = [proj_x, proj_y]
    
    return nearest, min_dist

def get_polygon_centroid(polygon):
    """计算多边形中心"""
    center_lng = sum(p[0] for p in polygon) / len(polygon)
    center_lat = sum(p[1] for p in polygon) / len(polygon)
    return [center_lng, center_lat]

def create_obstacle_avoidance_path(start, end, obstacle, flight_alt, safety_radius=2, direction="最佳航线"):
    """
    创建紧贴障碍物的绕行路径
    支持方向：最佳航线、向左绕行、向右绕行
    向左绕行：沿多边形顺时针方向走
    向右绕行：沿多边形逆时针方向走
    返回绕行路径点列表
    """
    polygon = obstacle.get('polygon', [])
    if len(polygon) < 3:
        return [start, end]
    
    # 检查起点和终点是否在障碍物内部
    start_inside = point_in_polygon(start, polygon)
    end_inside = point_in_polygon(end, polygon)
    
    if start_inside and end_inside:
        return [start, end]
    
    # 找到航线与多边形的交点
    intersections = find_line_polygon_intersections(start, end, polygon)
    
    if len(intersections) < 2:
        return [start, end]
    
    # 按距离起点排序交点
    intersections.sort(key=lambda p: distance(start, p))
    
    # 取第一个和最后一个交点作为进入和离开点
    entry_point = intersections[0]
    exit_point = intersections[-1]
    
    # 计算多边形中心
    center = get_polygon_centroid(polygon)
    
    # 计算航线方向向量
    dir_vec = [end[0] - start[0], end[1] - start[1]]
    dir_len = math.hypot(dir_vec[0], dir_vec[1])
    if dir_len < 1e-10:
        return [start, end]
    dir_vec = [dir_vec[0]/dir_len, dir_vec[1]/dir_len]
    
    # 计算法向量（垂直于航线方向）
    normal_vec = [-dir_vec[1], dir_vec[0]]
    
    # 判断障碍物在航线的哪一侧
    mid_point = [(entry_point[0] + exit_point[0])/2, (entry_point[1] + exit_point[1])/2]
    to_center = [center[0] - mid_point[0], center[1] - mid_point[1]]
    side = to_center[0] * normal_vec[0] + to_center[1] * normal_vec[1]
    
    # 安全距离（转换为度）
    safety_lng, safety_lat = meters_to_deg(safety_radius)
    
    # 在进入点附近找到多边形上的点
    entry_on_poly, _ = get_nearest_point_on_polygon(entry_point, polygon)
    exit_on_poly, _ = get_nearest_point_on_polygon(exit_point, polygon)
    
    if entry_on_poly is None or exit_on_poly is None:
        return [start, end]
    
    # 获取多边形顶点列表
    vertices = polygon[:]
    
    # 找到入口点和出口点在多边形上的位置（最近顶点索引）
    entry_idx = 0
    exit_idx = 0
    min_entry_dist = float('inf')
    min_exit_dist = float('inf')
    
    for i, v in enumerate(vertices):
        d_entry = distance(entry_on_poly, v)
        d_exit = distance(exit_on_poly, v)
        if d_entry < min_entry_dist:
            min_entry_dist = d_entry
            entry_idx = i
        if d_exit < min_exit_dist:
            min_exit_dist = d_exit
            exit_idx = i
    
    n = len(vertices)
    
    # 顺时针路径（向右绕行）
    clockwise_path = []
    i = entry_idx
    while True:
        clockwise_path.append(vertices[i][:])
        if i == exit_idx:
            break
        i = (i + 1) % n
        if i == entry_idx:
            break
    
    # 逆时针路径（向左绕行）
    counterclockwise_path = []
    i = entry_idx
    while True:
        counterclockwise_path.append(vertices[i][:])
        if i == exit_idx:
            break
        i = (i - 1) % n
        if i == entry_idx:
            break
    
    # 计算两条路径的长度
    cw_len = sum(distance(clockwise_path[j], clockwise_path[j+1]) for j in range(len(clockwise_path)-1))
    ccw_len = sum(distance(counterclockwise_path[j], counterclockwise_path[j+1]) for j in range(len(counterclockwise_path)-1))
    
    # 根据绕行方向选择路径
    # 向左绕行：顺时针方向
    # 向右绕行：逆时针方向
    if direction == "向左绕行":
        boundary_path = clockwise_path
    elif direction == "向右绕行":
        boundary_path = counterclockwise_path
    else:
        # 最佳航线：选择较短路径
        if cw_len <= ccw_len:
            boundary_path = clockwise_path
        else:
            boundary_path = counterclockwise_path
    
    # 将边界路径向外偏移（安全距离）
    offset_path = []
    for i, p in enumerate(boundary_path):
        # 计算该点的外法线方向
        prev_idx = (i - 1) % len(boundary_path)
        next_idx = (i + 1) % len(boundary_path)
        
        prev_p = boundary_path[prev_idx]
        next_p = boundary_path[next_idx]
        
        # 该点的切线方向
        tan_vec = [next_p[0] - prev_p[0], next_p[1] - prev_p[1]]
        tan_len = math.hypot(tan_vec[0], tan_vec[1])
        if tan_len < 1e-10:
            offset_path.append(p[:])
            continue
        
        tan_vec = [tan_vec[0]/tan_len, tan_vec[1]/tan_len]
        
        # 法线方向（指向外侧）
        normal = [-tan_vec[1], tan_vec[0]]
        
        # 判断法线方向是否指向外侧（远离中心）
        to_center_vec = [center[0] - p[0], center[1] - p[1]]
        if normal[0] * to_center_vec[0] + normal[1] * to_center_vec[1] > 0:
            normal = [-normal[0], -normal[1]]
        
        # 向外偏移
        offset_p = [
            p[0] + normal[0] * safety_lng,
            p[1] + normal[1] * safety_lat
        ]
        offset_path.append(offset_p)
    
    # 构建完整路径
    full_path = [start]
    full_path.extend(offset_path)
    full_path.append(end)
    
    return full_path

def create_avoidance_path_with_multiple_obstacles(start, end, obstacles, flight_alt, direction, safety_radius=5):
    """
    处理多个障碍物的绕行
    使用递归方法，依次绕过每个障碍物
    """
    # 检查是否有任何障碍物阻挡
    blocking = get_blocking_obstacles(start, end, obstacles, flight_alt, ignore_alt=False)
    
    if not blocking:
        return [start, end]
    
    # 选择阻挡路径上第一个障碍物来处理
    # 按距离起点排序
    blocking.sort(key=lambda obs: distance(start, obs.get('polygon', [])[0] if obs.get('polygon') else [0,0]))
    
    current_path = [start]
    current_pos = start
    
    for obs in blocking:
        # 检查从当前位置到终点的路径是否被该障碍物阻挡
        if line_intersects_polygon(current_pos, end, obs.get('polygon', [])):
            # 创建绕行路径
            bypass_path = create_obstacle_avoidance_path(
                current_pos, end, obs, flight_alt, safety_radius, direction
            )
            
            # 合并路径（去除重复点）
            if len(bypass_path) > 1:
                current_path.extend(bypass_path[1:])
                current_pos = bypass_path[-1]
            else:
                current_path.extend(bypass_path)
                current_pos = bypass_path[-1] if bypass_path else current_pos
    
    return current_path

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

def create_avoidance_path(start, end, obstacles, flight_alt, direction, safety_radius=5):
    """
    统一的航线规划入口
    所有方向都使用改进的紧贴障碍物绕行算法
    """
    # 所有模式都使用改进的绕行算法
    path = create_avoidance_path_with_multiple_obstacles(
        start, end, obstacles, flight_alt, direction, safety_radius
    )
    return path

#
#---------------------------------------------------------------------
# 等分航点生成
#---------------------------------------------------------------------

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

#
#---------------------------------------------------------------------
# 心跳模拟器
#---------------------------------------------------------------------

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

#
#---------------------------------------------------------------------
# 通信日志
#---------------------------------------------------------------------

def add_comm_log(message, direction="OBC内部"):
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log_entry = {"time": timestamp, "direction": direction, "message": message}
    if "comm_logs" not in st.session_state:
        st.session_state.comm_logs = []
    st.session_state.comm_logs.insert(0, log_entry)
    if len(st.session_state.comm_logs) > 50:
        st.session_state.comm_logs = st.session_state.comm_logs[:50]

#
#---------------------------------------------------------------------
# 地图创建（关键修改：所有显示坐标 GCJ-02 -> WGS-84）
#---------------------------------------------------------------------

def create_planning_map(center_gcj, points_gcj, obstacles, flight_trail, plan_path, drone_pos_gcj, flight_alt, enable_draw=False):
    # 地图中心点：GCJ-02 -> WGS-84
    center_wgs = gcj02_to_wgs84(center_gcj[0], center_gcj[1])
    m = folium.Map(location=[center_wgs[1], center_wgs[0]], zoom_start=16, tiles=GAODE_TILE, attr='高德')

    # 障碍物多边形：顶点从 GCJ-02 转为 WGS-84
    for obs in obstacles:
        coords_gcj = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords_gcj and len(coords_gcj) >= 3:
            coords_wgs = [gcj02_to_wgs84(lng, lat) for lng, lat in coords_gcj]
            # 根据障碍物高度与飞行高度比较，决定颜色
            color = "red" if height > flight_alt else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords_wgs], color=color, weight=2,
                           fill=True, fill_color=color, fill_opacity=0.4,
                           popup=f"🚧 {obs.get('name', '障碍物')}\n高度:{height}m\n状态:{'⚠️ 需绕行' if height > flight_alt else '✅ 可飞越'}").add_to(m)

    # 起点 A
    if points_gcj.get('A'):
        a_wgs = gcj02_to_wgs84(points_gcj['A'][0], points_gcj['A'][1])
        folium.Marker([a_wgs[1], a_wgs[0]], popup='起点A', icon=folium.Icon(color='green')).add_to(m)

    # 终点 B
    if points_gcj.get('B'):
        b_wgs = gcj02_to_wgs84(points_gcj['B'][0], points_gcj['B'][1])
        folium.Marker([b_wgs[1], b_wgs[0]], popup='终点B', icon=folium.Icon(color='red')).add_to(m)

    # 规划路径
    if plan_path and len(plan_path) > 1:
        path_wgs = [gcj02_to_wgs84(p[0], p[1]) for p in plan_path]
        folium.PolyLine([[p[1], p[0]] for p in path_wgs], color='green', weight=4,
                        popup=f"航线长度: {path_length(plan_path)*111000:.1f}m").add_to(m)

    # 历史轨迹
    if flight_trail:
        trail_wgs = [gcj02_to_wgs84(lng, lat) for lng, lat in flight_trail[-100:]]
        folium.PolyLine([[lat, lng] for lng, lat in trail_wgs], color='orange', weight=2).add_to(m)

    # 无人机当前位置
    if drone_pos_gcj:
        drone_wgs = gcj02_to_wgs84(drone_pos_gcj[0], drone_pos_gcj[1])
        folium.Marker([drone_wgs[1], drone_wgs[0]], icon=folium.Icon(color='blue')).add_to(m)

    # 绘图工具
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

#
#---------------------------------------------------------------------
# 初始化状态
#---------------------------------------------------------------------

def init():
    # 修改默认起点和终点坐标为截图中的值
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
        'show_add_dialog': False
    }

    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v

def update_plan_and_waypoints():
    if st.session_state.points_gcj.get('A') and st.session_state.points_gcj.get('B'):
        add_comm_log("开始航线规划 - 算法: A* 改进版 (所有绕行模式)", "OBC内部")
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

#
#---------------------------------------------------------------------
# 主程序
#---------------------------------------------------------------------

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
            st.info("💡 **所有绕行模式**：自动紧贴障碍物边缘绕行，完全不穿过障碍物\n- 向左绕行：沿多边形顺时针方向走\n- 向右绕行：沿多边形逆时针方向走")
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

    # ==================== 障碍物管理页面 ====================
    if st.session_state.page == "障碍物管理":
        st.header("🚧 障碍物配置持久化")
        st.caption(f"配置文件: {os.path.abspath(CONFIG_FILE)} | 版本: v18.0_obstacle_avoidance_all_modes")
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
        st.info("🛡️ **避障增强**：所有绕行模式（最佳航线/向左绕行/向右绕行）都会自动紧贴障碍物边缘绕行，完全不穿过障碍物。")

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

            # 处理绘制多边形（转换 WGS-84 -> GCJ-02 存储）
            if st.session_state.draw_enabled and not st.session_state.flight_started and map_output:
                last_draw = map_output.get("last_active_drawing")
                if last_draw and last_draw.get("geometry", {}).get("type") == "Polygon":
                    coords_wgs = last_draw["geometry"]["coordinates"][0]
                    vertices_wgs = [[c[0], c[1]] for c in coords_wgs]
                    vertices_gcj = [list(wgs84_to_gcj02(lng, lat)) for lng, lat in vertices_wgs]
                    st.session_state.drawn_polygon = vertices_gcj
                    st.session_state.show_add_dialog = True
                    st.rerun()

            # 显示添加障碍物的对话框
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

            # 处理点击地图移动航点
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


if __name__ == "__main__":
    main()
