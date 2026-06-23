import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
from folium.plugins import Draw
import random
import time
import math
import json
import os
from datetime import datetime
import pandas as pd

# ==================== 配置常量 ====================
SCHOOL_CENTER = [118.7490, 32.2340]
A_DFT = [118.746956, 32.232945]
B_DFT = [118.751589, 32.235204]
SAT_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
VEC_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
ATTR = "高德地图"

CONFIG_FILE = "obstacle_config.json"
BASE_SPEED_MPS = 5.0
HEARTBEAT_INTERVAL = 0.2

# ==================== 坐标转换 ====================
def gcj2wgs(lng, lat):
    if abs(lng) < 72 or abs(lng) > 138 or abs(lat) < 0.8 or abs(lat) > 56: return lng, lat
    dlat = -100 + 2*lng + 3*lat + 0.2*lat*lat + 0.1*lng*lat + 0.2*math.sqrt(abs(lng))
    dlat += (20*math.sin(6*lng*math.pi)+20*math.sin(2*lng*math.pi))*2/3
    dlat += (20*math.sin(lat*math.pi)+40*math.sin(lat/3*math.pi))*2/3
    dlat += (160*math.sin(lat/12*math.pi)+320*math.sin(lat*math.pi/30))*2/3
    dlng = 300 + lng + 2*lat + 0.1*lng*lng + 0.1*lng*lat + 0.1*math.sqrt(abs(lng))
    dlng += (20*math.sin(6*lng*math.pi)+20*math.sin(2*lng*math.pi))*2/3
    dlng += (20*math.sin(lng*math.pi)+40*math.sin(lng/3*math.pi))*2/3
    dlng += (150*math.sin(lng/12*math.pi)+300*math.sin(lng/30*math.pi))*2/3
    rad = lat/180*math.pi
    magic = 1 - 0.00669342162296594323 * math.sin(rad)**2
    sqrtmagic = math.sqrt(magic)
    dlat = dlat * 180 / ((6378245.0*(1-0.00669342162296594323))/(magic*sqrtmagic)*math.pi)
    dlng = dlng * 180 / (6378245.0/sqrtmagic*math.cos(rad)*math.pi)
    return lng-dlng, lat-dlat

def wgs2gcj(lng, lat):
    if abs(lng) < 72 or abs(lng) > 138 or abs(lat) < 0.8 or abs(lat) > 56: return lng, lat
    dlat = -100 + 2*lng + 3*lat + 0.2*lat*lat + 0.1*lng*lat + 0.2*math.sqrt(abs(lng))
    dlat += (20*math.sin(6*lng*math.pi)+20*math.sin(2*lng*math.pi))*2/3
    dlat += (20*math.sin(lat*math.pi)+40*math.sin(lat/3*math.pi))*2/3
    dlat += (160*math.sin(lat/12*math.pi)+320*math.sin(lat*math.pi/30))*2/3
    dlng = 300 + lng + 2*lat + 0.1*lng*lng + 0.1*lng*lat + 0.1*math.sqrt(abs(lng))
    dlng += (20*math.sin(6*lng*math.pi)+20*math.sin(2*lng*math.pi))*2/3
    dlng += (20*math.sin(lng*math.pi)+40*math.sin(lng/3*math.pi))*2/3
    dlng += (150*math.sin(lng/12*math.pi)+300*math.sin(lng/30*math.pi))*2/3
    rad = lat/180*math.pi
    magic = 1 - 0.00669342162296594323 * math.sin(rad)**2
    sqrtmagic = math.sqrt(magic)
    dlat = dlat * 180 / ((6378245.0*(1-0.00669342162296594323))/(magic*sqrtmagic)*math.pi)
    dlng = dlng * 180 / (6378245.0/sqrtmagic*math.cos(rad)*math.pi)
    return lng+dlng, lat+dlat

# ==================== 几何辅助 ====================
def dist(p1, p2): return math.hypot(p1[0]-p2[0], p1[1]-p2[1])
def point_in_poly(p, poly):
    x,y = p; inside=False
    for i in range(len(poly)):
        x1,y1 = poly[i]; x2,y2 = poly[(i+1)%len(poly)]
        if ((y1>y)!=(y2>y)) and (x<(x2-x1)*(y-y1)/(y2-y1)+x1): inside=not inside
    return inside
def lines_intersect(a,b,c,d):
    def ccw(A,B,C): return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
    return ccw(a,c,d)!=ccw(b,c,d) and ccw(a,b,c)!=ccw(a,b,d)
def line_cross_poly(p1,p2,poly):
    if point_in_poly(p1,poly) or point_in_poly(p2,poly): return True
    for i in range(len(poly)):
        if lines_intersect(p1,p2,poly[i],poly[(i+1)%len(poly)]): return True
    return False

def seg_to_poly_dist(p1, p2, poly):
    min_d = float('inf')
    for pt in poly:
        t = ((pt[0]-p1[0])*(p2[0]-p1[0]) + (pt[1]-p1[1])*(p2[1]-p1[1])) / (dist(p1,p2)**2+1e-9)
        t = max(0,min(1,t))
        proj = (p1[0]+t*(p2[0]-p1[0]), p1[1]+t*(p2[1]-p1[1]))
        d = dist(pt,proj)
        if d < min_d: min_d = d
    for i in range(len(poly)):
        p3,p4 = poly[i], poly[(i+1)%len(poly)]
        for t in range(11):
            pt = (p3[0]+(p4[0]-p3[0])*t/10, p3[1]+(p4[1]-p3[1])*t/10)
            d = dist(pt, (p1[0],p1[1]))
            if d < min_d: min_d = d
    return min_d * 111000

def should_avoid(obs, h): return h <= obs.get('height',20)

def path_safe(p1,p2,obs,rad_m,h):
    for o in obs:
        if not should_avoid(o,h): continue
        poly = o.get('polygon',[])
        if len(poly)<3: continue
        if line_cross_poly(p1,p2,poly): return False
        if seg_to_poly_dist(p1,p2,poly) < rad_m-0.1: return False
    return True

# ==================== 绕行生成 ====================
def gen_bypass(A,B,obs,rad_m,h,side='left'):
    avoid = [o for o in obs if should_avoid(o,h)]
    if not avoid: return [A,B]
    mx,my = (A[0]+B[0])/2, (A[1]+B[1])/2
    dx,dy = B[0]-A[0], B[1]-A[1]
    L = math.hypot(dx,dy)
    if L==0: return [A,B]
    ux,uy = dx/L, dy/L
    px,py = -uy, ux
    if side=='right': px,py = uy,-ux
    deg_m = 1/111000
    for attempt in range(1,31):
        off_m = rad_m*2*attempt
        off_deg = off_m*deg_m
        wp = (mx+px*off_deg, my+py*off_deg)
        if path_safe(A,wp,avoid,rad_m,h) and path_safe(wp,B,avoid,rad_m,h):
            return [A,wp,B]
    pts = [p for o in avoid for p in o.get('polygon',[])]
    if pts:
        cx = sum(p[0] for p in pts)/len(pts); cy = sum(p[1] for p in pts)/len(pts)
        far = max(pts, key=lambda p: dist((cx,cy),p))
        dx,dy = far[0]-cx, far[1]-cy
        L2 = math.hypot(dx,dy)
        if L2>0: dx,dy = dx/L2, dy/L2
        else: dx,dy = 1,0
        wp = (far[0]+dx*rad_m*15*deg_m, far[1]+dy*rad_m*15*deg_m)
        return [A,wp,B]
    return [A,B]

def plan_single_segment(A,B,obs,h,rad,strat):
    avoid = [o for o in obs if should_avoid(o,h)]
    straight = not any(line_cross_poly(A,B,o['polygon']) for o in avoid)
    if straight: return [A,B]
    if strat in ('left','right'):
        return gen_bypass(A,B,obs,rad,h,strat)
    else:
        left=gen_bypass(A,B,obs,rad,h,'left')
        right=gen_bypass(A,B,obs,rad,h,'right')
        if left and right:
            len_left = sum(dist(left[i],left[i+1]) for i in range(len(left)-1))
            len_right = sum(dist(right[i],right[i+1]) for i in range(len(right)-1))
            return left if len_left <= len_right else right
        return left or right or [A,B]

def plan_full_path(waypoints, obs, h, rad, strat):
    full = []
    for i in range(len(waypoints)-1):
        seg = plan_single_segment(waypoints[i], waypoints[i+1], obs, h, rad, strat)
        if i == 0:
            full.extend(seg)
        else:
            full.extend(seg[1:])
    return full

# ==================== 辅助函数 ====================
def point_to_seg_meters(p, a, b):
    ap = (p[0]-a[0], p[1]-a[1])
    ab = (b[0]-a[0], b[1]-a[1])
    t = (ap[0]*ab[0] + ap[1]*ab[1]) / (ab[0]*ab[0] + ab[1]*ab[1] + 1e-9)
    t = max(0, min(1, t))
    proj = (a[0] + t*ab[0], a[1] + t*ab[1])
    return math.hypot(p[0]-proj[0], p[1]-proj[1]) * 111000

def check_safety_radius(drone_pos, obstacles, flight_alt, safe_radius):
    if not drone_pos:
        return True, None, None
    min_dist = float('inf')
    danger_name = None
    for obs in obstacles:
        if obs.get('height',20) > flight_alt:
            poly = obs.get('polygon',[])
            if poly:
                for i in range(len(poly)):
                    p1 = poly[i]; p2 = poly[(i+1)%len(poly)]
                    d = point_to_seg_meters(drone_pos, p1, p2)
                    if d < min_dist:
                        min_dist = d
                        danger_name = obs.get('name','障碍物')
    if min_dist < safe_radius:
        return False, min_dist, danger_name
    return True, min_dist if min_dist!=float('inf') else None, None

# ==================== 心跳模拟器 ====================
class HeartbeatSim:
    def __init__(self,start):
        self.hist = []
        self.pos = start[:]
        self.path = [start[:]]
        self.idx = 0
        self.sim = False
        self.pause = False
        self.alt = 50
        self.spd = 50
        self.prog = 0
        self.total = 0
        self.trav = 0
        self.start_time = None
        self.elapsed = 0
        self.safety_violation = False
    def set_path(self,path,alt,spd):
        self.path = path
        self.idx = 0
        self.pos = path[0][:]
        self.alt = alt
        self.spd = spd
        self.sim = True
        self.pause = False
        self.prog = 0
        self.trav = 0
        self.total = sum(dist(path[i],path[i+1]) for i in range(len(path)-1))
        self.start_time = time.time()
        self.elapsed = 0
        self.safety_violation = False
    def reset(self):
        if self.path:
            self.pos = self.path[0][:]
        self.idx = 0
        self.sim = False
        self.pause = False
        self.prog = 0
        self.trav = 0
        self.start_time = None
        self.elapsed = 0
        self.safety_violation = False
    def pause(self): self.pause = True
    def resume(self): self.pause = False
    def stop(self): self.sim = False; self.pause = False; self.start_time = None
    def update(self, obstacles_gcj, safe_radius):
        if not self.sim or self.pause:
            return self._hb(obstacles_gcj, safe_radius)
        if self.start_time:
            self.elapsed = time.time() - self.start_time
        if self.idx < len(self.path)-1:
            tar = self.path[self.idx+1]
            dx,dy = tar[0]-self.pos[0], tar[1]-self.pos[1]
            d2t = math.hypot(dx,dy)
            speed_mps = 0.5 + (self.spd/100)*4.5
            step_m = speed_mps * HEARTBEAT_INTERVAL
            step_deg = step_m / 111000.0
            if d2t < step_deg:
                self.trav += d2t
                self.pos = tar[:]
                self.idx += 1
            else:
                r = step_deg / d2t
                self.pos[0] += dx * r
                self.pos[1] += dy * r
                self.trav += step_deg
            if self.total > 0:
                self.prog = min(1.0, self.trav / self.total)
            if self.idx >= len(self.path)-1:
                self.sim = False
                self.prog = 1.0
        else:
            self.sim = False
            self.prog = 1.0
        return self._hb(obstacles_gcj, safe_radius)
    def _hb(self, obstacles_gcj, safe_radius):
        speed = round(0.5 + (self.spd/100)*4.5,1) if self.sim and not self.pause else 0
        if self.sim and not self.pause:
            remaining_in_path = 0.0
            if self.idx < len(self.path)-1:
                remaining_in_path += dist(self.pos, self.path[self.idx+1])
                for i in range(self.idx+1, len(self.path)-1):
                    remaining_in_path += dist(self.path[i], self.path[i+1])
            remaining_dist = remaining_in_path * 111000
        else:
            remaining_dist = max(0, self.total - self.trav) * 111000
        safe, min_d, danger = check_safety_radius(self.pos, obstacles_gcj, self.alt, safe_radius)
        self.safety_violation = not safe
        battery = max(0, 100 - int(self.prog * 100))
        if speed > 0 and remaining_dist > 0:
            eta_sec = remaining_dist / speed
            if eta_sec < 60:
                remain_str = f"{eta_sec:.0f}秒"
            else:
                minutes = int(eta_sec // 60)
                seconds = int(eta_sec % 60)
                remain_str = f"{minutes:02d}:{seconds:02d}"
        else:
            remain_str = "00:00"
        voltage = 22.2 + random.uniform(-0.5,0.5)
        satellites = random.randint(8,14)
        delay = round(random.uniform(10,50),1) if self.sim else 0
        loss = round(random.uniform(0,0.2),1) if self.sim else 0
        arrived = not self.sim and self.prog >= 1.0
        return {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.pos[0], "lat": self.pos[1],
            "altitude": self.alt + random.randint(-5,5) if self.sim else random.randint(0,10),
            "speed": speed,
            "progress": self.prog,
            "total": self.total,
            "traveled": self.trav,
            "current_wp": f"{self.idx+1}/{len(self.path)}",
            "remain": remain_str,
            "battery": battery,
            "elapsed": self.elapsed,
            "delay_ms": delay,
            "loss_percent": loss,
            "simulating": self.sim,
            "paused": self.pause,
            "flight_time": self.elapsed,
            "voltage": voltage,
            "satellites": satellites,
            "arrived": arrived,
            "safety_violation": self.safety_violation,
            "remaining_distance": remaining_dist
        }

# ==================== 障碍物缓存 ====================
def save_cache():
    if 'saved' not in st.session_state: st.session_state.saved = []
    import copy
    st.session_state.saved = copy.deepcopy(st.session_state.obs)
    st.success(f"保存 {len(st.session_state.obs)} 个障碍物")
def load_cache():
    if 'saved' in st.session_state and st.session_state.saved:
        st.session_state.obs = st.session_state.saved
        st.success(f"加载 {len(st.session_state.obs)} 个障碍物")
        return True
    st.warning("无缓存")
    return False

# ==================== 安全半径可视化 ====================
def add_safety(m, obs, rad, h):
    for o in obs:
        if should_avoid(o,h):
            for pt in o.get('polygon',[]):
                folium.Circle([pt[1],pt[0]], rad, color='orange', fill=True, fill_opacity=0.2, popup=f"安全区{rad}m").add_to(m)

# ==================== 地图生成 ====================
def make_map(center, waypoints, obs, hist, full_path, maptype, rad, h, drone_pos=None):
    tiles = SAT_URL if maptype=='satellite' else VEC_URL
    m = folium.Map(location=[center[1],center[0]], zoom_start=16, tiles=tiles, attr=ATTR)
    Draw(export=True, draw_options={'polygon':{'allowIntersection':False,'showArea':True}}).add_to(m)
    add_safety(m, obs, rad, h)
    for i,o in enumerate(obs):
        coords=o.get('polygon',[])
        if len(coords)>=3:
            color = 'red' if o.get('height',20) > h else 'orange'
            folium.Polygon([[c[1],c[0]] for c in coords], color=color, weight=3, fill=True, fill_opacity=0.4,
                           popup=f"{o.get('name',f'障碍物{i+1}')}\n高度:{o.get('height',20)}m").add_to(m)
    for idx, wp in enumerate(waypoints):
        color = 'green' if idx==0 else ('red' if idx==len(waypoints)-1 else 'blue')
        folium.Marker([wp[1],wp[0]], popup=f"航点{idx+1}", icon=folium.Icon(color=color)).add_to(m)
    if full_path and len(full_path)>1:
        folium.PolyLine([[p[1],p[0]] for p in full_path], color='green', weight=5, opacity=0.9, popup="完整避障航线").add_to(m)
        for p in full_path[1:-1]: folium.CircleMarker([p[1],p[0]], 3, color='green', fill=True).add_to(m)
    if len(waypoints) > 1:
        straight_line = [[wp[1], wp[0]] for wp in waypoints]
        folium.PolyLine(straight_line, color='gray', weight=2, dash_array='5,5', popup="航点连线").add_to(m)
    if hist:
        trail = [[p[1],p[0]] for p in hist[-30:] if len(p)==2]
        if len(trail)>1: folium.PolyLine(trail, color='orange', weight=2).add_to(m)
    if drone_pos:
        folium.Circle([drone_pos[1],drone_pos[0]], rad, color='blue', fill=True, fill_opacity=0.2, popup="安全区").add_to(m)
        folium.Marker([drone_pos[1],drone_pos[0]], popup="无人机", icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(m)
    return m

# ==================== 主程序 ====================
def main():
    st.set_page_config(layout="wide")
    st.title("🏫 无人机地面站系统 - 航点飞行（最终版）")

    # 初始化状态
    if 'waypoints' not in st.session_state: st.session_state.waypoints = [A_DFT[:], B_DFT[:]]
    if 'obs' not in st.session_state: st.session_state.obs = []
    if 'hb' not in st.session_state: st.session_state.hb = HeartbeatSim(st.session_state.waypoints[0][:])
    if 'last_time' not in st.session_state: st.session_state.last_time = time.time()
    if 'running' not in st.session_state: st.session_state.running = False
    if 'alt' not in st.session_state: st.session_state.alt = 50
    if 'hist' not in st.session_state: st.session_state.hist = []
    if 'full_path' not in st.session_state: st.session_state.full_path = None
    if 'pending_poly' not in st.session_state: st.session_state.pending_poly = None
    if 'pending_h' not in st.session_state: st.session_state.pending_h = 20
    if 'drone_spd' not in st.session_state: st.session_state.drone_spd = 50
    if 'safe_rad' not in st.session_state: st.session_state.safe_rad = 5
    if 'sel_strat' not in st.session_state: st.session_state.sel_strat = 'best'
    if 'new_wp_lng' not in st.session_state: st.session_state.new_wp_lng = A_DFT[0]
    if 'new_wp_lat' not in st.session_state: st.session_state.new_wp_lat = A_DFT[1]

    with st.sidebar:
        st.header("控制面板")
        page = st.radio("模块", ["规划","监控","障碍物"])
        map_type = "satellite" if st.radio("地图", ["卫星影像","矢量街道"]) == "卫星影像" else "vector"
        st.markdown("---")
        st.subheader("无人机参数")
        st.session_state.drone_spd = st.slider("速度系数",10,100,st.session_state.drone_spd)
        st.session_state.safe_rad = st.number_input("安全半径(米)",1,30,st.session_state.safe_rad)
        st.session_state.alt = st.number_input("飞行高度(米)",0,200,st.session_state.alt)
        st.markdown("---")
        st.subheader("绕行策略")
        strat = st.radio("避障方式",["最佳航线","向左绕行","向右绕行"])
        strat_map = {"最佳航线":"best","向左绕行":"left","向右绕行":"right"}
        st.session_state.sel_strat = strat_map[strat]
        st.info(f"障碍物: {len(st.session_state.obs)}")
        if st.button("刷新规划", use_container_width=True):
            with st.spinner("规划全航线中..."):
                st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                            st.session_state.obs,
                                                            st.session_state.alt,
                                                            st.session_state.safe_rad,
                                                            st.session_state.sel_strat)
            st.rerun()

    if page == "规划":
        st.header("航线规划 - 多航点避障")
        st.info("📝 点击地图📐画多边形→设置高度→「添加障碍物」；下方可添加/删除航点（起点和终点固定）")
        col1,col2 = st.columns([1,1.5])
        with col1:
            st.markdown("#### 🗺️ 航点管理")
            # 起点
            st.markdown("**起点**")
            col_s = st.columns(2)
            with col_s[0]:
                a_lat = st.number_input("纬度", value=st.session_state.waypoints[0][1], format="%.6f", key="a_lat")
            with col_s[1]:
                a_lng = st.number_input("经度", value=st.session_state.waypoints[0][0], format="%.6f", key="a_lng")
            if st.button("更新起点"):
                st.session_state.waypoints[0] = [a_lng, a_lat]
                st.rerun()
            # 中间航点
            st.markdown("**中间航点**")
            if len(st.session_state.waypoints) > 2:
                for i in range(1, len(st.session_state.waypoints)-1):
                    col_wp = st.columns([3,1])
                    col_wp[0].write(f"航点{i}: ({st.session_state.waypoints[i][0]:.6f}, {st.session_state.waypoints[i][1]:.6f})")
                    if col_wp[1].button("删除", key=f"del_wp_{i}"):
                        st.session_state.waypoints.pop(i)
                        st.rerun()
            else:
                st.write("暂无中间航点")
            # 添加新航点
            st.markdown("**添加新航点**")
            col_add = st.columns(2)
            with col_add[0]:
                new_lng = st.number_input("经度", value=st.session_state.new_wp_lng, format="%.6f", key="new_lng")
            with col_add[1]:
                new_lat = st.number_input("纬度", value=st.session_state.new_wp_lat, format="%.6f", key="new_lat")
            if st.button("➕ 添加航点"):
                st.session_state.waypoints.insert(-1, [new_lng, new_lat])
                st.rerun()
            # 终点
            st.markdown("**终点**")
            col_e = st.columns(2)
            with col_e[0]:
                b_lat = st.number_input("纬度", value=st.session_state.waypoints[-1][1], format="%.6f", key="b_lat")
            with col_e[1]:
                b_lng = st.number_input("经度", value=st.session_state.waypoints[-1][0], format="%.6f", key="b_lng")
            if st.button("更新终点"):
                st.session_state.waypoints[-1] = [b_lng, b_lat]
                st.rerun()
            st.markdown("---")
            # 障碍物添加
            st.markdown("#### 🏗️ 新障碍物高度")
            st.session_state.pending_h = st.number_input("高度(米)",1,200,st.session_state.pending_h)
            if st.button("➕ 添加障碍物"):
                if st.session_state.pending_poly and len(st.session_state.pending_poly)>=3:
                    st.session_state.obs.append({"name":f"建筑物{len(st.session_state.obs)+1}",
                                                 "polygon":st.session_state.pending_poly,
                                                 "height":st.session_state.pending_h})
                    st.success(f"已添加，共{len(st.session_state.obs)}个")
                    st.session_state.pending_poly = None
                    st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                                st.session_state.obs,
                                                                st.session_state.alt,
                                                                st.session_state.safe_rad,
                                                                st.session_state.sel_strat)
                    st.rerun()
                else: st.warning("请先在地图上画多边形")
            if st.button("🔄 重新规划路径"):
                with st.spinner("规划全航线中..."):
                    st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                                st.session_state.obs,
                                                                st.session_state.alt,
                                                                st.session_state.safe_rad,
                                                                st.session_state.sel_strat)
                st.rerun()
            st.markdown("#### ✈️ 飞行控制")
            if st.button("▶️ 开始飞行"):
                if st.session_state.full_path is None or len(st.session_state.full_path) < 2:
                    st.warning("请先点击「刷新规划」生成完整路径")
                else:
                    st.session_state.hb.set_path(st.session_state.full_path, st.session_state.alt, st.session_state.drone_spd)
                    st.session_state.running = True
                    st.session_state.hist = []
                    st.success("飞行开始，请切换至「监控」页面")
            if st.button("⏹️ 停止飞行"):
                st.session_state.running = False
                st.session_state.hb.stop()
            st.caption(f"航线共{len(st.session_state.waypoints)}个航点")
            if st.session_state.full_path:
                st.caption(f"完整路径含{len(st.session_state.full_path)}个航段点")
        with col2:
            center = st.session_state.waypoints[0] or SCHOOL_CENTER
            if st.session_state.full_path is None:
                st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                            st.session_state.obs,
                                                            st.session_state.alt,
                                                            st.session_state.safe_rad,
                                                            st.session_state.sel_strat)
            drone_pos = st.session_state.hb.pos if st.session_state.running else None
            m = make_map(center, st.session_state.waypoints, st.session_state.obs, st.session_state.hist,
                         st.session_state.full_path, map_type,
                         st.session_state.safe_rad, st.session_state.alt, drone_pos)
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
            if output and output.get("last_active_drawing"):
                d = output["last_active_drawing"]
                if d and d.get("geometry",{}).get("type")=="Polygon":
                    coords = d["geometry"]["coordinates"][0]
                    if len(coords)>=3:
                        st.session_state.pending_poly = [[p[0],p[1]] for p in coords]
                        st.success("已捕获多边形，请设置高度后点「添加障碍物」")
        st.caption("图例：绿色=避障航线 红色=障碍物 橙色=安全区 | 蓝色旗帜=中间航点")

    elif page == "监控":
        st.header("📡 飞行实时画面 - 任务执行监控")
        # 控制按钮行
        col_btn = st.columns(4)
        with col_btn[0]:
            if st.button("▶️ 开始任务", use_container_width=True):
                if not st.session_state.running:
                    if st.session_state.full_path is None:
                        st.warning("请先在规划页面刷新规划路径")
                    else:
                        st.session_state.hb.set_path(st.session_state.full_path, st.session_state.alt, st.session_state.drone_spd)
                        st.session_state.running = True
                        st.rerun()
                else:
                    st.session_state.hb.resume()
                    st.rerun()
        with col_btn[1]:
            if st.button("⏸️ 暂停", use_container_width=True):
                if st.session_state.running:
                    st.session_state.hb.pause()
                    st.rerun()
                else:
                    st.warning("当前没有飞行任务")
        with col_btn[2]:
            if st.button("⏹️ 停止", use_container_width=True):
                st.session_state.running = False
                st.session_state.hb.stop()
                st.rerun()
        with col_btn[3]:
            if st.button("🔄 重置", use_container_width=True):
                st.session_state.running = False
                st.session_state.hb.reset()
                st.session_state.hist = []
                st.rerun()
        st.markdown("---")
        # 自动刷新（每0.2秒）
        if st.session_state.running and time.time() - st.session_state.last_time >= HEARTBEAT_INTERVAL:
            new_hb = st.session_state.hb.update(st.session_state.obs, st.session_state.safe_rad)
            st.session_state.last_time = time.time()
            st.session_state.hist.append([new_hb['lng'], new_hb['lat']])
            if len(st.session_state.hist) > 200:
                st.session_state.hist.pop(0)
            if new_hb['arrived']:
                st.session_state.running = False
                st.success("🏁 无人机已安全到达目的地！")
            st.rerun()
        # 获取最新心跳数据
        if st.session_state.hb.hist:
            d = st.session_state.hb.hist[0]
        else:
            d = {"current_wp":"0/0","speed":0,"elapsed":0,"total":0,"traveled":0,"remain":"00:00","battery":0,"progress":0,"delay_ms":0,"loss_percent":0,
                 "flight_time":0,"voltage":22.2,"satellites":0,"arrived":False,"safety_violation":False,"remaining_distance":0}
        total_waypoints = len(st.session_state.waypoints)
        if total_waypoints > 0 and 'progress' in d:
            segment_index = int(d['progress'] * (total_waypoints - 1))
            current_wp_num = segment_index + 1
            current_wp_num = min(current_wp_num, total_waypoints)
        else:
            current_wp_num = 0
        st.markdown("### ✈️ 飞行进度")
        st.progress(d.get('progress',0), text=f"任务进度: {d.get('progress',0)*100:.1f}%")
        st.markdown("### 📊 实时飞行数据")
        row1 = st.columns(4)
        row1[0].metric("🎯 当前航点", f"{current_wp_num}/{total_waypoints}" if total_waypoints>0 else "0/0")
        row1[1].metric("💨 飞行速度", f"{d.get('speed',0)} m/s")
        elapsed = d.get('elapsed',0)
        row1[2].metric("⏰ 已用时间", f"{int(elapsed//60):02d}:{int(elapsed%60):02d}")
        remaining = d.get('remaining_distance',0)
        row1[3].metric("📏 剩余距离", f"{remaining:.0f} m" if remaining>=0 else "0 m")
        row2 = st.columns(2)
        row2[0].metric("🕐 预计到达", d.get('remain','00:00'))
        battery = d.get('battery',0)
        row2[1].metric("🔋 电量模拟", f"{battery}%")
        st.markdown("---")
        # 设备状态与通信拓扑
        col_status, col_top = st.columns(2)
        with col_status:
            st.subheader("📡 设备状态")
            online = st.session_state.running
            st.markdown(f"- **GCS**：{'✅ 在线' if online else '❌ 离线'}")
            st.markdown(f"- **OBC**：{'✅ 在线' if online else '❌ 离线'}")
            st.markdown(f"- **FCU**：{'✅ 在线' if online else '❌ 离线'}")
        with col_top:
            st.subheader("🔗 通信链路拓扑与数据流")
            delay = d.get('delay_ms',0)
            loss = d.get('loss_percent',0)
            st.markdown(f"""
            - **GCS** ↔ **OBC**：延迟 {delay} ms  
            - **GCS** ↔ **FCU**：延迟 {delay+5} ms  
            - **OBC** ↔ **FCU**：延迟 ~{max(0,delay-2)} ms  
            - **丢包率**：{loss}%
            """)
            st.code("GCS → OBC → FCU → UAV")
            st.caption("数据流：遥控指令 → 飞控 → 执行器 | 遥测数据 ← 飞控 ← 传感器")
        st.markdown("---")
        # 实时飞行地图
        st.subheader("🗺️ 实时飞行地图")
        if st.session_state.hb.hist:
            latest = st.session_state.hb.hist[0]
            center = [latest['lat'], latest['lng']]
        elif st.session_state.waypoints:
            center = [st.session_state.waypoints[0][1], st.session_state.waypoints[0][0]]
        else:
            center = [SCHOOL_CENTER[1], SCHOOL_CENTER[0]]
        tiles = SAT_URL if map_type=="satellite" else VEC_URL
        m = folium.Map(location=center, zoom_start=17, tiles=tiles, attr=ATTR)
        add_safety(m, st.session_state.obs, st.session_state.safe_rad, st.session_state.alt)
        for o in st.session_state.obs:
            coords = o.get('polygon',[])
            if len(coords)>=3:
                folium.Polygon([[c[1],c[0]] for c in coords], color='red', fill=True, fill_opacity=0.3).add_to(m)
        if st.session_state.full_path:
            folium.PolyLine([[p[1],p[0]] for p in st.session_state.full_path], color='green', weight=3).add_to(m)
        if st.session_state.hb.hist:
            trail = [[h['lat'],h['lng']] for h in st.session_state.hb.hist[:30] if 'lat' in h]
            if len(trail)>1:
                folium.PolyLine(trail, color='orange', weight=2).add_to(m)
            latest = st.session_state.hb.hist[0]
            folium.Marker([latest['lat'], latest['lng']], popup=f"📍 当前位置\n高度:{latest['altitude']}m",
                          icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(m)
        for i,wp in enumerate(st.session_state.waypoints):
            color = 'green' if i==0 else ('red' if i==len(st.session_state.waypoints)-1 else 'blue')
            folium.Marker([wp[1], wp[0]], popup=f"航点{i+1}", icon=folium.Icon(color=color)).add_to(m)
        folium_static(m, width=1000, height=500)
        st.markdown("---")
        # 数据图表
        st.subheader("📈 实时数据图表")
        if len(st.session_state.hb.hist) > 1:
            col_ch1, col_ch2 = st.columns(2)
            with col_ch1:
                speed_data = [{"时间(s)": i*HEARTBEAT_INTERVAL, "速度(m/s)": h['speed']} for i,h in enumerate(st.session_state.hb.hist[:30])]
                st.line_chart(pd.DataFrame(speed_data), x="时间(s)", y="速度(m/s)")
                st.caption("速度变化趋势")
            with col_ch2:
                remain_data = [{"时间(s)": i*HEARTBEAT_INTERVAL, "剩余距离(m)": max(0,h['remaining_distance'])} for i,h in enumerate(st.session_state.hb.hist[:30])]
                st.line_chart(pd.DataFrame(remain_data), x="时间(s)", y="剩余距离(m)")
                st.caption("剩余距离变化趋势")
        else:
            st.info("等待飞行数据...")
        st.markdown("---")
        # 飞行日志记录
        st.subheader("📋 飞行日志记录")
        if st.session_state.hb.hist:
            log_df = pd.DataFrame([{
                "时间": h['timestamp'],
                "飞行时间(s)": f"{h['elapsed']:.1f}",
                "纬度": h['lat'],
                "经度": h['lng'],
                "高度(m)": h['altitude'],
                "速度(m/s)": h['speed'],
                "电压(V)": h['voltage'],
                "卫星数": h['satellites'],
                "剩余距离(m)": f"{h['remaining_distance']:.0f}",
                "进度": f"{h['progress']*100:.1f}%"
            } for h in st.session_state.hb.hist[:20]])
            st.dataframe(log_df, use_container_width=True)
            if st.button("📊 导出完整飞行数据", use_container_width=True):
                full_df = pd.DataFrame([{
                    "timestamp": h['timestamp'],
                    "flight_time_s": h['elapsed'],
                    "lat": h['lat'],
                    "lng": h['lng'],
                    "altitude_m": h['altitude'],
                    "speed_mps": h['speed'],
                    "voltage_v": h['voltage'],
                    "satellites": h['satellites'],
                    "remaining_distance_m": h['remaining_distance'],
                    "progress_pct": h['progress']*100,
                    "safety_violation": h['safety_violation']
                } for h in st.session_state.hb.hist])
                csv = full_df.to_csv(index=False)
                st.download_button("📥 下载CSV", csv, f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "text/csv", use_container_width=True)
        else:
            st.info("暂无飞行数据")

    elif page == "障碍物":
        st.header("障碍物管理")
        st.info(f"共 {len(st.session_state.obs)} 个障碍物")
        col1,col2 = st.columns([1,1.5])
        with col1:
            for i,o in enumerate(st.session_state.obs):
                c1,c2,c3 = st.columns([2,1,1])
                c1.write(f"🚧 {o.get('name',f'障碍物{i+1}')}")
                c2.write(f"高度:{o.get('height',20)}m")
                if c3.button("删除", key=f"del{i}"):
                    st.session_state.obs.pop(i)
                    st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                                st.session_state.obs,
                                                                st.session_state.alt,
                                                                st.session_state.safe_rad,
                                                                st.session_state.sel_strat)
                    st.rerun()
            if st.button("💾 保存到缓存"): save_cache()
            if st.button("📂 从缓存加载"):
                if load_cache():
                    st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                                st.session_state.obs,
                                                                st.session_state.alt,
                                                                st.session_state.safe_rad,
                                                                st.session_state.sel_strat)
                    st.rerun()
            if st.button("🗑️ 全部清除"):
                st.session_state.obs=[]
                st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                            st.session_state.obs,
                                                            st.session_state.alt,
                                                            st.session_state.safe_rad,
                                                            st.session_state.sel_strat)
                st.rerun()
        with col2:
            tiles = SAT_URL if map_type=="satellite" else VEC_URL
            m = folium.Map(location=[SCHOOL_CENTER[1],SCHOOL_CENTER[0]], zoom_start=16, tiles=tiles, attr=ATTR)
            for o in st.session_state.obs:
                coords=o.get('polygon',[])
                if len(coords)>=3:
                    folium.Polygon([[c[1],c[0]] for c in coords], color='red', weight=3, fill=True, fill_opacity=0.5, popup=f"高度:{o.get('height',20)}m").add_to(m)
            folium.Marker([A_DFT[1],A_DFT[0]], popup="起点", icon=folium.Icon(color='green')).add_to(m)
            folium.Marker([B_DFT[1],B_DFT[0]], popup="终点", icon=folium.Icon(color='red')).add_to(m)
            folium_static(m, width=700, height=500)

if __name__ == "__main__":
    main()
