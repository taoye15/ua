import streamlit as st
import folium
from streamlit_folium import st_folium
import math
from typing import List, Tuple

# ===================== GCJ-02 坐标转换核心算法 =====================
PI = 3.14159265358979323846

def wgs84_to_gcj02(lon: float, lat: float) -> Tuple[float, float]:
    """WGS84 转 GCJ-02 火星坐标"""
    dlat = _transform_lat(lon - 120.15, lat - 31.95)
    dlon = _transform_lon(lon - 120.15, lat - 31.95)
    rad_lat = lat / 180.0 * PI
    magic = math.sin(rad_lat)
    magic = 2 - magic * 0.0002 * math.cos(lon / 180.0 * PI)
    dlat = dlat * magic
    dlon = dlon * magic
    gcj_lat = lat + dlat
    gcj_lon = lon + dlon
    return round(gcj_lon, 6), round(gcj_lat, 6)

def gcj02_to_wgs84(lon: float, lat: float) -> Tuple[float, float]:
    """GCJ-02 转 WGS84"""
    dlat = _transform_lat(lon - 120.15, lat - 31.95)
    dlon = _transform_lon(lon - 120.15, lat - 31.95)
    rad_lat = lat / 180.0 * PI
    magic = math.sin(rad_lat)
    magic = 2 - magic * 0.0002 * math.cos(lon / 180.0 * PI)
    dlat = dlat * magic
    dlon = dlon * magic
    wgs_lat = lat - dlat
    wgs_lon = lon - dlon
    return round(wgs_lon, 6), round(wgs_lat, 6)

def _transform_lat(x: float, y: float) -> float:
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * PI) + 40.0 * math.sin(y / 3.0 * PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * PI) + 320 * math.sin(y * PI / 12.0)) * 2.0 / 3.0
    return ret

def _transform_lon(x: float, y: float) -> float:
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * PI) + 40.0 * math.sin(x / 3.0 * PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * PI) + 300.0 * math.sin(x * PI / 12.0)) * 2.0 / 3.0
    return ret

# ===================== 全局配置 & 初始化 =====================
# 南京科技职业学院 中心点 WGS84 坐标
NJPI_WGS_LON = 118.6723
NJPI_WGS_LAT = 32.2256
# 转换为GCJ02
NJPI_GCJ_LON, NJPI_GCJ_LAT = wgs84_to_gcj02(NJPI_WGS_LON, NJPI_WGS_LAT)

# SessionState 初始化记忆变量
if "heartbeat_status" not in st.session_state:
    st.session_state.heartbeat_status = "离线"
if "obstacle_polygons" not in st.session_state:
    st.session_state.obstacle_polygons = []  # 障碍物多边形记忆列表
if "obstacle_points" not in st.session_state:
    st.session_state.obstacle_points = []  # 临时障碍物点
if "ab_points" not in st.session_state:
    st.session_state.ab_points = []  # AB点列表
if "drone_sim_pos" not in st.session_state:
    st.session_state.drone_sim_pos = [NJPI_GCJ_LAT, NJPI_GCJ_LON]
if "map_zoom" not in st.session_state:
    st.session_state.map_zoom = 16

# ===================== 页面布局 =====================
st.set_page_config(page_title="无人机地图地面站", layout="wide")
st.title("🛸 无人机心跳包 + OSM地图 + 障碍物圈选 + 模拟飞行系统")

# 侧边栏控制面板
with st.sidebar:
    st.subheader("📡 心跳包状态")
    hb_btn = st.button("刷新心跳包")
    if hb_btn:
        st.session_state.heartbeat_status = "在线" if st.session_state.heartbeat_status=="离线" else "离线"
    st.info(f"当前状态：{st.session_state.heartbeat_status}")

    st.subheader("🗺️ 地图设置")
    map_type = st.radio("地图图层", ["标准街道图", "卫星影像图"])
    st.slider("地图缩放", min_value=10, max_value=18, value=st.session_state.map_zoom, key="map_zoom")

    st.subheader("🚧 障碍物圈选")
    st.write("点击地图添加障碍物顶点，≥3个点自动闭合")
    poly_btn = st.button("保存当前障碍物")
    if poly_btn and len(st.session_state.obstacle_points)>=3:
        st.session_state.obstacle_polygons.append(st.session_state.obstacle_points.copy())
        st.session_state.obstacle_points = []
        st.success("障碍物已保存！")
    clear_poly = st.button("清空所有障碍物")
    if clear_poly:
        st.session_state.obstacle_polygons = []
        st.session_state.obstacle_points = []

    st.subheader("📍 AB点航线")
    clear_ab = st.button("清空AB点")
    if clear_ab:
        st.session_state.ab_points = []

    st.subheader("✈️ 模拟飞行")
    sim_start = st.button("开始模拟飞行")

# ===================== 构建OSM地图 =====================
# 初始化地图
m = folium.Map(
    location=[NJPI_GCJ_LAT, NJPI_GCJ_LON],
    zoom_start=st.session_state.map_zoom,
    tiles=None
)

# 加载OpenStreetMap图层
if map_type == "标准街道图":
    folium.TileLayer("OpenStreetMap", name="OSM标准地图").add_to(m)
else:
    # 卫星图层（高德卫星 适配GCJ02）
    folium.TileLayer(
        tiles='https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德卫星影像',
        name="卫星实况地图"
    ).add_to(m)

# 添加学校标记
folium.Marker(
    location=[NJPI_GCJ_LAT, NJPI_GCJ_LON],
    popup="南京科技职业学院",
    icon=folium.Icon(color="red", icon="school")
).add_to(m)

# 绘制临时障碍物点（未保存的）
for point in st.session_state.obstacle_points:
    folium.CircleMarker(
        location=point,
        radius=5,
        color="purple",
        fill=True,
        fill_color="purple",
        popup="障碍物顶点"
    ).add_to(m)

# 绘制记忆的障碍物多边形
for poly in st.session_state.obstacle_polygons:
    folium.Polygon(
        locations=poly,
        color="orange",
        fill=True,
        fill_color="orange",
        fill_opacity=0.3,
        popup="障碍物区域"
    ).add_to(m)

# 绘制AB点及连线
if len(st.session_state.ab_points) > 0:
    for idx, (lat, lon) in enumerate(st.session_state.ab_points):
        folium.Marker([lat, lon], popup=f"点{idx+1}", icon=folium.Icon(color="blue")).add_to(m)
    if len(st.session_state.ab_points) >=2:
        folium.PolyLine(
            locations=st.session_state.ab_points,
            color="blue",
            weight=3,
            popup="AB航线连线"
        ).add_to(m)

# 绘制无人机模拟位置
folium.Marker(
    location=st.session_state.drone_sim_pos,
    popup="无人机模拟位置",
    icon=folium.Icon(color="green", icon="plane")
).add_to(m)

# ===================== 地图交互逻辑 =====================
map_data = st_folium(m, width=1200, height=600)

# 处理地图点击事件
if map_data and map_data["last_clicked"]:
    click_lat = map_data["last_clicked"]["lat"]
    click_lon = map_data["last_clicked"]["lng"]
    
    # 如果AB点不足2个，优先添加为航点
    if len(st.session_state.ab_points) < 2:
        st.session_state.ab_points.append([click_lat, click_lon])
    else:
        # 否则添加为障碍物顶点
        st.session_state.obstacle_points.append([click_lat, click_lon])

# ===================== 模拟飞行逻辑 =====================
if sim_start and len(st.session_state.ab_points)>=2:
    # 简易两点直线模拟飞行
    start = st.session_state.ab_points[0]
    end = st.session_state.ab_points[1]
    st.session_state.drone_sim_pos = end
    st.success("模拟飞行完成，已飞到B点！")

# ===================== 坐标信息展示 =====================
st.divider()
st.subheader("📌 坐标信息（GCJ-02 已转换）")
col1, col2 = st.columns(2)
with col1:
    st.write(f"学校GCJ02经度：{NJPI_GCJ_LON}")
    st.write(f"学校GCJ02纬度：{NJPI_GCJ_LAT}")
with col2:
    st.write(f"当前模拟无人机坐标：{st.session_state.drone_sim_pos}")
st.write("💡 操作说明：点击地图添加AB航点（最多2个），超过2个点击自动添加为障碍物顶点，≥3个点可保存为障碍物多边形，数据自动记忆保存")
