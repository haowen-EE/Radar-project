import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from sklearn.cluster import DBSCAN
from matplotlib.animation import FuncAnimation

# 1) 读取 CSV，按 detIdx==0 分割帧（保持你的原逻辑）
def load_frames(csv_file):
    df = pd.read_csv(csv_file)
    frames = []
    current = []
    for idx, row in df.iterrows():
        if row.get('detIdx', 0) == 0 and current:
            frames.append(pd.DataFrame(current))
            current = []
        current.append(row)
    if current:
        frames.append(pd.DataFrame(current))
    return frames

def draw_bounding_box(ax, xmin, xmax, ymin, ymax, zmin, zmax, color='r', alpha=0.2, lw=0.8):
    corners = np.array([
        [xmin, ymin, zmin], [xmax, ymin, zmin], [xmax, ymax, zmin], [xmin, ymax, zmin],
        [xmin, ymin, zmax], [xmax, ymin, zmax], [xmax, ymax, zmax], [xmin, ymax, zmax]
    ])
    faces = [
        [corners[i] for i in [0,1,2,3]],
        [corners[i] for i in [4,5,6,7]],
        [corners[i] for i in [0,1,5,4]],
        [corners[i] for i in [2,3,7,6]],
        [corners[i] for i in [1,2,6,5]],
        [corners[i] for i in [0,3,7,4]]
    ]
    box = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor='k', linewidths=lw)
    ax.add_collection3d(box)

# === NEW(v2 from before): 你的“电动滑板车+人”几何常量 ==========================
# 坐标：x=横向(朝雷达)，y=沿道路，z=向上。地面 z=0。
L_LEN = 1.17   # 外轮廓长度(沿 y), m
L_W   = 0.23   # 宽度(沿 x), m
L_H   = 1.25   # 高度(沿 z), m
CUT_LEN = 0.73 # 内缺口长度, m
CUT_H   = 1.05 # 内缺口高度, m
PILLAR_Y = L_LEN - CUT_LEN   # 立柱厚度(沿 y) = 0.44 m
DECK_Z   = L_H   - CUT_H     # 踏板厚度(沿 z) = 0.20 m

HUM_H = 1.75   # 人高
HUM_R = 0.18   # 人半径
HUM_Z0 = DECK_Z
HUM_YC = (PILLAR_Y + L_LEN)/2.0  # 人圆柱中心: 相对后沿 y 偏移 ≈ 0.805 m
# === END NEW(v2) ==========================================================

# === NEW(v3): “行走人（躯干圆柱）”可靠口径 ==============================
# 采用更稳的“躯干”近似：高度 ~1.30 m，自 z0~0.40 m 起，半径 0.18 m（直径 0.36 m）
PED_EXPECT = {"dx": 0.36, "dy": 0.35, "dz": 1.30, "z0": 0.40}  # 期望尺寸
# 门限（更宽松以兼顾个体差异/抖动）
PED_CFG = {
    "min_points": 15,
    "x_width_range": (0.20, 0.60),   # 肩宽/厚衣/背包
    "y_len_range":   (0.15, 0.90),   # 身体前后厚度 + 抖动/展宽
    "z_height_range":(1.05, 1.55),   # 躯干高度
    "zmin_range":    (0.25, 0.60),   # 躯干下沿高度（约 0.40 ± 0.15）
    "score_thresh":  4.0
}
# === END NEW(v3) ==========================================================

# === NEW(v3): 形状评分（通用） ============================================
def norm_shape_score(dx, dy, dz, ex, ey, ez):
    # 与期望值的归一化平方误差和（越小越像）
    sx = ((dx-ex)/max(ex,1e-6))**2
    sy = ((dy-ey)/max(ey,1e-6))**2
    sz = ((dz-ez)/max(ez,1e-6))**2
    return sx + sy + sz
# === END NEW(v3) ==========================================================

# === NEW(v2 from before): 工具（可保留，后面用不到刚体覆盖也没关系） ==========
def draw_box(ax, x0, y0, z0, sx, sy, sz, color='tab:purple', alpha=0.15):
    xmin, xmax = x0, x0+sx
    ymin, ymax = y0, y0+sy
    zmin, zmax = z0, z0+sz
    draw_bounding_box(ax, xmin, xmax, ymin, ymax, zmin, zmax, color=color, alpha=alpha)

def draw_cylinder(ax, xc, yc, z0, R, H, color='tab:orange', alpha=0.15, n_theta=32, n_z=10):
    theta = np.linspace(0, 2*np.pi, n_theta)
    z = np.linspace(z0, z0+H, n_z)
    Theta, Z = np.meshgrid(theta, z)
    X = xc + R*np.cos(Theta)
    Y = yc + R*np.sin(Theta)  # 圆柱轴线沿 z
    ax.plot_surface(X, Y, Z, linewidth=0.0, antialiased=False, alpha=alpha, color=color, edgecolor='none')
# === END (工具) ===========================================================

# === NEW(v3): 在方框上打名字 =============================================
def draw_labeled_box(ax, xmin, xmax, ymin, ymax, zmin, zmax, label, color='lime', alpha=0.10):
    # 高亮方框
    draw_bounding_box(ax, xmin, xmax, ymin, ymax, zmin, zmax, color=color, alpha=alpha, lw=1.6)
    # 标签放在盒顶中心略上
    xmid = 0.5*(xmin+xmax); ymid = 0.5*(ymin+ymax); ztop = zmax + 0.05
    ax.text(xmid, ymid, ztop, label, color='black',
            bbox=dict(facecolor='white', alpha=0.9, edgecolor='black', boxstyle='round,pad=0.3'),
            ha='center', va='bottom', fontsize=10)
# === END NEW(v3) ==========================================================

# === NEW(v2 from before): 多普勒（若要显示速度可继续用） ===================
def radial_velocity_and_fd(d0, y, v, fc):
    c = 299792458.0
    lam = c / fc
    r = np.sqrt(d0**2 + y**2)
    vr = v * (y / r)
    fD = 2.0 * vr / lam
    return r, vr, fD
# === END ==================================================================

# === NEW(v2 from before): 旧的“滑板车+人”检测 =============================
DETECT_CFG = {
    "min_points": 20,
    "x_width_range": (0.12, 0.40),
    "y_len_range":   (0.70, 1.70),
    "z_height_range":(1.20, 2.10),
    "zmin_max": 0.25,
    "score_thresh": 4.0
}
def shape_score_scooter(dx, dy, dz):
    return norm_shape_score(dx, dy, dz, L_W, L_LEN, HUM_H)

def detect_scooter_rider_from_clusters(df_with_labels):
    best = None
    for cid in sorted(df_with_labels['cluster'].unique()):
        if cid == -1:  # 噪声
            continue
        pts = df_with_labels[df_with_labels['cluster'] == cid]
        if len(pts) < DETECT_CFG["min_points"]:
            continue
        xmin, xmax = pts['x'].min(), pts['x'].max()
        ymin, ymax = pts['y'].min(), pts['y'].max()
        zmin, zmax = pts['z'].min(), pts['z'].max()
        dx, dy, dz = xmax-xmin, ymax-ymin, zmax-zmin
        # 粗门限
        if not (DETECT_CFG["x_width_range"][0] <= dx <= DETECT_CFG["x_width_range"][1]):  continue
        if not (DETECT_CFG["y_len_range"][0]   <= dy <= DETECT_CFG["y_len_range"][1]):    continue
        if not (DETECT_CFG["z_height_range"][0]<= dz <= DETECT_CFG["z_height_range"][1]): continue
        if zmin > DETECT_CFG["zmin_max"]:
            continue
        sc = shape_score_scooter(dx, dy, dz)
        cand = {"type":"Scooter+Rider","cid":cid,"center_x":0.5*(xmin+xmax),"center_y":0.5*(ymin+ymax),
                "zmin":zmin,"dx":dx,"dy":dy,"dz":dz,"score":sc,
                "bbox":(xmin,xmax,ymin,ymax,zmin,zmax)}
        if best is None or sc < best["score"]:
            best = cand
    if best is not None and best["score"] <= DETECT_CFG["score_thresh"]:
        return best
    return None
# === END ==================================================================

# === NEW(v3): 新增“行走人（躯干）”检测 ====================================
def detect_pedestrian_from_clusters(df_with_labels):
    best = None
    for cid in sorted(df_with_labels['cluster'].unique()):
        if cid == -1:
            continue
        pts = df_with_labels[df_with_labels['cluster'] == cid]
        if len(pts) < PED_CFG["min_points"]:
            continue
        xmin, xmax = pts['x'].min(), pts['x'].max()
        ymin, ymax = pts['y'].min(), pts['y'].max()
        zmin, zmax = pts['z'].min(), pts['z'].max()
        dx, dy, dz = xmax-xmin, ymax-ymin, zmax-zmin

        # 门限（宽/长/高 + 躯干下沿高度）
        if not (PED_CFG["x_width_range"][0] <= dx <= PED_CFG["x_width_range"][1]):  continue
        if not (PED_CFG["y_len_range"][0]   <= dy <= PED_CFG["y_len_range"][1]):    continue
        if not (PED_CFG["z_height_range"][0]<= dz <= PED_CFG["z_height_range"][1]): continue
        if not (PED_CFG["zmin_range"][0]    <= zmin <= PED_CFG["zmin_range"][1]):   continue

        sc = norm_shape_score(dx, dy, dz, PED_EXPECT["dx"], PED_EXPECT["dy"], PED_EXPECT["dz"])
        cand = {"type":"Pedestrian","cid":cid,"center_x":0.5*(xmin+xmax),"center_y":0.5*(ymin+ymax),
                "zmin":zmin,"dx":dx,"dy":dy,"dz":dz,"score":sc,
                "bbox":(xmin,xmax,ymin,ymax,zmin,zmax)}
        if best is None or sc < best["score"]:
            best = cand
    if best is not None and best["score"] <= PED_CFG["score_thresh"]:
        return best
    return None
# === END NEW(v3) ==========================================================

# === NEW(v3): 二选一（同帧同时命中时选评分更优者） =========================
def select_best_detection(df_with_labels):
    a = detect_scooter_rider_from_clusters(df_with_labels)
    b = detect_pedestrian_from_clusters(df_with_labels)
    if a is None and b is None:
        return None
    if a is not None and b is not None:
        return a if a["score"] <= b["score"] else b
    return a if a is not None else b
# === END NEW(v3) ==========================================================

# === CHANGED(v3): 跟踪器加入尺寸/标签，便于画有名方框 =======================
class ABTracker:
    """极简 α-β 跟踪器（默认不容忍遮挡：miss>0 即不显示）"""
    def __init__(self, alpha=0.6, beta=0.2, dt=1.0, max_miss=0):
        self.alpha, self.beta, self.dt = alpha, beta, dt
        self.max_miss = max_miss
        self.active = False
        self.miss = 0
        self.y = 0.0; self.v = 0.0; self.x = 0.0
        self.dx = 0.0; self.dy = 0.0; self.dz = 0.0; self.zmin = 0.0
        self.label = ""

    def predict(self):
        self.y = self.y + self.v * self.dt

    def update(self, meas):  # meas: dict with center_x,center_y,dx,dy,dz,zmin,label
        if meas is None:
            self.miss += 1
            if self.miss > self.max_miss:
                self.active = False
            else:
                self.predict()
            return

        if not self.active:
            self.y = meas["center_y"]; self.v = 0.0; self.x = meas["center_x"]
            self.dx = meas["dx"]; self.dy = meas["dy"]; self.dz = meas["dz"]; self.zmin = meas["zmin"]
            self.label = meas["type"]
            self.miss = 0; self.active = True; return

        # 预测-更新
        y_pred = self.y + self.v * self.dt
        r = meas["center_y"] - y_pred
        self.y = y_pred + self.alpha * r
        self.v = self.v + (self.beta * r) / self.dt
        self.x = 0.8*self.x + 0.2*meas["center_x"]
        # 尺寸低通
        self.dx = 0.8*self.dx + 0.2*meas["dx"]
        self.dy = 0.8*self.dy + 0.2*meas["dy"]
        self.dz = 0.8*self.dz + 0.2*meas["dz"]
        self.zmin = 0.8*self.zmin + 0.2*meas["zmin"]
        self.label = meas["type"]
        self.miss = 0
# === END CHANGED(v3) ======================================================


# 2) 画一帧（保留点云+普通 AABB；高亮并命名被跟踪目标）
def plot_frame(df, ax, xlim, ylim, zlim, tracker=None, fc=77e9):
    ax.clear()

    # 原始点云与聚类
    coords = df[['x', 'y', 'z']].values
    db = DBSCAN(eps=0.55, min_samples=3).fit(coords)
    df = df.copy()
    df['cluster'] = db.labels_
    ax.scatter(df['x'], df['y'], df['z'], c=df['cluster'], cmap='tab10', s=8, alpha=0.7)

    # 画所有簇的 AABB（普通）
    for cluster_id in sorted(df['cluster'].unique()):
        if cluster_id == -1:
            continue
        pts = df[df['cluster'] == cluster_id]
        xmin, xmax = pts['x'].min(), pts['x'].max()
        ymin, ymax = pts['y'].min(), pts['y'].max()
        zmin, zmax = pts['z'].min(), pts['z'].max()
        color = plt.cm.tab10(cluster_id % 10)
        draw_bounding_box(ax, xmin, xmax, ymin, ymax, zmin, zmax, color=color, alpha=0.18, lw=0.8)

    # === NEW(v3): 选择最优检测 -> 跟踪 ======================================
    detection = select_best_detection(df)
    if tracker is not None:
        tracker.update(detection)
    # === END NEW(v3) =======================================================

    # === NEW(v3): 仅当 active 且本帧有检测时，高亮+命名 =====================
    if tracker is not None and tracker.active and tracker.miss == 0:
        xmin = tracker.x - tracker.dx/2; xmax = tracker.x + tracker.dx/2
        ymin = tracker.y - tracker.dy/2; ymax = tracker.y + tracker.dy/2
        zmin = tracker.zmin;            zmax = tracker.zmin + tracker.dz
        draw_labeled_box(ax, xmin, xmax, ymin, ymax, zmin, zmax, label=tracker.label, color='lime', alpha=0.10)

        # 可选：显示速度/多普勒（保留原有左上角信息）
        # _, vr, fD = radial_velocity_and_fd(d0=tracker.x, y=tracker.y, v=max(tracker.v,1e-6), fc=fc)
        ax.text2D(0.02, 0.95, f"Detected: {tracker.label}  |  y={tracker.y:.2f} m  v={tracker.v:.2f} m/s",
                  transform=ax.transAxes, fontsize=10,
                  bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'))
    else:
        ax.text2D(0.02, 0.95, "No target detected", transform=ax.transAxes,
                  fontsize=10, bbox=dict(facecolor='white', alpha=0.65, edgecolor='none'))
    # === END NEW(v3) =======================================================

    # 统一坐标与视角
    ax.set_title('3D Point Cloud (Animated) + Detect→Track with Named Box')
    ax.set_xlim(xlim); ax.set_ylim(ylim); ax.set_zlim(zlim)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=20, azim=60)
    ax.set_box_aspect([1,1,1])  # 等比例显示

# 3) 动画主程序
def animate(frames, alpha=0.6, beta=0.2, interval_ms=1000, fc=77e9, max_miss=0):
    all_points = pd.concat(frames)
    # 点范围
    x_min, x_max = all_points['x'].min(), all_points['x'].max()
    y_min, y_max = all_points['y'].min(), all_points['y'].max()
    z_min, z_max = all_points['z'].min(), all_points['z'].max()

    # 显示边界（保守一些）
    xlim = (x_min - 0.5, x_max + 0.5)
    ylim = (y_min - 2.0, y_max + 2.0)
    zlim = (min(0.0, z_min - 0.2), max(z_max + 0.5, HUM_H + 0.3))

    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')

    interval_s = interval_ms / 1000.0
    tracker = ABTracker(alpha=alpha, beta=beta, dt=interval_s, max_miss=max_miss)

    def update(i):
        plot_frame(frames[i], ax, xlim, ylim, zlim, tracker=tracker, fc=fc)

    ani = FuncAnimation(fig, update, frames=len(frames), interval=interval_ms)
    plt.show()


if __name__ == "__main__":
    # 用你的实际 CSV 文件名
    csv_file = "xwr18xx_AOP_processed_stream_2025_05_12T04_57_17_397_with_timestamp2.csv"
    frames = load_frames(csv_file)

    # 跟踪器与动画参数
    animate(frames,
            alpha=0.6, beta=0.2,
            interval_ms=1000,
            fc=77e9,
            max_miss=0)  # 0 = 本帧没检出就不显示，避免“无中生有”


