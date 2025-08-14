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

def draw_bounding_box(ax, xmin, xmax, ymin, ymax, zmin, zmax, color='r', alpha=0.2):
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
    box = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor='k', linewidths=0.8)
    ax.add_collection3d(box)

# =============================
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

# === NEW(v2): 绘制刚体（两个盒子并集 + 圆柱） ==============================
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

def draw_L_scooter(ax, x_centerline, y_back, z0=0.0, color='tab:purple'):
    """ L 形 = 立柱盒 + 踏板盒，从 (x_centerline - L_W/2, y_back, z0) 起 """
    x0 = x_centerline - L_W/2.0
    y0 = y_back
    draw_box(ax, x0, y0, z0, L_W, PILLAR_Y, L_H, color=color, alpha=0.18)  # 立柱
    draw_box(ax, x0, y0, z0, L_W, L_LEN,    DECK_Z, color=color, alpha=0.18)  # 踏板
# === END NEW(v2) ==========================================================

# === NEW(v2): 多普勒工具与检测/跟踪 =======================================
def radial_velocity_and_fd(d0, y, v, fc):
    c = 299792458.0
    lam = c / fc
    r = np.sqrt(d0**2 + y**2)
    vr = v * (y / r)
    fD = 2.0 * vr / lam
    return r, vr, fD

# 几何门限（粗门限，防止“无中生有”）
DETECT_CFG = {
    "min_points": 20,          # 簇最少点数
    "x_width_range": (0.12, 0.40), # 期望宽度 ~0.23 m
    "y_len_range":   (0.70, 1.70), # 期望长度 ~1.17 m
    "z_height_range":(1.20, 2.10), # 期望总高 ~1.75 m（有人）
    "zmin_max": 0.25,          # 贴地：zmin 不高于 0.25 m
    "score_thresh": 4.0        # 形状评分门限（越小越贴合）
}

def shape_score(dx, dy, dz):
    # 与期望值的归一化平方误差，越小越好
    ex, ey, ez = L_W, L_LEN, HUM_H
    sx = ((dx-ex)/max(ex,1e-6))**2
    sy = ((dy-ey)/max(ey,1e-6))**2
    sz = ((dz-ez)/max(ez,1e-6))**2
    return sx + sy + sz

def detect_scooter_rider_from_clusters(df_with_labels):
    """从聚类结果中选一个最像‘人+车’的簇；找不到则返回 None"""
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
        if zmin > DETECT_CFG["zmin_max"]:  # 不贴地
            continue

        sc = shape_score(dx, dy, dz)
        cand = {
            "cid": cid,
            "center_x": 0.5*(xmin+xmax),
            "center_y": 0.5*(ymin+ymax),
            "zmin": zmin,
            "dx": dx, "dy": dy, "dz": dz,
            "score": sc,
            "n": len(pts)
        }
        if best is None or sc < best["score"]:
            best = cand

    # 打分门限：太离谱就不要
    if best is not None and best["score"] <= DETECT_CFG["score_thresh"]:
        return best
    return None

class ABTracker:
    """极简 α-β 跟踪器（默认不容忍遮挡：miss>0 即不显示）"""
    def __init__(self, alpha=0.6, beta=0.2, dt=1.0, max_miss=0):
        self.alpha, self.beta = alpha, beta
        self.dt = dt
        self.max_miss = max_miss
        self.active = False
        self.miss = 0
        self.y = 0.0
        self.v = 0.0
        self.x = 0.0  # 横向位置 ~ d0

    def predict(self):
        self.y = self.y + self.v * self.dt

    def update(self, meas):  # meas: dict with center_x, center_y
        if meas is None:
            # 没有检测：不显示，且不“无中生有”
            self.miss += 1
            if self.miss > self.max_miss:
                self.active = False
            else:
                self.predict()
            return

        # 有检测
        if not self.active:
            # 初始化
            self.y = meas["center_y"]
            self.v = 0.0
            self.x = meas["center_x"]
            self.miss = 0
            self.active = True
            return

        # 预测-更新
        y_pred = self.y + self.v * self.dt
        r = meas["center_y"] - y_pred
        self.y = y_pred + self.alpha * r
        self.v = self.v + (self.beta * r) / self.dt
        # x 用低通
        self.x = 0.8*self.x + 0.2*meas["center_x"]
        self.miss = 0
# === END NEW(v2) ==========================================================


# 2) 画一帧（保持你的点云+包围盒；在末尾基于“检测/跟踪”决定是否叠加刚体）
def plot_frame(df, ax, xlim, ylim, zlim,
               # === CHANGED(v2): 去掉强制 overlay，改为由 tracker 控制 ===
               tracker=None, fc=77e9
               # === END CHANGED(v2) ===
               ):
    ax.clear()

    # 原始点云与聚类
    coords = df[['x', 'y', 'z']].values
    db = DBSCAN(eps=0.55, min_samples=3).fit(coords)
    df = df.copy()
    df['cluster'] = db.labels_
    ax.scatter(df['x'], df['y'], df['z'], c=df['cluster'], cmap='tab10', s=8, alpha=0.7)

    # 画 AABB
    for cluster_id in sorted(df['cluster'].unique()):
        if cluster_id == -1:
            continue
        pts = df[df['cluster'] == cluster_id]
        xmin, xmax = pts['x'].min(), pts['x'].max()
        ymin, ymax = pts['y'].min(), pts['y'].max()
        zmin, zmax = pts['z'].min(), pts['z'].max()
        color = plt.cm.tab10(cluster_id % 10)
        draw_bounding_box(ax, xmin, xmax, ymin, ymax, zmin, zmax, color=color, alpha=0.18)

    # === NEW(v2): 目标检测 -> 跟踪更新 =====================================
    detection = detect_scooter_rider_from_clusters(df)
    if tracker is not None:
        tracker.update(detection)
    # === END NEW(v2) =======================================================

    # === NEW(v2): 只有当“当前帧有检测且 tracker.active”为真时，才叠加刚体模型与多普勒 ===
    if tracker is not None and tracker.active and tracker.miss == 0:
        # 跟踪器状态
        d0_est = tracker.x
        y_center = tracker.y
        v_est = tracker.v

        # L 形后沿位置：y_back = “人圆柱中心 y” - HUM_YC
        y_back = y_center - HUM_YC

        # 画刚体：L + 人
        draw_L_scooter(ax, x_centerline=d0_est, y_back=y_back, z0=0.0, color='tab:purple')
        draw_cylinder(ax, d0_est, y_center, HUM_Z0, HUM_R, HUM_H, color='tab:orange', alpha=0.18)

        # 多普勒（用估计速度；若你希望用差分速度，可直接用 v_est）
        _, vr, fD = radial_velocity_and_fd(d0=d0_est, y=y_center, v=max(v_est, 1e-6), fc=fc)
        ax.text2D(0.02, 0.95,
                  f"Target DETECTED  |  y={y_center:.2f} m  v={v_est:.2f} m/s\n"
                  f"v_r={vr:.2f} m/s  f_D={fD/1e3:.2f} kHz",
                  transform=ax.transAxes, fontsize=10,
                  bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
    else:
        # 没有检测 -> 明确提示“未检出”，但不画任何目标几何
        ax.text2D(0.02, 0.95, "No target detected", transform=ax.transAxes,
                  fontsize=10, bbox=dict(facecolor='white', alpha=0.6, edgecolor='none'))
    # === END NEW(v2) =======================================================

    # 统一坐标与视角
    ax.set_title('3D Point Cloud (Animated) + DETECT-THEN-TRACK Overlay')
    ax.set_xlim(xlim); ax.set_ylim(ylim); ax.set_zlim(zlim)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=20, azim=60)
    ax.set_box_aspect([1,1,1])  # 等比例显示


# 3) 动画主程序
def animate(frames,
            # === CHANGED(v2): 新增跟踪器控制参数（默认不容忍遮挡） ===
            alpha=0.6, beta=0.2, interval_ms=1000, fc=77e9, max_miss=0
            # === END CHANGED(v2) ===
            ):
    all_points = pd.concat(frames)
    # 点范围
    x_min, x_max = all_points['x'].min(), all_points['x'].max()
    y_min, y_max = all_points['y'].min(), all_points['y'].max()
    z_min, z_max = all_points['z'].min(), all_points['z'].max()

    # === NEW(v2): 显示边界（保守一些） ===
    xlim = (x_min - 0.5, x_max + 0.5)
    ylim = (y_min - 2.0, y_max + 2.0)
    zlim = (min(0.0, z_min - 0.2), max(z_max + 0.5, HUM_H + 0.3))
    # === END NEW(v2) ===

    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')

    interval_s = interval_ms / 1000.0

    # === NEW(v2): 初始化跟踪器（dt=帧间隔，默认 max_miss=0 => 无遮挡容忍） ===
    tracker = ABTracker(alpha=alpha, beta=beta, dt=interval_s, max_miss=max_miss)
    # === END NEW(v2) ===

    def update(i):
        plot_frame(frames[i], ax, xlim, ylim, zlim,
                   tracker=tracker, fc=fc)

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

