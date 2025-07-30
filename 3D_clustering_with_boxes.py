import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from sklearn.cluster import DBSCAN
from matplotlib.animation import FuncAnimation

# 1. 读取 CSV，按 detIdx==0 分割帧
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

# 2. 画一个帧的数据（加包围盒，固定坐标轴范围）
def plot_frame(df, ax, xlim, ylim, zlim):
    ax.clear()
    coords = df[['x', 'y', 'z']].values
    db = DBSCAN(eps=0.55, min_samples=3).fit(coords)
    df['cluster'] = db.labels_
    ax.scatter(df['x'], df['y'], df['z'], c=df['cluster'], cmap='tab10', s=8, alpha=0.7)
    for cluster_id in sorted(df['cluster'].unique()):
        if cluster_id == -1:
            continue
        pts = df[df['cluster'] == cluster_id]
        xmin, xmax = pts['x'].min(), pts['x'].max()
        ymin, ymax = pts['y'].min(), pts['y'].max()
        zmin, zmax = pts['z'].min(), pts['z'].max()
        color = plt.cm.tab10(cluster_id % 10)
        draw_bounding_box(ax, xmin, xmax, ymin, ymax, zmin, zmax, color=color, alpha=0.18)
    ax.set_title('3D Point Cloud (Animated)')
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_zlim(zlim)

# 3. 动画主程序
def animate(frames):
    # 提前计算所有点云的全局范围
    all_points = pd.concat(frames)
    xlim = (all_points['x'].min(), all_points['x'].max())
    ylim = (all_points['y'].min(), all_points['y'].max())
    zlim = (all_points['z'].min(), all_points['z'].max())

    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')

    def update(i):
        plot_frame(frames[i], ax, xlim, ylim, zlim)

    # interval单位毫秒，800=0.8秒一帧，可按需调整更慢/更快
    ani = FuncAnimation(fig, update, frames=len(frames), interval=1000)
    plt.show()

if __name__ == "__main__":
    # 用你的实际文件名
    frames = load_frames("xwr18xx_AOP_processed_stream_2025_05_12T04_57_17_397_with_timestamp2.csv")
    animate(frames)
