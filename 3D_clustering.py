import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

# 参数配置
csv_file = 'xwr18xx_AOP_processed_stream_2025_05_12T04_57_17_397_with_timestamp2.csv'
eps = 0.55        # DBSCAN 聚类半径 (可根据场景调节)，调试范围0.55-0.6，也可以试试0.4
min_samples = 3   # DBSCAN 最小样本数 (可根据场景调节)
speed_threshold = 4.0  # >4 m/s 判定为“危险电动滑板车”

def plot_clusters(df): # 还需要修改，存疑？可视化帮助，matplotlib 调用
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(df['x'], df['y'], df['z'], c=df['cluster'], cmap='tab10', s=8)
    plt.title('DBSCAN Cluster 3D View')
    plt.show()

def main():
    # 1. 读取 CSV
    df = pd.read_csv(csv_file)

    # 2. 聚类：基于 (x, y, z) 三维坐标
    coords = df[['x', 'y', 'z']].values
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(coords)
    df['cluster'] = db.labels_

    # 3. 对每个簇计算平均速度并分类
    results = []
    for cluster_id in sorted(df['cluster'].unique()):
        if cluster_id == -1:
            continue  # 忽略噪声点
        cluster_pts = df[df['cluster'] == cluster_id]

        #demo 结构分析部分，判断空间扩展范围（滑板车和人）
        x_range = cluster_pts['x'].max() - cluster_pts['x'].min()
        y_range = cluster_pts['y'].max() - cluster_pts['y'].min()
        z_range = cluster_pts['z'].max() - cluster_pts['z'].min()
        
        if x_range > 1.0 and z_range <0.3
            printf(f"Cluster {cluster_id} 可能是滑板车底座（横向长，竖向短）")
        elif z_range > 1.7:
            print(f"Cluster {cluster_id} 可能是举手人或滑板车带人")
        #平均速度分类
        avg_speed = np.mean(np.abs(cluster_pts['v']))
        cls = 'Dangerous e-scooter' if avg_speed > speed_threshold else 'Pedestrian/Other'
        results.append({
            'cluster_id': cluster_id,
            'n_points': len(cluster_pts),
            'avg_speed_m_s': round(avg_speed, 3),
            'classification': cls
        })

    # 4. 打印结果
    for r in results:
        print(f"Cluster {r['cluster_id']}: points={r['n_points']}, "
              f"avg_speed={r['avg_speed_m_s']} m/s, class={r['classification']}")

if __name__ == "__main__":
    main()
