import pandas as pd
import numpy as np
from sklearn.cluster import DBSCAN

# 参数配置
csv_file = 'xwr18xx_AOP_processed_stream_2025_05_12T04_57_17_397_with_timestamp2.csv'
eps = 0.5         # DBSCAN 聚类半径 (可根据场景调节)
min_samples = 3   # DBSCAN 最小样本数 (可根据场景调节)
speed_threshold = 4.0  # >4 m/s 判定为“危险电动滑板车”

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
