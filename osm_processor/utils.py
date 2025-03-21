import math

import matplotlib.pyplot as plt
from matplotlib import collections as mc
import numpy as np

import numpy as np
from sklearn.linear_model import RANSACRegressor, LinearRegression
import random


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


# 假设points已按x排序
points = [Point(0, 0), Point(1, 0.1), ..., Point(25, 0)]
segments = []
s = 0
min_length = 3
max_length = 10
threshold1 = 0.2  # y=0的阈值
threshold2 = 0.3  # y=n的阈值
threshold3 = 1.0  # 斜线残差阈值
zero_threshold = 0.1  # 判断n是否为0的阈值

while s < len(points):
    best_e = s
    best_model = None
    best_params = []
    for e in range(s + min_length - 1, min(s + max_length, len(points))):
        current = points[s:e + 1]
        X = np.array([p.x for p in current]).reshape(-1, 1)
        y = np.array([p.y for p in current])

        # 模型1：y=0
        inliers1 = [p for p in current if abs(p.y) <= threshold1]

        # 模型2：y=n≠0
        best_n_inliers = []
        best_n = None
        for _ in range(20):  # 随机采样次数
            sample = random.choice(current)
            n = sample.y
            inliers = [p for p in current if abs(p.y - n) <= threshold2]
            if len(inliers) > len(best_n_inliers):
                best_n_inliers = inliers
                best_n = n

        # 模型3：斜线
        ransac = RANSACRegressor(LinearRegression(), residual_threshold=threshold3)
        try:
            ransac.fit(X, y)
            inlier_mask = ransac.inlier_mask_
            inliers3 = [current[i] for i in range(len(current)) if inlier_mask[i]]
        except:
            inliers3 = []

        # 选择最佳模型
        models = [
            (len(inliers1), 1, 0),
            (len(best_n_inliers), 2, best_n),
            (len(inliers3), 3, (ransac.estimator_.coef_[0], ransac.estimator_.intercept_))
        ]
        models.sort(reverse=True, key=lambda x: x[0])
        best = models[0]

        if best[0] >= min_length and best[0] / len(current) > 0.6:
            if best[1] == 2 and abs(best[2]) < zero_threshold:
                best = (best[0], 1, 0)
            best_model = best[1]
            best_params = best[2]
            best_e = e

    if best_model:
        start_x = points[s].x
        end_x = points[best_e].x
        if best_model == 1:
            segments.append(('type1', start_x, end_x, 0))
        elif best_model == 2:
            segments.append(('type2', start_x, end_x, best_params))
        else:
            a, b = best_params
            segments.append(('type3', start_x, end_x, a, b))
        s = best_e + 1
    else:
        s += 1

print(segments)


def calculate_curvature(coords1, coords2, coords3):
    (x1, y1), (x2, y2), (x3, y3) = coords1, coords2, coords3
    vector1 = (x2 - x1, y2 - y1)
    vector2 = (x3 - x2, y3 - y2)
    vector3 = (x3 - x1, y3 - y1)
    cross = vector1[0] * vector2[1] - vector1[1] * vector2[0]
    if cross == 0.:
        return 0.

    ab = math.hypot(vector1[0], vector1[1])
    bc = math.hypot(vector2[0], vector2[1])
    ca = math.hypot(vector3[0], vector3[1])

    curvature = 2 * abs(cross) / (ab * bc * ca)

    sign = 1 if cross > 0 else -1
    return sign * curvature

def visualize_network(roadnetwork, output_file="road_network.pdf"):
    if not roadnetwork.road_segments:
        print("No segments to visualize.")
        return

    # 初始化绘图
    plt.figure(figsize=(12, 12), dpi=300)  # 改用正方形画布
    ax = plt.gca()

    # 设置等比例坐标轴
    ax.set_aspect('equal')  # 关键修改：保证坐标比例一致
    plt.axis('off')  # 关闭所有坐标轴元素

    # 生成更易区分的颜色方案
    colors = plt.cm.gist_ncar(np.linspace(0, 1, len(roadnetwork.road_segments)))

    # 使用LineCollection优化绘制性能
    all_lines = []
    for idx, segment in enumerate(roadnetwork.road_segments.values()):
        coords = []
        for node_id in segment.nodes:
            if node_id in roadnetwork.nodes:
                node = roadnetwork.nodes[node_id]
                coords.append([node.x, node.y])

        if len(coords) > 1:
            # 转换为线段集合
            lines = np.array([(coords[i], coords[i + 1]) for i in range(len(coords) - 1)])
            all_lines.append(lines)

            # 使用渐变色增强可视性（可选）
            lc = mc.LineCollection(
                lines,
                colors=[colors[idx % len(colors)]],
                linewidths=1.2,
                alpha=0.8,
                zorder=3
            )
            ax.add_collection(lc)

    # 绘制交叉点（优化渲染顺序）
    if roadnetwork.junctions:
        junc_coords = np.array([
            (roadnetwork.nodes[jid].x, roadnetwork.nodes[jid].y)
            for jid in roadnetwork.junctions if jid in roadnetwork.nodes
        ])
        ax.scatter(
            junc_coords[:, 0], junc_coords[:, 1],
            s=8, c='#FF1F1F',  # 更醒目的红色
            edgecolors='#400000',  # 深色边框
            linewidths=0.1,
            zorder=2,  # 确保交叉点显示在道路上方
            marker='o'  # 圆形标记
        )

    # 自动适应数据范围
    ax.autoscale_view()

    # 保存为紧凑PDF（去除白边）
    plt.savefig(
        output_file,
        format='pdf',
        bbox_inches='tight',
        pad_inches=0.05,  # 最小化留白
        transparent=False  # 保持白色背景
    )
    plt.close()