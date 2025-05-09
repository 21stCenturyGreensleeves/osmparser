import math

import matplotlib.pyplot as plt
from matplotlib import collections as mc
import numpy as np
from matplotlib.collections import LineCollection, PatchCollection
import matplotlib as mpl
import matplotlib.patches as mpatches

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

    if hasattr(roadnetwork, 'buildings') and roadnetwork.buildings:
        building_patches = []
        for building in roadnetwork.buildings.values():
            # 收集建筑轮廓点坐标
            coordinates = []
            for node_id in building.nodes:
                if node_id in roadnetwork.nodes:
                    node = roadnetwork.nodes[node_id]
                    coordinates.append([node.x, node.y])

            # 创建多边形补丁（至少需要3个点）
            if len(coordinates) >= 3:
                polygon = mpatches.Polygon(
                    coordinates,
                    closed=True,  # 自动闭合多边形
                    fill=True,
                    edgecolor='#404040',  # 深灰色边框
                    facecolor='#F0F0F0',  # 浅灰色填充
                    linewidth=0.3,
                    alpha=0.7,  # 半透明效果
                    zorder=1  # 确保在最底层
                )
                building_patches.append(polygon)

        # 批量添加建筑集合
        if building_patches:
            ax.add_collection(PatchCollection(
                building_patches,
                match_original=True  # 保留各patch的独立样式
            ))

    # 生成更易区分的颜色方案
    colors = plt.cm.tab20(np.linspace(0, 1, len(roadnetwork.road_segments)))

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
                linewidths=1.5,
                alpha=1.0,
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

def seg_to_parampoly3(coords):
    """
    coeff_x[0, 1, 2, 3]: du, cu, bu, au
    coeff_y[0, 1, 2, 3]: dv, cv, bv, av
    """
    diffs = np.diff(coords, axis=0)
    chord_lengths = np.linalg.norm(diffs, axis=1)
    t = np.insert(np.cumsum(chord_lengths), 0, 0)
    t_normalized = t / t[-1]

    x = coords[:, 0]
    y = coords[:, 1]

    A = np.vstack([t_normalized ** 3, t_normalized ** 2, t_normalized, np.ones_like(t_normalized)]).T
    coeff_x, _, _, _ = np.linalg.lstsq(A, x, rcond=None)
    coeff_y, _, _, _ = np.linalg.lstsq(A, y, rcond=None)

    return coeff_x, coeff_y