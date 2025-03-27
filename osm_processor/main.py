import os

from fontTools.ttx import process

from utils import visualize_network
from models import RoadNetwork
import matplotlib.pyplot as plt
from scenariogeneration import xodr

def plot_coordinates(points):
    if not points:
        print("警告：输入列表为空！")
        return

    # 分离 x 和 y 坐标
    x_coords = [point[0] for point in points]
    y_coords = [point[1] for point in points]

    # 创建图形和坐标轴
    fig, ax = plt.subplots()

    # 绘制散点图（点和连线）
    ax.plot(x_coords, y_coords, 'ro-')  # 红点 + 实线
    ax.scatter(x_coords, y_coords, color='red', zorder=5)  # 确保点在最上层

    # 设置坐标轴样式
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('zero')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')

    # 自动调整坐标范围（带 10% 边距）
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)
    x_margin = 0.1 * (x_max - x_min) if (x_max != x_min) else 0.5
    y_margin = 0.1 * (y_max - y_min) if (y_max != y_min) else 0.5

    ax.set_xlim(x_min - x_margin, x_max + x_margin)
    ax.set_ylim(y_min - y_margin, y_max + y_margin)

    # 添加网格和标签
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_title("二元坐标系图", pad=20)
    ax.set_xlabel("X 轴", loc='right')
    ax.set_ylabel("Y 轴", loc='top')

    # 显示图形
    plt.show()

def main():
    # 获取OSM文件路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)
    osm_path = os.path.join(project_root, 'osm', 'map (1).osm')  # 替换为实际文件名

    # 初始化路网处理器
    processor = RoadNetwork(osm_path)

    # 处理数据
    processor.process_road_network()
    visualize_network(processor)
    odr = processor.xodr_generator()

    odr.adjust_roads_and_lanes()
    odr.write_xml("road_network.xodr")

if __name__ == "__main__":
    main()