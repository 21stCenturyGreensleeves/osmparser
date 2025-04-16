import os


from utils import visualize_network
from models import RoadNetwork
import matplotlib.pyplot as plt
from scenariogeneration import xodr

def main():
    # 获取OSM文件路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)
    osm_path = os.path.join(project_root, 'osm', '2JunctionExample.osm')  # 替换为实际文件名

    # 初始化路网处理器
    processor = RoadNetwork(osm_path)


    # 处理数据
    processor.process_road_network()
    processor.filter_isolated_components(0)
    visualize_network(processor)
    odr = processor.xodr_generator()

    odr.adjust_roads_and_lanes()
    odr.write_xml("road_network.xodr")

if __name__ == "__main__":
    main()