import os
from network import RoadNetwork
from osm_processor.xodr_generator import XODRGenerator
from utils import visualize_network

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
    # 生成XODR
    generator = XODRGenerator()
    odr = generator.generate(processor)

    # 保存文件
    output_path = os.path.join("output", "generated_road.xodr")
    odr.write_xml(output_path)

if __name__ == "__main__":
    main()