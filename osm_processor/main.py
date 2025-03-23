import os
from utils import visualize_network
from models import RoadNetwork

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
    for way_id, way in processor.ways.items():
        print(f"way_id : {way_id}")
        for seg in way.ways_road_segments:
            print(seg.id)
    for nodes in processor.nodes.values():
        if nodes.is_junction or nodes.is_endpoint:
            print("yes")


if __name__ == "__main__":
    main()