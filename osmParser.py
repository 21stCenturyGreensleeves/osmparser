import xml.etree.ElementTree as ET
from collections import defaultdict

import matplotlib.pyplot as plt
from matplotlib import collections as mc
import numpy as np


CAR_HIGHWAY_TYPES = {
    'motorway', 'motorway_junction', 'motorway_link',
    'trunk', 'trunk_link',
    'primary', 'primary_link',
    'secondary', 'secondary_link'
    'tertiary', 'tertiary',
    'unclassified', 'residential', 'service'
}

class OSMNode:
    def __init__(self, node_id, lat, lon):
        self.id = node_id
        self.lat = float(lat)
        self.lon = float(lon)
        self.ways = set() #  记录这个节点所在的所有道路
        self.is_junction = False
        self.is_endpoint = False

class OSMWay:
    def __init__(self, way_id, tags, nodes):
        self.id = way_id
        self.tags = tags
        self.node_ids = nodes


class RoadSegment:
    def __init__(self, segment_id, start_id, end_id, segment_nodes, owned_way):
        self.id = segment_id
        self.start = start_id
        self.end = end_id
        self.nodes = segment_nodes  # 只存id

        self.owned = owned_way

class GeometrySegment:
    def __init__(self, seg_type, params, start_node, end_node):
        self.type = seg_type
        self.params = params
        self.start = start_node
        self.end = end_node
        self.length = self._calculate_length()

    def _calculate_length(self):
        if self.type == 'line':
            return self.params['length']
        elif self.type == 'arc':
            return abs(self.params['curvature']) * self.params['angle']

class RoadNetwork:
    def __init__(self, osm_file):
        self.osm_file = osm_file
        self.nodes = {}
        self.ways = {}
        self.junctions = []
        self.road_segments = {}

    def parse_osm(self):
        tree = ET.parse(self.osm_file)
        root = tree.getroot()

        for node in root.findall('node'):
            node_id = node.attrib['id']
            self.nodes[node_id] = OSMNode(
                node_id,
                float(node.attrib['lat']),
                float(node.attrib['lon'])
            )
        # 记录所有nodes到roadNetwork的nodes中

        for way in root.findall('way'):

            tags = {}
            for tag in way.findall('tag'):
                tags[tag.attrib['k']] = tag.attrib.get('v', '')
            if tags.get('highway') not in CAR_HIGHWAY_TYPES:
                continue

            way_id = way.attrib['id']
            way_nodes = [nd.attrib['ref'] for nd in way.findall('nd')]

            for nd in way.findall('nd'):
                node_ref = nd.attrib['ref']
                if node_ref in self.nodes:
                    node = self.nodes[node_ref]
                    node.ways.add(way_id)
                    if len(node.ways) >= 2:
                        node.is_junction = True
            # 便利所有way，对于每个way中的所有node处理其ways中的内容
            # 如果ways数量超过2则判定这个点为junction

            self.ways[way.attrib['id']] = OSMWay(way.attrib['id'], tags, way_nodes)
            # 记录way中所有的nd的id记录在list中，创建一个OSMWay的实例，放入ways

    def mark_endpoints(self):
        for way in self.ways.values():
            if len(way.node_ids) == 0:
                continue
            start_node = self.nodes.get(way.node_ids[0])
            end_node = self.nodes.get(way.node_ids[-1])

            if start_node and not start_node.is_junction:
                start_node.is_endpoint = True
            if end_node and not end_node.is_junction:
                end_node.is_endpoint = True

    def split_way_into_segment(self):
        segment_id = 10000

        for way in self.ways.values():
            segment_nodes = []
            for node_id in way.node_ids:
                segment_nodes.append(node_id)
                node = self.nodes[node_id]
                # 在交叉点或者端点的地方分割
                if node.is_junction or node.is_endpoint:
                    if len(segment_nodes) > 1:
                        segment = RoadSegment(segment_id,
                                              segment_nodes[0],
                                              segment_nodes[-1],
                                              segment_nodes.copy(),
                                              way)
                        self.road_segments[segment_id] = segment
                        segment_id += 1
                    segment_nodes = [node_id]

    def process_road_network(self):
        self.parse_osm()
        self.mark_endpoints()
        self.split_way_into_segment()

    def find_connected_components(self):
        parent = {}  # 记录seg节点的父节点

        def find(seg_id):  # 如果在parent中一个seg的父节点不是它本身，就一直往回追溯直到找到其所属于的节点
            while parent[seg_id] != seg_id:
                parent[seg_id] = parent[parent[seg_id]]
                seg_id = parent[seg_id]
            return seg_id

        def union(seg1, seg2):
            root1 = find(seg1)
            root2 = find(seg2)
            if root1 != root2:
                parent[root2] = root1

        for seg_id in self.road_segments:
            parent[seg_id] = seg_id

        node_to_segments = defaultdict(list)
        for seg_id, seg in self.road_segments.items():
            node_to_segments[seg.start].append(seg_id)
            node_to_segments[seg.end].append(seg_id)
        # 记录下对于所有key 节点属于哪些value(list) segment
        # 合并共享节点的路段
        for seg_list in node_to_segments.values():
            if len(seg_list) > 1:
                anchor = seg_list[0]
                for seg in seg_list[1:]:
                    union(anchor, seg)

        # 生成最终组件映射
        return {seg_id: find(seg_id) for seg_id in self.road_segments}

    def update_junctions_after_filtering(self):
        # 由于第一次剔除的时候会有一些节点不该出现，所以使用这个方法在剔除后添加节点
        for node in self.nodes.values():
            node.is_junction = False
        node_counter = defaultdict(int)
        for seg in self.road_segments.values():
            node_counter[seg.start] += 1
            node_counter[seg.end] += 1

        for node_id, count in node_counter.items():
            if count >= 2:
                node = self.nodes.get(node_id)
                if node:
                    node.is_junction = True
                    self.junctions.append(node_id)

    def filter_isolated_components(self, min_segment_size = 2):
        components = self.find_connected_components()

        # 统计每个组件的大小
        component_sizes = {}
        for seg_id, component_id in components.items():
            component_sizes[component_id] = component_sizes.get(component_id, 0) + 1

        # 确定需要保留的组件ID，删除其余的所有ID
        valid_components = {
            cid for cid, size in component_sizes.items()
            if size > min_segment_size
        }
        self.road_segments = {
            seg_id: seg
            for seg_id, seg in self.road_segments.items()
            if components[seg_id] in valid_components
        }

        # 重新计算交叉点
        self.update_junctions_after_filtering()

    def visualize_network(self, output_file="road_network.pdf"):
        if not self.road_segments:
            print("No segments to visualize.")
            return

        # 初始化绘图
        plt.figure(figsize=(12, 12), dpi=300)  # 改用正方形画布
        ax = plt.gca()

        # 设置等比例坐标轴
        ax.set_aspect('equal')  # 关键修改：保证坐标比例一致
        plt.axis('off')  # 关闭所有坐标轴元素

        # 生成更易区分的颜色方案
        colors = plt.cm.gist_ncar(np.linspace(0, 1, len(self.road_segments)))

        # 使用LineCollection优化绘制性能
        all_lines = []
        for idx, segment in enumerate(self.road_segments.values()):
            coords = []
            for node_id in segment.nodes:
                if node_id in self.nodes:
                    node = self.nodes[node_id]
                    coords.append([node.lon, node.lat])

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
        if self.junctions:
            junc_coords = np.array([
                (self.nodes[jid].lon, self.nodes[jid].lat)
                for jid in self.junctions if jid in self.nodes
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


if __name__ == "__main__":
    road_network = RoadNetwork('osm/map (1).osm')
    road_network.process_road_network()
    road_network.filter_isolated_components(3)
    road_network.visualize_network()
