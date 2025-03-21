import xml.etree.ElementTree as ET
from collections import defaultdict
import numpy as np
from pyproj import Proj

from osm_processor.models import OSMNode, OSMWay, RoadSegment

CAR_HIGHWAY_TYPES = {
    'motorway', 'motorway_junction', 'motorway_link',
    'trunk', 'trunk_link',
    'primary', 'primary_link',
    'secondary', 'secondary_link',
    'tertiary',
    'unclassified', 'residential', 'service'
}

class RoadNetwork:
    def __init__(self, osm_file):
        self.osm_file = osm_file
        self.nodes = {}
        self.ways = {} # ways的顺序也无关紧要
        self.junctions = []
        self.road_segments = {}
        self.connections = defaultdict(list)

        self.utm_zone = None
        self.proj = None

    # 获取UTM带号
    def _determine_utm_zone(self, longitude):
        return int((longitude + 180) // 6) + 1

    def parse_osm(self):
        tree = ET.parse(self.osm_file)
        root = tree.getroot()

        lats, lons = [], []
        for node in root.findall('node'):
            lats.append(float(node.attrib['lat']))
            lons.append(float(node.attrib['lon']))

        if not lats or not lons:
            raise ValueError("**NO NODES IN OSM FILE**")

        center_lat = np.mean(lats)
        center_lon = np.mean(lons)

        self.utm_zone = self._determine_utm_zone(center_lat)
        hemisphere = 'north' if center_lat >= 0 else 'south'
        self.proj = Proj(proj='utm', zone=self.utm_zone, ellps='WGS84', south=(hemisphere == 'south'))

        # 处理所有的node数据并且记录其lat，lon，x，y 后续只使用x与y
        for node in root.findall('node'):
            node_id = node.attrib['id']
            lat = float(node.attrib['lat'])
            lon = float(node.attrib['lon'])
            x, y = self.proj(lon, lat)

            self.nodes[node_id] = OSMNode(node_id, lat, lon, x, y)

        # 处理所有的道路信息，记录所有的tags，如果道路的类型部位预定的类型则不计入以下操作，记录道路的id，nodes
        for way in root.findall('way'):
            tags = {}
            for tag in way.findall('tag'):
                tags[tag.attrib['k']] = tag.attrib.get('v', '')
            if tags.get('highway') not in CAR_HIGHWAY_TYPES:
                continue

            way_id = way.attrib['id']
            way_nodes = [nd.attrib['ref'] for nd in way.findall('nd')] # TODO（问题在于nodes的记录顺序是否和osm中相同？）

            for nd in way.findall('nd'):
                node_ref = nd.attrib['ref']
                if node_ref in self.nodes:
                    node = self.nodes[node_ref]
                    node.ways.add(way_id)
                    if len(node.ways) >= 2:
                        node.is_junction = True

            self.ways[way.attrib['id']] = OSMWay(way.attrib['id'], tags, way_nodes)
            # 记录way中所有的nd的id记录在list中，创建一个OSMWay的实例，放入ways

    # 直接将每个way的endpoint标记为endpoint就可以，不影响其他操作
    def mark_endpoints(self):
        for way in self.ways.values():
            if len(way.node_ids) == 0:
                continue
            start_node = self.nodes.get(way.node_ids[0])
            end_node = self.nodes.get(way.node_ids[-1])

            start_node.is_endpoint = True
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
            # 是否可能会出现结尾endpoint没有被标记的情况？保险
            if len(segment_nodes) > 1:
                segment = RoadSegment(segment_id,
                                        segment_nodes[0],
                                        segment_nodes[-1],
                                        segment_nodes.copy(),
                                        way)
                self.road_segments[segment_id] = segment
                segment_id += 1

    def process_road_network(self):
        self.parse_osm()
        self.mark_endpoints()
        self.split_way_into_segment()

        for segment in self.road_segments.values():
            segment.compute_geometry_segments(self.nodes)

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

