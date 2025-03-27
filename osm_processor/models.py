import math

from scenariogeneration import xodr

from utils import calculate_curvature
import xml.etree.ElementTree as ET
import numpy as np
from pyproj import Proj
from collections import defaultdict
from typing import Any

GEOMETRY_ID = 100000
SEGMENT_ID = 10000
MAX_SPEED = 40
LANES_NUMBER = 2
CAR_HIGHWAY_TYPES = {
    'motorway', 'motorway_junction', 'motorway_link',
    'trunk', 'trunk_link',
    'primary', 'primary_link',
    'secondary', 'secondary_link',
    'tertiary',
    'unclassified', 'residential', 'service'
}


class OSMNode:
    """
    记录节点信息
    """
    id: Any
    lat: float
    lon: float
    x: float
    y: float
    ways: Any
    is_junction: bool
    is_endpoint: bool
    adjacent_segment: list
    def __init__(self, node_id, lat, lon, x, y):
        self.id = node_id
        self.lat = float(lat)
        self.lon = float(lon)
        # 适应matplotlib的坐标系的改动
        self.x = x
        self.y = y
        self.ways = set()  # 记录这个节点所在的所有道路，nodes中的ways是无分的
        self.is_junction = False
        self.is_endpoint = False
        # TODO 是否要记录这个点相邻的路段信息？
        self.adjacent_segment = []


class RoadNetwork:
    osm_file = str
    nodes = map
    ways = map
    junction = list
    road_segment = map
    connections = defaultdict(list)
    def __init__(self, osm_file):
        self.osm_file = osm_file
        self.nodes = {}
        self.ways = {}  # ways的顺序也无关紧要
        self.junctions = []
        self.road_segments = {}
        self.connections = defaultdict(list)

        self.utm_zone = None
        self.proj = None

    # 获取UTM带号
    def _determine_utm_zone(self, longitude):
        return math.floor(longitude / 6.0) + 31

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

        self.utm_zone = self._determine_utm_zone(center_lon)
        print(f"所处utm带为{self.utm_zone}")
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
            way_nodes = [nd.attrib['ref'] for nd in way.findall('nd')]  # TODO（问题在于nodes的记录顺序是否和osm中相同？）

            for nd in way.findall('nd'):
                node_ref = nd.attrib['ref']
                if node_ref in self.nodes:
                    node = self.nodes[node_ref]
                    node.ways.add(way_id)
                    if len(node.ways) >= 2:
                        node.is_junction = True

            self.ways[way.attrib['id']] = OSMWay(way.attrib['id'], tags, way_nodes)

    # 直接将每个way的endpoint标记为endpoint就可以，不影响其他操作
    def mark_endpoints(self):
        for way in self.ways.values():
            if len(way.node_ids) == 0:
                continue
            start_node = self.nodes.get(way.node_ids[0])
            end_node = self.nodes.get(way.node_ids[-1])

            start_node.is_endpoint = True
            end_node.is_endpoint = True

    def update_roadnetwork_segments(self):
        for way in self.ways.values():
            for segment in way.ways_road_segments:
                self.road_segments[segment.id] = segment

    def process_road_network(self):
        self.parse_osm()  # 初始化节点，道路
        self.mark_endpoints()  # 标记终止节点
        segment_id = SEGMENT_ID
        for way in self.ways.values():  # 将所有道路分割为segment
            segment_id = way.split_way_into_segment(self.nodes, segment_id)
        self.update_roadnetwork_segments()  # 在路网中记录segments

        for segment in self.road_segments.values():
            segment.compute_geometry_segments(self.nodes)  # 记录所有segment中的curvatures
        for way in self.ways.values():
            for segment in way.ways_road_segments:
                segment.split_into_geom()

    def get_junction_segments(self):
        """
        一个dict，node_id -> 与这个node相连的所有segment
        """
        junction_segments = defaultdict(list)
        for seg in self.road_segments.values():
            if seg.start in self.junctions:
                junction_segments[seg.start].append(seg)
            if seg.end in self.junctions:
                junction_segments[seg.end].append(seg)
        return junction_segments


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
        for way in self.ways.values():
            for node_in_way in way.node_ids:
                if self.nodes[node_in_way].is_junction:
                    way.junctions.append(node_in_way)

    def filter_isolated_components(self, min_segment_size=2):
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

    def xodr_generator(self):
        odr = xodr.OpenDrive("my_road_network")
        road_id = 1
        junction_id = 100
        junctions = defaultdict()  # id -> CommonJunctionCreator
        junctions_with_seg = defaultdict(list)  # id -> seg_id 用于add_connection
        for way in self.ways.values():
            for seg in way.ways_road_segments:  # 再次循环中创建所有的road并且添加到odr中
                # 首先生成这段路段的所有道路
                geometry = []
                for geom_seg in seg.geometry_segments:
                    geom = None
                    if geom_seg.type_ == 'straight':
                        geom = xodr.Line(geom_seg.length)
                    elif geom_seg.type_ == 'arc':
                        geom = xodr.Arc(geom_seg.start_curvature, geom_seg.length)
                    elif geom_seg.type_ == 'spiral':
                        start_k, end_k = geom_seg.start_curvature, geom_seg.end_curvature
                        geom = xodr.Spiral(start_k, end_k, geom_seg.length)
                    geometry.append(geom)
                road = xodr.create_road(geometry, id=road_id, left_lanes=way.lanes, right_lanes=way.lanes)
                odr.add_road(road)
                seg.road_id = road_id  # 记录 road_id

                # TODO：链接segment内的所有道路
                # 然后研究这段路段的 首 尾 节点是否为junction，连接道路

                start_next_node = self.nodes[seg.nodes[1]]
                end_prev_node = self.nodes[seg.nodes[-2]]
                start_node = self.nodes[seg.start]
                end_node = self.nodes[seg.end]
                jc = None
                if start_node.is_junction:
                    if seg.start not in junctions:
                        jc = xodr.CommonJunctionCreator(junction_id, name="fuzhoudragon")
                        odr.add_junction_creator(jc)
                        junctions[seg.start] = jc
                        junction_id+=10
                    elif seg.start in junctions:
                        jc = junctions[seg.start]
                    jc.add_incoming_road_cartesian_geometry(road,
                                                            x=start_next_node.x - start_node.x,
                                                            y=start_next_node.y - start_node.y,
                                                            heading=math.atan2(
                                                                            start_next_node.y - start_node.y,
                                                                            start_next_node.x - start_node.x),
                                                            road_connection="predecessor")
                    print(f"cartesian situation:{start_next_node.x - start_node.x}, {start_next_node.y - start_node.y}")
                    if jc.id not in junctions_with_seg:
                        junctions_with_seg[jc.id].append(road_id)
                    else:
                        for other_seg in junctions_with_seg[jc.id]:
                            jc.add_connection(road_id, other_seg)
                            print(f"{road_id}, {other_seg} has been connected!")
                    junctions_with_seg[jc.id].append(road_id)
                if end_node.is_junction:
                    if seg.end not in junctions:
                        jc = xodr.CommonJunctionCreator(junction_id, name="fuzhoudragon")
                        junctions[seg.end] = jc
                        odr.add_junction_creator(jc)
                        junction_id += 10
                    elif seg.end in junctions:
                        jc = junctions[seg.end]
                    jc.add_incoming_road_cartesian_geometry(road,
                                                            x=end_node.x - end_prev_node.x,
                                                            y=end_node.y - end_prev_node.y,
                                                            heading=math.atan2(
                                                                            end_node.x - end_prev_node.x,
                                                                            end_node.y - end_prev_node.y),
                                                            road_connection="successor")
                    print(f"cartesian situation:{start_next_node.x - start_node.x}, {start_next_node.y - start_node.y}")
                    if jc.id not in junctions_with_seg:
                        junctions_with_seg[jc.id].append(road_id)
                    else:
                        for other_seg in junctions_with_seg[jc]:
                            jc.add_connection(road_id, other_seg)
                            print(f"{road_id}, {other_seg} has been connected!")
                    junctions_with_seg[jc].append(road_id)
                road_id += 1
        print("xodr_generation completed")
        return odr


class JunctionNode:
    def __init__(self, center_node):
        self.center_node = center_node
        self.junctions_info = []

# Node记录
class OSMWay:
    """
    记录道路信息，包含道路的车道数量，限速，layers表示的高度层级，是否为单行道，是否为隧道
    """
    id: Any
    tags = Any
    node_ids = Any
    ways_road_segments = list

    junctions = list

    oneway = bool
    lanes = int
    layer = int
    max_speed = float
    tunnel = bool
    def __init__(self, way_id, tags, nodes):
        self.id = way_id
        self.tags = tags
        self.node_ids = nodes
        self.ways_road_segments = []  # 记录的顺序是从头到尾吗？ 是

        self.junctions = []

        self.oneway = False
        self.lanes = LANES_NUMBER
        self.layer = 1
        self.max_speed = MAX_SPEED
        self.tunnel = False

        if 'lanes' in self.tags:
            self.lanes = int(self.tags['lanes'])
        if 'layer' in self.tags:
            self.layer = int(self.tags['layer'])
        if 'maxspeed' in self.tags:
            self.max_speed = float(self.tags['maxspeed'])
        if 'oneway' in self.tags and self.tags['oneway'] == 'yes':
            self.oneway = True
        if 'tunnel' in self.tags and self.tags['tunnel'] == 'yes':
            self.tunnel = True

    def split_way_into_segment(self, nodes_dict, segment_id):
        segment_nodes = []
        for node_id in self.node_ids:
            segment_nodes.append(node_id)
            node = nodes_dict[node_id]
            # 在交叉点或者端点的地方分割
            if node.is_junction or node.is_endpoint:
                if len(segment_nodes) > 1:
                    segment = RoadSegment(segment_id,
                                          segment_nodes[0],
                                          segment_nodes[-1],
                                          segment_nodes.copy(),
                                          self.id)
                    self.ways_road_segments.append(segment)
                    segment_id += 1
                segment_nodes = [node_id]
        if len(segment_nodes) > 1:
            segment = RoadSegment(segment_id, segment_nodes[0], segment_nodes[-1], segment_nodes.copy(), self.id)
            self.ways_road_segments.append(segment)
            segment_id += 1
        return segment_id

class RoadSegment:
    """
    将道路按junction与segment划分为多条路段
    """
    def __init__(self, segment_id, start_id, end_id, segment_nodes, owned_way_id):
        self.id = segment_id
        self.start = start_id
        self.end = end_id
        self.nodes = segment_nodes  # 只存id
        self.owned_way_id = owned_way_id
        self.road_id = None

        self.geometry_segments = []
        self.predecessor = None
        self.successor = None
        self._precomputed_lengths = None
        self.curvatures = None


    def print_curvatures(self):
        for curvature in self.curvatures:
            print(f'{curvature} \n')

    def printGeometryInfo(self):
        for segment in self.geometry_segments:
            segment.print_info()

    def compute_geometry_segments(self, nodes_dict):
        """"
        传入network中的nodes合集
        """
        coords = np.array([(nodes_dict[node_id].x, nodes_dict[node_id].y) for node_id in self.nodes])
        for x, y in coords:
            print(f"x: {x}, y: {y}")
        # 如果坐标总数小于2则无法成立
        if len(coords) < 2:
            return
        dx = np.diff(coords[:, 0])
        dy = np.diff(coords[:, 1])
        self._precomputed_lengths = np.hypot(dx, dy).tolist()

        total_length = sum(self._precomputed_lengths)
        print(f"整条路的长度为: {total_length:.2f} 米")
        # 获得这个segment上所有的曲率
        curvatures = self._compute_curvatures(coords)
        self.curvatures = curvatures

    def _compute_curvatures(self, coords):
        n = len(coords)
        curvatures = np.zeros(n)

        if n < 3:
            return curvatures.tolist()  # 不足3点无法计算曲率

        for i in range(1, n - 1):
            curvatures[i] = calculate_curvature(coords[i - 1], coords[i], coords[i + 1])

        return curvatures.tolist()

    def fit_line(self, k, indices):
        """
        对曲率序列 k 在指定索引范围 indices 上进行线性回归。
        返回斜率 a、截距 b 和最大绝对误差 max_error。
        """
        x = np.array(indices)
        y = np.array([k[i] for i in indices])
        a, b = np.polyfit(x, y, 1)  # 线性拟合，得到 a 和 b
        predicted = a * x + b
        max_error = np.max(np.abs(y - predicted))
        return a, b, max_error

    def calculate_geom_length(self, start, end):
        return sum(self._precomputed_lengths[start:end])

    def split_into_geom(self, epsilon=0.0001, delta=0.001, eta=0.001):
        """
        将曲率序列 k 分割成无缝连接的直线、弧线和螺旋线段，并提供曲率信息。

        参数：
        - k: 曲率值列表
        - epsilon: 拟合误差阈值
        - delta: 斜率阈值，区分恒定和线性变化
        - eta: 曲率阈值，区分直线和弧线

        返回：
        - segments: 列表，每个元素为 (start_idx, end_idx, type, curvature_info)
          - type: 'straight', 'arc', 'spiral'
          - curvature_info:
            - 直线: 0
            - 弧线: 曲率值 b
            - 螺旋线: (start_curvature, end_curvature)
        """
        n = len(self.curvatures)
        segments = []
        i = 0

        while i < n:
            # 如果只剩一个点，单独处理
            if i == n - 1:
                type_ = 'straight'
                curvature_info = 0
                segments.append((i, i, type_, curvature_info, self.calculate_geom_length(i, n-1)))
                break

            # 尝试从 i 开始扩展线段
            for j in range(i + 1, n):
                indices = list(range(i, j + 1))
                a, b, max_error = self.fit_line(self.curvatures, indices)

                # 如果拟合误差小于阈值，继续扩展
                if max_error < epsilon:
                    continue
                else:
                    # 无法扩展到 j，使用前一个 j
                    if j > i + 1:
                        prev_indices = list(range(i, j))
                        a, b, max_error = self.fit_line(self.curvatures, prev_indices)
                        if abs(a) < delta:  # 曲率近似恒定
                            if abs(b) < eta:
                                type_ = 'straight'
                                curvature_info = 0
                            else:
                                type_ = 'arc'
                                curvature_info = b
                        else:  # 曲率线性变化
                            type_ = 'spiral'
                            start_curvature = a * i + b
                            end_curvature = a * (j - 1) + b
                            curvature_info = (start_curvature, end_curvature)
                        segments.append((i, j - 1, type_, curvature_info, self.calculate_geom_length(i, j-1)))
                        i = j - 1  # 下一个线段从 j-1 开始，确保无缝连接
                        break
                    else:
                        # 无法找到至少两个点的线段，单独处理 i
                        type_ = 'straight' if abs(self.curvatures[i]) < eta else 'arc'
                        curvature_info = 0 if type_ == 'straight' else self.curvatures[i]
                        segments.append((i, i, type_, curvature_info, self.calculate_geom_length(i, j-1)))
                        i += 1
                        break
            else:
                # j 到达末尾，取剩余部分
                indices = list(range(i, n))
                a, b, max_error = self.fit_line(self.curvatures, indices)
                if abs(a) < delta:
                    if abs(b) < eta:
                        type_ = 'straight'
                        curvature_info = 0
                    else:
                        type_ = 'arc'
                        curvature_info = b
                else:
                    type_ = 'spiral'
                    start_curvature = a * i + b
                    end_curvature = a * (n - 1) + b
                    curvature_info = (start_curvature, end_curvature)
                segments.append((i, n - 1, type_, curvature_info, self.calculate_geom_length(i, n-1)))
                break
        self.geometry_segments = []
        """
        填充路段的geometry_segment TODO：目前是将一整段都当作
        """
        for start, end, type_, curvature_info, length in segments:
            geom_seg = None
            if type_ == 'straight':
                geom_seg = Line(id=GEOMETRY_ID,
                                       start_node_id=self.nodes[start],
                                       end_node_id=self.nodes[end],
                                       node_ids=self.nodes[start:end+1],
                                       length=length)
            elif type_ == 'arc':
                geom_seg = Arc(id=GEOMETRY_ID,
                                start_node_id=self.nodes[start],
                                end_node_id=self.nodes[end],
                                node_ids=self.nodes[start:end + 1],
                                length=length,
                                curvature = curvature_info)
            elif type_ == 'spiral':
                geom_seg = Spiral(id=GEOMETRY_ID,
                               start_node_id=self.nodes[start],
                               end_node_id=self.nodes[end],
                               node_ids=self.nodes[start:end + 1],
                               length=length,
                               start_curvature=curvature_info[0],
                               end_curvature=curvature_info[1] )
            self.geometry_segments.append(geom_seg)
        return segments


class GeometrySegment:
    """
    将road segment分割为多段Geometry Segment便于xodr道路的生成，
    """

    def __init__(self, id, start_node_id, end_node_id, node_ids, length, start_curvature, end_curvature, type_):
        self.geom_id = id
        self.start = start_node_id
        self.end = end_node_id
        self.node_ids = node_ids
        self.length = length
        self.start_curvature = start_curvature
        self.end_curvature = end_curvature
        self.type_ = type_

    def print_info(self):
        """打印几何段基础信息"""
        print(
            f"Geometry Type: {self.__class__.__name__}\n"
            f"Nodes Count: {len(self.node_ids)}\n"
            f"Start Node: {self.start}\n"
            f"End Node: {self.end}\n"
            f"Length: {self.length:.2f}m\n"
            f"Start Curvature: {self.start_curvature:.4f}\n"
            f"End Curvature: {self.end_curvature:.4f}\n"
            "-----------------------"
        )
class Line(GeometrySegment):
    """直线段，曲率恒为 0"""
    def __init__(self, id, start_node_id, end_node_id, node_ids, length):
        super().__init__(
            id=id,
            start_node_id=start_node_id,
            end_node_id=end_node_id,
            node_ids=node_ids,
            length=length,
            start_curvature=0.0,  # 直线曲率为 0
            end_curvature=0.0,
            type_="straight"
        )
class Arc(GeometrySegment):
    """圆弧段，曲率恒定"""
    def __init__(self, id, start_node_id, end_node_id, node_ids, length, curvature):
        super().__init__(
            id=id,
            start_node_id=start_node_id,
            end_node_id=end_node_id,
            node_ids=node_ids,
            length=length,
            start_curvature=curvature,
            end_curvature=curvature,
            type_="arc"
        )
class Spiral(GeometrySegment):
    """螺旋线段，曲率线性变化"""
    def __init__(self, id, start_node_id, end_node_id, node_ids, length, start_curvature, end_curvature):
        super().__init__(
            id=id,
            start_node_id=start_node_id,
            end_node_id=end_node_id,
            node_ids=node_ids,
            length=length,
            start_curvature=start_curvature,  # 起始曲率
            end_curvature=end_curvature,      # 结束曲率（与起始不同）
            type_="spiral"
        )

