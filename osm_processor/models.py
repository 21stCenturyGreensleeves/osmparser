import math

from scenariogeneration import xodr

from sg_example import planview
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
Pi = 3.14
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
        self.adjacent_segment = []

class RoadNetwork:
    """
    储存了网络中所有的 节点 路口 道路 ？
    """
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

        self.buildings = {}

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
            if 'building' in tags:
                building_id = tags['building']
                node_refs = [nd.attrib['ref'] for nd in way.findall('nd')]
                self.buildings[building_id] = Building(building_id, node_refs)
                continue
            elif tags.get('highway') not in CAR_HIGHWAY_TYPES:
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

    def multi_geom_function(self):
        """
        使用多条几何线拟合、组合后产生的曲线组合
        """
        for segment in self.road_segments.values():
            segment.compute_geometry_segments(self.nodes)  # 记录所有segment中的curvatures
        for way in self.ways.values():
            for segment in way.ways_road_segments:
                segment.split_into_geom()

    def poly3_function(self):
        for way in self.ways.values():
            for segment in way.ways_road_segments:
                segment.produce_poly3(self.nodes)

    def process_road_network(self):
        self.parse_osm()  # 初始化节点，道路
        self.mark_endpoints()  # 标记终止节点
        segment_id = SEGMENT_ID
        for way in self.ways.values():  # 将所有道路分割为segment
            segment_id = way.split_way_into_segment(self.nodes, segment_id)
        self.update_roadnetwork_segments()  # 在路网中记录segments
        self.multi_geom_function()


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
        road_id = 10000
        junction_id = 1
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
                    elif geom_seg.type_ == 'poly3':
                        geom = xodr.ParamPoly3(
                            geom_seg.au, geom_seg.bu, geom_seg.cu, geom_seg.du,
                            geom_seg.av, geom_seg.bv, geom_seg.cv, geom_seg.dv,
                            length=geom_seg.length,
                            prange="normalized"  # 使用 u∈[0,1]
                        )
                    geometry.append(geom)
                    # 创建一个geometry
                    # planview 的 h_start为起始角，0为东，pi/2为北
                planview = xodr.PlanView(x_start=None, y_start=None, h_start = None)
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
                        jc = xodr.CommonJunctionCreator(junction_id, name=f"fuzhoudragon{junction_id}", startnum=junction_id)
                        junctions[seg.start] = jc
                        junction_id=junction_id+10
                    elif seg.start in junctions:
                        jc = junctions[seg.start]
                    angle_pre = math.atan2(start_next_node.y - start_node.y, start_next_node.x - start_node.x) - Pi
                    jc.add_incoming_road_cartesian_geometry(road,
                                                            x=start_next_node.x - start_node.x,
                                                            y=start_next_node.y - start_node.y,
                                                            heading=angle_pre,
                                                            road_connection="predecessor")
                    # print(f"road_id:{road_id}:cartesian situation:{start_next_node.x - start_node.x}, {start_next_node.y - start_node.y}, angle: {angle_pre / 3.14} Pi")
                    if jc.id not in junctions_with_seg:
                        junctions_with_seg[jc.id].append(road_id)
                    else:
                        for other_seg in junctions_with_seg[jc.id]:
                            jc.add_connection(road_id, other_seg)
                            print(f"{road_id}, {other_seg} has been connected in {jc.id} junction:startpoint")
                        junctions_with_seg[jc.id].append(road_id)


                if end_node.is_junction:
                    if seg.end not in junctions:
                        jc = xodr.CommonJunctionCreator(junction_id, name=f"fuzhoudragon{junction_id}", startnum=junction_id)
                        junctions[seg.end] = jc
                        junction_id = junction_id + 10
                    elif seg.end in junctions:
                        jc = junctions[seg.end]
                    angle_suc = math.atan2(end_prev_node.y - end_node.y, end_prev_node.x - end_node.x) - Pi
                    jc.add_incoming_road_cartesian_geometry(road,
                                                            x=(end_prev_node.x - end_node.x),
                                                            y=(end_prev_node.y - end_node.y),
                                                            heading=angle_suc,
                                                            road_connection="successor")
                    # print(f"road_id:{road_id}:cartesian situation:{start_next_node.x - start_node.x}, {start_next_node.y - start_node.y}, angle: {angle_suc / Pi} Pi")
                    if jc.id not in junctions_with_seg:
                        junctions_with_seg[jc.id].append(road_id)
                    else:
                        for other_seg in junctions_with_seg[jc.id]:
                            jc.add_connection(road_id, other_seg)
                            print(f"{road_id}, {other_seg} has been connected! in {jc.id} junction:endpoint")
                        junctions_with_seg[jc.id].append(road_id)
                road_id += 1
        for jun in junctions.values():
            odr.add_junction_creator(jun)
        return odr

class Building:
    def __init__(self, id, nodes):
        self.id = id
        self.nodes = nodes

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
        对曲率序列 k 在指定索引范围 indices 上进行线性回归
        返回斜率 a、截距 b 和最大绝对误差 max_error 进行判断是否类型继续\已经改变
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
        将曲率序列 k 分割成无缝连接的直线、弧线和螺旋线段，并提供曲率信息
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
        # TODO 目前是将一整段segment直接转变为road，后续是否需要改变？
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

    def produce_poly3(self, nodes_dict):
        coords = np.array([(nodes_dict[node_id].x, nodes_dict[node_id].y) for node_id in self.nodes])
        if len(coords) < 4:  # 至少需要4个点以稳定拟合
            return

        # 计算实际总长度（基于UTM坐标差）
        dx = np.diff(coords[:, 0])
        dy = np.diff(coords[:, 1])
        segment_lengths = np.hypot(dx, dy)
        total_length = np.sum(segment_lengths)

        # 参数t基于累积弧长（归一化到[0,1]）
        t = np.insert(np.cumsum(segment_lengths), 0, 0) / total_length

        # 确保端点精确
        A = np.vstack([t ** 3, t ** 2, t, np.ones_like(t)]).T
        A = np.vstack([A, [1, 1, 1, 1], [0, 0, 0, 1]])  # 添加u=1和u=0的约束

        # 解方程组
        try:
            coeffs_x, _, _, _ = np.linalg.lstsq(A, np.append(coords[:, 0], [coords[-1, 0], coords[0, 0]]), rcond=None)
            coeffs_y, _, _, _ = np.linalg.lstsq(A, np.append(coords[:, 1], [coords[-1, 1], coords[0, 1]]), rcond=None)
        except np.linalg.LinAlgError:
            return

        # 确保端点精确
        coeffs_x[-1] = coords[0, 0]  # dU = 起始点x
        coeffs_y[-1] = coords[0, 1]  # dV = 起始点y
        coeffs_x[2] = coords[-1, 0] - (coeffs_x[0] + coeffs_x[1] + coeffs_x[3])  # 调整cU

        geom_seg = Poly3(
            id=GEOMETRY_ID,
            start_node_id=self.start,
            end_node_id=self.end,
            node_ids=self.nodes,
            length=total_length,  # 使用实际坐标计算的总长度
            au=coeffs_x[0], bu=coeffs_x[1], cu=coeffs_x[2], du=coeffs_x[3],
            av=coeffs_y[0], bv=coeffs_y[1], cv=coeffs_y[2], dv=coeffs_y[3]
        )
        self.geometry_segments = [geom_seg]


class GeometrySegment:
    """
    将road segment分割为多段Geometry Segment便于xodr道路的生成，
    """

    def __init__(self, id, start_node_id, end_node_id, node_ids, length,
                 start_curvature=0.0, end_curvature=0.0,
                 type_="None",
                 au=0.0,
                 bu=0.0,
                 cu=0.0,
                 du=0.0,
                 av=0.0,
                 bv=0.0,
                 cv=0.0,
                 dv=0.0):
        self.geom_id = id
        self.start = start_node_id
        self.end = end_node_id
        self.node_ids = node_ids
        self.length = length
        self.start_curvature = start_curvature
        self.end_curvature = end_curvature
        self.type_ = type_
        self.au = au
        self.bu = bu
        self.cu = cu
        self.du = du
        self.av = av
        self.bv = bv
        self.cv = cv
        self.dv = dv
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
    """圆弧，曲率恒定"""
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
    """螺旋线，曲率从起始曲率到结束曲律进行线性变化"""
    def __init__(self, id, start_node_id, end_node_id, node_ids, length, start_curvature, end_curvature):
        super().__init__(
            id=id,
            start_node_id=start_node_id,
            end_node_id=end_node_id,
            node_ids=node_ids,
            length=length,
            start_curvature=start_curvature,  # 起始曲率
            end_curvature=end_curvature,      # 结束曲率
            type_="spiral"
        )
class Poly3(GeometrySegment):
    def __init__(self, id, start_node_id, end_node_id, node_ids, length, au, bu, cu, du, av, bv, cv, dv):
        super().__init__(
            id=id,
            start_node_id=start_node_id,
            end_node_id=end_node_id,
            node_ids=node_ids,
            length=length,
            au = au,
            bu = bu,
            cu = cu,
            du = du,
            av = av,
            bv = bv,
            cv = cv,
            dv = dv,
            type_="poly3"
        )