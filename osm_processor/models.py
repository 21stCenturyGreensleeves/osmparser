from utils import calculate_curvature
from sklearn.linear_model import RANSACRegressor, LinearRegression
import xml.etree.ElementTree as ET
from collections import defaultdict
import numpy as np
from pyproj import Proj

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

    def __init__(self, node_id, lat, lon, x, y):
        self.id = node_id
        self.lat = float(lat)
        self.lon = float(lon)
        # 适应matplotlib的坐标系的改动
        self.x = y
        self.y = -x
        self.ways = set()  # 记录这个节点所在的所有道路，nodes中的ways是无分的
        self.is_junction = False
        self.is_endpoint = False
        # TODO 是否要记录这个点相邻的路段信息？
        self.adjacent_segment = []


class RoadNetwork:
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

    def process_road_network(self):
        self.parse_osm()
        self.mark_endpoints()
        for way in self.ways.values():
            way.split_way_into_segment(self.nodes)

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


# Node记录
class OSMWay:
    """
    记录道路信息，包含道路的车道数量，限速，layers表示的高度层级，是否为单行道，是否为隧道
    """

    def __init__(self, way_id, tags, nodes):
        self.id = way_id
        self.tags = tags
        self.node_ids = nodes
        self.ways_road_segments = []  # 记录的顺序是从头到尾吗？

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

    def split_way_into_segment(self, nodes_dict, segment_id_param=SEGMENT_ID):
        segment_id = segment_id_param

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

        self.geometry_segments = []
        self.predecessor = None
        self.successor = None
        self._precomputed_lengths = None
        self.curvatures = None

    def printGeometryInfo(self):
        for segment in self.geometry_segments:
            segment.print_info()

    def compute_geometry_segments(self, nodes_dict):
        """"
        传入network中的nodes合集
        """
        coords = np.array([(nodes_dict[node_id].x, nodes_dict[node_id].y) for node_id in self.nodes])
        # 如果坐标总数小于2则无法成立
        if len(coords) < 2:
            return
        dx = np.diff(coords[:, 0])
        dy = np.diff(coords[:, 1])
        self._precomputed_lengths = np.hypot(dx, dy).tolist()
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

    def geometry_process(self):
        min_length = 2
        max_length = 100
        threshold1 = 0.01
        threshold2 = 0.02
        threshold3 = 1.0
        zero_threshold = 0.01

        segments = []
        s = 0

        while s < len(self.curvatures):
            best_e = s
            best_model = None
            best_params = []

            for e in range(s + min_length - 1, len(self.curvatures)):
                current_points = self.curvatures[s:e + 1]
                X = np.array([p.x for p in current_points]).reshape(-1, 1)
                y = np.array([p.y for p in current_points])

                inliers1 = [p for p in current_points if abs(p.y) <= threshold1]

                best_n_inliers = []
                best_n_value = None
                for _ in range(20):  # 随机采样次数
                    sample = self.random.choice(current_points)
                    n = sample.y
                    candidates = [p for p in current_points if abs(p.y - n) <= threshold2]
                    if len(candidates) > len(best_n_inliers):
                        best_n_inliers = candidates
                        best_n_value = n

                inliers3 = []
                try:
                    ransac = RANSACRegressor(
                        LinearRegression(),
                        residual_threshold=threshold3
                    )
                    ransac.fit(X, y)
                    inlier_mask = ransac.inlier_mask_
                    inliers3 = [current_points[i] for i, mask in enumerate(inlier_mask) if mask]
                except:
                    pass

                model_candidates = [
                    (len(inliers1), 1, 0),  # (匹配数, 模型类型, 参数)
                    (len(best_n_inliers), 2, best_n_value),
                    (len(inliers3), 3, (
                        ransac.estimator_.coef_[0],
                        ransac.estimator_.intercept_
                    ))
                ]

                # 按匹配点数降序排序
                model_candidates.sort(reverse=True, key=lambda x: x[0])

                # 选择最优模型
                selected_model = model_candidates[0]
                if selected_model[0] >= min_length and selected_model[0] / len(current_points) > 0.6:
                    # 处理近似零值的情况
                    if selected_model[1] == 2 and abs(selected_model[2]) < zero_threshold:
                        selected_model = (selected_model[0], 1, 0)

                    best_model = selected_model[1]
                    best_params = selected_model[2]
                    best_e = e

                # 生成线段描述
            if best_model:
                start_x = self.curvatures[s][0]
                end_x = self.curvatures[best_e][0]
                if best_model == 1:
                    segments.append(('type1', start_x, end_x, 0))
                elif best_model == 2:
                    segments.append(('type2', start_x, end_x, best_params))
                else:
                    a, b = best_params
                    segments.append(('type3', start_x, end_x, a, b))
                s = best_e + 1  # 移动到下一个未处理点
            else:
                s += 1  # 未找到合适模型，前进1个点

            return segments


class GeometrySegment:
    """
    将road segment分割为多段Geometry Segment便于xodr道路的生成，
    """

    def __init__(self, start_node_id, end_node_id, node_ids, length, start_curvature, end_curvature):
        self.start = start_node_id
        self.end = end_node_id
        self.node_ids = node_ids
        self.length = length
        self.start_curvature = start_curvature
        self.end_curvature = end_curvature

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
    def __init__(self, start_node_id, end_node_id, node_ids, length):
        super().__init__(start_node_id, end_node_id, node_ids, length, 0.0, 0.0)


class Arc(GeometrySegment):
    def __init__(self, start_node_id, end_node_id, node_ids, length, curvature):
        super().__init__(start_node_id, end_node_id, node_ids, length, curvature, curvature)


class Spiral(GeometrySegment):
    def __init__(self, start_node_id, end_node_id, node_ids, length, start_curvature, end_curvature):
        super().__init__(start_node_id, end_node_id, node_ids, length, start_curvature, end_curvature)
