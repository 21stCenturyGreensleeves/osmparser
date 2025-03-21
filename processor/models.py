import numpy as np
from utils import calculate_curvature

MAX_SPEED = 40
LANES_NUMBER = 2

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
        self.ways = set() #  记录这个节点所在的所有道路，nodes中的ways是无分的
        self.is_junction = False
        self.is_endpoint = False
        # TODO 是否要记录这个点相邻的路段信息？
        self.adjacent_segment = []

# Node记录
class OSMWay:
    """
    记录道路信息，包含道路的车道数量，限速，layers表示的高度层级，是否为单行道，是否为隧道
    """
    def __init__(self, way_id, tags, nodes):
        self.id = way_id
        self.tags = tags
        self.node_ids = nodes

        self.oneway = False
        self.lanes = LANES_NUMBER
        self.layer = 1
        self.maxspeed = MAX_SPEED
        self.tunnel = False

        if 'lanes' in self.tags:
            self.lanes = int(self.tags['lanes'])
        if 'layer' in self.tags:
            self.layer = int(self.tags['layer'])
        if 'maxspeed' in self.tags:
            self.maxspeed = float(self.tags['maxspeed'])
        if 'oneway' in self.tags and self.tags['oneway'] == 'yes':
            self.oneway = True
        if 'tunnel' in self.tags and self.tags['tunnel'] == 'yes':
            self.tunnel = True

class RoadSegment:
    """
    将道路按junction与segment划分为多条路段
    """
    def __init__(self, segment_id, start_id, end_id, segment_nodes, owned_way):
        self.id = segment_id
        self.start = start_id
        self.end = end_id
        self.nodes = segment_nodes  # 只存id

        self.owned = owned_way

        self.geometry_segments = []
        self.predecessor = None
        self.successor = None
        self._precomputed_lengths = None

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

        self._segment_geometry(coords, curvatures)
        for segment in self.geometry_segments:
            segment.print_info()
        print("===========================================================>")

    def _compute_curvatures(self, coords):
        n = len(coords)
        curvatures = np.zeros(n)

        if n < 3:
            return curvatures.tolist()  # 不足3点无法计算曲率

        for i in range(1, n - 1):
            curvatures[i] = calculate_curvature(coords[i - 1], coords[i], coords[i + 1])

        return curvatures.tolist()

    def _segment_geometry(self, coords, curvatures):
        n = len(coords)
        i = 0

        curvatures_threshold = 1e-5
        while i < n - 1:
            j = self._find_next_segment(coords, curvatures, i)
            if j>= len(self.nodes):
                break
            #该段的节点起始到终止index
            start_node_id = self.nodes[i]
            end_node_id = self.nodes[j]
            segment_coords = coords[i:j + 1]

            length = sum(self._precomputed_lengths[i:j]) if i < j else 0.0

            k_start = curvatures[i]
            k_end = curvatures[j]
            if abs(k_start - k_end) < 1e-5:
                if abs(k_start) < 1e-5:
                    geom = Line(start_node_id, end_node_id, self.nodes[i:j+1], length)
                else:
                    geom = Arc(start_node_id, end_node_id, self.nodes[i:j+1], length, k_start)
            else:
                geom = Spiral(start_node_id, end_node_id, self.nodes[i:j+1], length, k_start, k_end)
            self.geometry_segments.append(geom)
            i = j

    def _find_next_segment(self, coords, curvatures, start_idx):
        n = len(coords)
        if start_idx >= n - 1:
            return start_idx
        current_type = self._determine_segment_type(curvatures, start_idx)
        for j in range(start_idx + 1, n):
            seg_type = self._determine_segment_type(curvatures, j)
            if seg_type != current_type:
                return max(j - 1, start_idx+1)
        return n - 1

    def _determine_segment_type(self, curvatures, idx):
        if idx == 0:
            return 'line'
        k_prev = curvatures[idx - 1]
        k_curr = curvatures[idx]
        if abs(k_curr) < 1e-5:
            return 'line'
        elif abs(k_curr - k_prev) < 1e-5:
            return 'arc'
        else:
            return 'spiral'

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