import xml.etree.ElementTree as ET
from collections import defaultdict
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import numpy as np
from pyproj import Proj
from scipy.optimize import least_squares

MAX_SPEED = 40
LANES_NUMBER = 2

CAR_HIGHWAY_TYPES = {
    'motorway', 'motorway_junction', 'motorway_link',
    'trunk', 'trunk_link',
    'primary', 'primary_link',
    'secondary', 'secondary_link'
    'tertiary', 'tertiary',
    'unclassified', 'residential', 'service'
}


def calculate_angle(prev_node, curr_node, next_node):
    vec1 = (curr_node.x - prev_node.x, curr_node.y - prev_node.y)
    vec2 = (next_node.x - curr_node.x, next_node.y - curr_node.y)

    angle = np.degrees(np.arctan2(vec2[1], vec2[0]) -
                       np.arctan2(vec1[1], vec1[0]))
    return angle % 360

class OSMNode:
    def __init__(self, node_id, lat, lon, x, y):
        self.id = node_id
        self.lat = float(lat)
        self.lon = float(lon)
        self.x = y        # 适应matplotlib的坐标系的改动
        self.y = -x
        self.ways = set()      # 记录这个节点所在的所有道路
        self.is_junction = False
        self.is_endpoint = False

class OSMWay:
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

class GeometrySegment:
    def __init__(self, start_node_id, end_node_id, length, start_curvature, end_curvature):
        self.start = start_node_id
        self.end = end_node_id
        self.length = length
        self.start_curvature = start_curvature
        self.end_curvarure = end_curvature

class Line(GeometrySegment):
    def __init__(self, start_node_id, end_node_id, length):
        super().__init__(start_node_id, end_node_id, length, 0.0, 0.0)

class Arc(GeometrySegment):
    def __init__(self, start_node_id, end_node_id, length, curvature):
        super().__init__(start_node_id, end_node_id, length, curvature, curvature)

class Spiral(GeometrySegment):
    def __init__(self, start_node_id, end_node_id, length, start_curvature, end_curvature):
        super().__init__(start_node_id, end_node_id, length, start_curvature, end_curvature)

class RoadSegment:
    def __init__(self, segment_id, start_id, end_id, segment_nodes, owned_way):
        self.id = segment_id
        self.start = start_id
        self.end = end_id
        self.nodes = segment_nodes  # 只存id

        self.owned = owned_way

        self.geometry_segments = []
        self.predecessor = None
        self.successor = None

    def compute_geometry_segments(self, nodes_dict):
        coords = []
        for node_id in self.nodes:
            node = nodes_dict[node_id]
            coords.append((node.x, node.y))
        if len(coords) < 2:
            return
        curvatures = self._calculate_curvatures(coords)
        self._segment_geometry(coords, curvatures, nodes_dict)

    def _calculate_curvatures(self, coords):
        n = len(coords)
        curvatures = np.zeros(n)
        for i in range(1, n - 1):
            x_prev, y_prev = coords[i - 1]
            x_curr, y_curr = coords[i]
            x_next, y_next = coords[i + 1]
            dx1 = x_curr - x_prev
            dy1 = y_curr - y_prev
            dx2 = x_next - x_curr
            dy2 = y_next - y_curr
            denom = (dx1 ** 2 + dy1 ** 2)  ** 1.5 + (dx2 ** 2 + dy2 ** 2)  ** 1.5
            if denom == 0:
                curvature = 0.0
            else:
                area = dx1 * dy2 - dy1 * dx2
                curvature = 2 * area / denom
            curvatures[i] = curvature
        curvatures[0] = curvatures[1]
        curvatures[-1] = curvatures[-2]
        return curvatures

    def _segment_geometry(self, coords, curvatures, nodes_dict):
        n = len(coords)
        i = 0
        while i < n - 1:
            j = self._find_next_segment(coords, curvatures, i)
            start_node_id = self.nodes[i]
            end_node_id = self.nodes[j]
            segment_coords = coords[i:j + 1]
            length = self._calculate_length(segment_coords)
            k_start = curvatures[i]
            k_end = curvatures[j]
            if abs(k_start - k_end) < 1e-5:
                if abs(k_start) < 1e-5:
                    geom = Line(start_node_id, end_node_id, length)
                else:
                    geom = Arc(start_node_id, end_node_id, length, k_start)
            else:
                geom = Spiral(start_node_id, end_node_id, length, k_start, k_end)
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
                return j - 1
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

    def _calculate_length(self, coords):
        length = 0.0
        for i in range(1, len(coords)):
            dx = coords[i][0] - coords[i - 1][0]
            dy = coords[i][1] - coords[i - 1][1]
            length += np.hypot(dx, dy)
        return length
class RoadNetwork:
    def __init__(self, osm_file):
        self.osm_file = osm_file
        self.nodes = {}
        self.ways = {}
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

        self.utm_zone = self._determine_utm_zone(center_lon)
        hemisphere = 'north' if center_lat >= 0 else 'south'
        self.proj = Proj(proj='utm', zone=self.utm_zone, ellps='WGS84', south=(hemisphere == 'south'))

        for node in root.findall('node'):
            node_id = node.attrib['id']
            lat = float(node.attrib['lat'])
            lon = float(node.attrib['lon'])
            x, y = self.proj(lon, lat)

            self.nodes[node_id] = OSMNode(node_id, lat, lon, x, y)

        for way in root.findall('way'):
            tags = {}
            for tag in way.findall('tag'):
                tags[tag.attrib['k']] = tag.attrib.get('v', '')
            if tags.get('highway') not in CAR_HIGHWAY_TYPES:
                continue

            way_id = way.attrib['id']
            way_nodes = [nd.attrib['ref'] for nd in way.findall('nd')]
            self.ways[way.attrib['id']] = OSMWay(way_id, tags, way_nodes)

            for node_id in way_nodes:
                if node_id in self.nodes:
                    self.nodes[node_id].ways.add(way_id)

        for node in self.nodes.values():
            if len(node.ways) >= 2:
                node.is_junction = True
            for way_id in node.ways:
                way = self.ways[way_id]
                if way.node_ids[0] == node.id or way.node_ids[-1] == node.id:
                    node.is_endpoint = True
        # 标记 点 是否为 junction 或 endpoint

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


# UTM坐标系统的坐标系方向与Matplotlib的默认坐标系方向不一致，UTM的坐标系y轴向上，Matplotlib的默认y轴向下
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
                    coords.append([node.x, node.y])

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
                (self.nodes[jid].x, self.nodes[jid].y)
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


# if __name__ == "__main__":
#     road_network = RoadNetwork('osm/map (1).osm')
#     road_network.process_road_network()
#     road_network.filter_isolated_components(3)
#     road_network.visualize_network()
