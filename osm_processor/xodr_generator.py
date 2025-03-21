from collections import deque, defaultdict
from typing import Dict, List, Optional
from scenariogeneration import xodr
from osm_processor.models import RoadSegment
from network import RoadNetwork


class XODRGenerator:
    """
    思路：通过一个点入手逐步构建前驱后继网络
    """
    def __init__(self):
        self.odr = xodr.OpenDrive("generated_road")
        self._road_objects = {}  # Dict[str, xodr.Road]
        self._junction_counter = 1000  # Junction ID起始值
        self._junction_connections = defaultdict(list)

    def generate(self, road_network: RoadNetwork) -> xodr.OpenDrive:
        topology = self._build_topology_graph(road_network)
        self._generate_roads(road_network, topology)
        self._create_junctions(road_network, topology)
        self.odr.adjust_roads_and_lanes()
        return self.odr

    def _build_topology_graph(self, network: RoadNetwork) -> Dict[str, List[RoadSegment]]:
        """构建带方向性的拓扑图"""
        graph = defaultdict(list)

        for seg in network.road_segments.values():
            start_node = seg.nodes[0]
            end_node = seg.nodes[-1]

            # 正向连接
            graph[start_node].append((end_node, seg, 'forward'))

            # 反向连接（仅限双向道路）
            if not seg.owned.oneway:
                graph[end_node].append((start_node, seg, 'reverse'))

        return graph

    def _generate_roads(self, network: RoadNetwork, topology: Dict[str, List]):
        """BFS遍历生成道路并设置连接关系"""
        visited = set()
        queue = deque()

        # 选择第一个junction作为起点
        start_node = next(iter(network.junctions), None)
        if not start_node:
            start_node = next(iter(network.road_segments.values())).nodes[0]

        print(f"Starting BFS from node: {start_node}")
        self._initialize_queue(queue, start_node, topology, visited)

        while queue:
            current_node, prev_segment, direction = queue.popleft()

            for next_node, segment, seg_direction in topology.get(current_node, []):
                if segment.id in visited:
                    continue

                self._process_segment(segment, seg_direction, prev_segment)
                visited.add(segment.id)

                # 记录连接关系
                if next_node in network.junctions:
                    self._record_junction_connection(next_node, segment)

                queue.append((next_node, segment, seg_direction))

    def _initialize_queue(self, queue, start_node, topology, visited):
        for next_node, init_segment, direction in topology.get(start_node, []):
            if init_segment.id not in visited:
                queue.append((next_node, None, direction))
                visited.add(init_segment.id)

    def _process_segment(self, segment: RoadSegment, direction: str, prev_segment: Optional[RoadSegment]):
        """创建xodr道路对象并设置属性"""
        road = xodr.Road(road_id=int(segment.id), name=f"Road_{segment.id}")

        # 设置几何
        planview = xodr.PlanView()
        self._convert_geometry(planview, segment)
        road.add_planview(planview)

        # 设置车道
        lanes = self._create_lane_section(segment)
        road.add_lanes(lanes)

        # 设置连接关系
        if prev_segment:
            road.add_predecessor(xodr.ElementType.road, int(prev_segment.id))
        # Successor将在BFS后续步骤中更新

        self._road_objects[segment.id] = road
        self.odr.add_road(road)

    def _convert_geometry(self, planview: xodr.PlanView, segment: RoadSegment):
        """转换几何元素到OpenDRIVE格式"""
        for geom in segment.geometry_segments:
            if geom.type == 'line':
                planview.add_geometry(xodr.Line(length=geom.length))
            elif geom.type == 'arc':
                planview.add_geometry(xodr.Arc(curvature=geom.curvature, length=geom.length))
            else:
                planview.add_geometry(xodr.Spiral(0, 0, length=geom.length))  # 默认处理

    def _create_lane_section(self, segment: RoadSegment) -> xodr.Lanes:
        """创建车道配置"""
        lanes = xodr.Lanes()
        lane_section = xodr.LaneSection(s=0)

        if segment.owned.oneway:
            for _ in range(segment.owned.lanes):
                lane_section.add_right_lane(xodr.Lane(a=3.0))
        else:
            for _ in range(segment.owned.lanes):
                lane_section.add_right_lane(xodr.Lane(a=3.0))
                lane_section.add_left_lane(xodr.Lane(a=3.0))

        lanes.add_lane_section(lane_section)
        return lanes

    def _record_junction_connection(self, junction_id: str, connected_segment: RoadSegment):
        """记录交叉点连接关系"""
        if not hasattr(self, '_junction_connections'):
            self._junction_connections = defaultdict(list)
        self._junction_connections[junction_id].append(connected_segment.id)

    def _create_junctions(self, network: RoadNetwork, topology: Dict):
        """生成交叉点对象"""
        for junc_id, connected_roads in self._junction_connection():
            junction = xodr.Junction(
                name=f"Junction_{junc_id}",
                id=self._junction_counter,
                junction_type="default"
            )
            self._junction_counter += 1

            # 获取所有连接到此junction的道路
            for road_id in connected_roads:
                incoming_road = self._road_objects[road_id]

                # 查找所有可能连接
                for other_road_id in connected_roads:
                    if road_id == other_road_id:
                        continue

                    # 判断连接点类型
                    contact_point = self._determine_contact_point(
                        incoming_road,
                        self._road_objects[other_road_id]
                    )

                    connection = xodr.Connection(
                        incoming_road=int(road_id),
                        connecting_road=int(other_road_id),
                        contact_point=contact_point,
                        connection_type="virtual"
                    )
                    junction.add_connection(connection)

            self.odr.add_junction(junction)

    def _determine_contact_point(self, incoming: xodr.Road, connecting: xodr.Road) -> str:
        """智能判断接触点类型（简化实现）"""
        # 实际实现需要根据几何位置关系判断
        return "start" if incoming.planview.get_total_length() > connecting.planview.get_total_length() else "end"