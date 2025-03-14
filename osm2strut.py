import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PatchCollection
import matplotlib.patches as patches

class OSMElement:
    """OSM元素基类"""

    def __init__(self, element_id, tags, coordinates):
        self.id = element_id
        self.tags = tags
        self.coordinates = coordinates

class Road(OSMElement):
    """道路类"""

    def __init__(self, element_id, tags, coordinates):
        super().__init__(element_id, tags, coordinates)
        self.road_type = tags.get('highway', 'unknown')
        self.maxspeed = tags.get('maxspeed', None)

class Building(OSMElement):
    """建筑类"""

    def __init__(self, element_id, tags, coordinates):
        super().__init__(element_id, tags, coordinates)
        self.building_type = tags.get('building', 'unknown')
        self.levels = tags.get('building:levels', None)

class TrafficSign(OSMElement):
    """交通标志类"""

    def __init__(self, element_id, tags, coordinates):
        super().__init__(element_id, tags, coordinates)
        self.sign_type = tags.get('traffic_signals', 'unknown')

class Waterway(OSMElement):
    """水系类"""

    def __init__(self, element_id, tags, coordinates, holes=None):
        super().__init__(element_id, tags, coordinates)
        self.water_type = tags.get('waterway', 'unknown')
        self.holes = holes if holes else []

class GreenSpace(OSMElement):
    """绿地类"""

    def __init__(self, element_id, tags, coordinates):
        super().__init__(element_id, tags, coordinates)
        self.nature_type = tags.get('natural', 'unknown')

class OSMParser:
    def __init__(self, osm_file):
        self.osm_file = osm_file
        self.nodes = {}
        self.ways = {}
        self.elements = {
            'roads': [],
            'buildings': [],
            'traffic_signs': [],
            'waterways': [],
            'greenspaces': []
        }

    def parse(self):
        tree = ET.parse(self.osm_file)
        root = tree.getroot()

        # 收集所有节点坐标
        for node in root.findall('node'):
            node_id = node.attrib['id']
            self.nodes[node_id] = (
                float(node.attrib['lon']),
                float(node.attrib['lat'])
            )

        # 收集所有路径的坐标
        for way in root.findall('way'):
            way_id = way.attrib['id']
            coords = []
            for nd in way.findall('nd'):
                node_id = nd.attrib['ref']
                if node_id in self.nodes:
                    coords.append(self.nodes[node_id])
            self.ways[way_id] = coords

        # 处理道路、建筑等元素
        for way in root.findall('way'):
            tags = {}
            for tag in way.findall('tag'):
                tags[tag.attrib['k']] = tag.attrib.get('v', '')

            coords = self.ways.get(way.attrib['id'], [])

            element = None
            if 'highway' in tags:
                if tags['highway'] in ['motorway',
                                        'motorway_link'
                                        'trunk_link',
                                        'primary',
                                        'primary_link',
                                        'secondary',
                                        'secondary_link',
                                        'tertiary',
                                        'tertiary_link',
                                        'residential',
                                        'unknown',
                                        'unclassified',
                                        'service',
                                       'trunk']:
                    element = Road(way.attrib['id'], tags, coords)
                    self.elements['roads'].append(element)
            elif 'building' in tags:
                element = Building(way.attrib['id'], tags, coords)
                self.elements['buildings'].append(element)
            elif 'natural' in tags and tags['natural'] == 'water':
                element = Waterway(way.attrib['id'], tags, coords)
                self.elements['waterways'].append(element)
            elif 'waterway' in tags:
                element = Waterway(way.attrib['id'], tags, coords)
                self.elements['waterways'].append(element)
            elif 'natural' in tags:
                element = GreenSpace(way.attrib['id'], tags, coords)
                self.elements['greenspaces'].append(element)

        # 处理关系（如湖泊）
        for relation in root.findall('relation'):
            tags = {}
            for tag in relation.findall('tag'):
                tags[tag.attrib['k']] = tag.attrib.get('v', '')

            if tags.get('type') == 'multipolygon' and tags.get('natural') == 'water':
                outer_ways = []
                inner_ways = []
                for member in relation.findall('member'):
                    # 仅处理way类型的成员
                    if member.attrib.get('type') != 'way':
                        continue
                    way_id = member.attrib['ref']
                    if way_id not in self.ways:
                        continue
                    role = member.attrib.get('role')
                    if role == 'outer':
                        outer_ways.append(self.ways[way_id])
                    elif role == 'inner':
                        inner_ways.append(self.ways[way_id])

                # 处理outer环闭合
                merged_outer = []
                for way_coords in outer_ways:
                    if way_coords and way_coords[0] != way_coords[-1]:
                        way_coords = way_coords + [way_coords[0]]
                    merged_outer.extend(way_coords)
                # 确保整体闭合
                if merged_outer and merged_outer[0] != merged_outer[-1]:
                    merged_outer.append(merged_outer[0])

                # 处理inner环闭合
                processed_inner = []
                for way_coords in inner_ways:
                    if way_coords and way_coords[0] != way_coords[-1]:
                        way_coords = way_coords + [way_coords[0]]
                    processed_inner.append(way_coords)

                if merged_outer:
                    element = Waterway(relation.attrib['id'], tags, merged_outer, processed_inner)
                    self.elements['waterways'].append(element)

        # 处理交通标志
        for node in root.findall('node'):
            tags = {}
            for tag in node.findall('tag'):
                tags[tag.attrib['k']] = tag.attrib.get('v', '')
            coords = self.nodes.get(node.attrib['id'], None)
            if coords and 'traffic_sign' in tags:
                element = TrafficSign(node.attrib['id'], tags, [coords])
                self.elements['traffic_signs'].append(element)

        return self.elements

class MapVisualizer:
    @staticmethod
    def visualize(elements):
        fig, ax = plt.subplots(figsize=(12, 12))

        # 绘制绿地
        green_patches = []
        for area in elements['greenspaces']:
            if len(area.coordinates) > 2:
                polygon = patches.Polygon(area.coordinates, closed=True)
                green_patches.append(polygon)
        if green_patches:
            ax.add_collection(PatchCollection(
                green_patches, facecolor='#9bcd9b', edgecolor='none'))

        # 绘制道路
        road_lines = []
        for road in elements['roads']:
            if len(road.coordinates) > 1:
                road_lines.append(road.coordinates)
        if road_lines:
            ax.add_collection(LineCollection(
                road_lines, colors='#666666', linewidths=1.5))

        # 绘制建筑
        building_patches = []
        for building in elements['buildings']:
            if len(building.coordinates) > 2:
                polygon = patches.Polygon(building.coordinates, closed=True)
                building_patches.append(polygon)
        if building_patches:
            ax.add_collection(PatchCollection(
                building_patches, facecolor='#a38f73', edgecolor='#7a6953'))

        # 绘制交通标志
        if elements['traffic_signs']:
            sign_x = [s.coordinates[0][0] for s in elements['traffic_signs']]
            sign_y = [s.coordinates[0][1] for s in elements['traffic_signs']]
            ax.scatter(sign_x, sign_y, c='yellow', s=30, marker='^', label='Traffic Signs')

        ax.autoscale()
        ax.axis('equal')
        plt.show()

# 使用示例
if __name__ == "__main__":
    osm_file = "osm/sustc.osm"  # 替换为实际路径
    parser = OSMParser(osm_file)
    elements = parser.parse()
    visualizer = MapVisualizer()
    visualizer.visualize(elements)