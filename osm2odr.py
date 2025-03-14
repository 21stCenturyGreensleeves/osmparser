import xml.etree.ElementTree as ET
from collections import defaultdict

# 配置参数
CAR_HIGHWAY_TYPES = {
    'motorway', 'trunk', 'primary', 'secondary',
    'tertiary', 'unclassified', 'residential', 'service'
}


class OSMNode:
    def __init__(self, node_id, lat, lon):
        self.id = node_id
        self.lat = float(lat)
        self.lon = float(lon)
        self.ways = []  # 包含该节点的所有道路


class OSMRoad:
    def __init__(self, way_id, tags, nodes):
        self.id = way_id
        self.tags = tags
        self.node_ids = nodes
        self.junction_start = False
        self.junction_end = False


class OpenDriveRoad:
    def __init__(self, road_id, nodes, road_type):
        self.id = road_id
        self.nodes = nodes  # 包含节点对象列表
        self.road_type = road_type
        self.predecessor = None
        self.successor = None


def parse_osm(osm_file):
    tree = ET.parse(osm_file)
    root = tree.getroot()

    # 包含所有节点和道路
    nodes = {}
    ways = []

    # 解析所有节点
    for elem in root.findall('node'):
        nodes[elem.attrib['id']] = OSMNode(
            elem.attrib['id'],
            elem.attrib['lat'],
            elem.attrib['lon']
        )

    # 解析道路数据
    for elem in root.findall('way'):
        tags = {t.attrib['k']: t.attrib['v'] for t in elem.findall('tag')}
        if tags.get('highway') in CAR_HIGHWAY_TYPES:
            way_nodes = [nd.attrib['ref'] for nd in elem.findall('nd')]
            road = OSMRoad(elem.attrib['id'], tags, way_nodes)
            ways.append(road)

            # 记录节点被哪些道路包含
            for nid in way_nodes:
                nodes[nid].ways.append(road)

    return nodes, ways


def build_junctions(nodes):
    junctions = defaultdict(list)

    # 统计节点被道路引用次数
    for node_id, node in nodes.items():
        if len(node.ways) >= 2:  # 被两条以上道路共享的视为junction节点
            junctions[node_id] = node.ways

    return junctions


def convert_to_opendrive(nodes, ways, junctions):
    opendrive_roads = []
    junction_id = 10000  # junction ID起始值

    # 创建基本道路
    for idx, way in enumerate(ways):
        road_nodes = [nodes[nid] for nid in way.node_ids]
        opendrive_road = OpenDriveRoad(
            road_id=way.id,
            nodes=road_nodes,
            road_type=way.tags['highway']
        )

        # 检查起点终点是否为junction
        start_node = way.node_ids[0]
        end_node = way.node_ids[-1]

        if start_node in junctions:
            opendrive_road.predecessor = {
                'element_type': 'junction',
                'element_id': f"j_{start_node}"
            }

        if end_node in junctions:
            opendrive_road.successor = {
                'element_type': 'junction',
                'element_id': f"j_{end_node}"
            }

        opendrive_roads.append(opendrive_road)

    # 创建Junction对象（此处简化处理）
    opendrive_junctions = []
    for j_node, j_ways in junctions.items():
        junction = {
            'id': f"j_{j_node}",
            'connections': [
                {'incoming': w.id, 'connecting_road': w.id}
                for w in j_ways
            ]
        }
        opendrive_junctions.append(junction)

    return opendrive_roads, opendrive_junctions


def generate_xodr(roads, junctions):
    root = ET.Element('OpenDRIVE')

    # 创建道路
    for road in roads:
        road_elem = ET.SubElement(root, 'road', {
            'id': str(road.id),
            'name': road.road_type,
            'length': '100'  # 此处需要实际计算几何长度
        })

        # 添加几何信息（此处简化为直线）
        planview = ET.SubElement(road_elem, 'planView')
        geometry = ET.SubElement(planview, 'geometry', {
            's': '0',
            'x': str(road.nodes[0].lon),
            'y': str(road.nodes[0].lat),
            'hdg': '0',
            'length': '100'
        })
        ET.SubElement(geometry, 'line')

        # 添加连接关系
        links = ET.SubElement(road_elem, 'links')
        if road.predecessor:
            ET.SubElement(links, 'predecessor', {
                'elementType': road.predecessor['element_type'],
                'elementId': road.predecessor['element_id']
            })
        if road.successor:
            ET.SubElement(links, 'successor', {
                'elementType': road.successor['element_type'],
                'elementId': road.successor['element_id']
            })

    # 添加Junction
    for j in junctions:
        junction_elem = ET.SubElement(root, 'junction', {
            'id': j['id'],
            'name': 'Crossing'
        })
        for conn in j['connections']:
            ET.SubElement(junction_elem, 'connection', {
                'incomingRoad': conn['incoming'],
                'connectingRoad': conn['connecting_road']
            })

    return ET.ElementTree(root)


# 主流程
if __name__ == "__main__":
    # 解析OSM文件
    nodes, ways = parse_osm('osm/simpletest.osm')

    # 识别Junction
    junctions = build_junctions(nodes)

    # 转换为OpenDRIVE结构
    opendrive_roads, opendrive_junctions = convert_to_opendrive(nodes, ways, junctions)

    # 生成XODR文件
    xodr_tree = generate_xodr(opendrive_roads, opendrive_junctions)
    xodr_tree.write('output.xodr', encoding='utf-8', xml_declaration=True)