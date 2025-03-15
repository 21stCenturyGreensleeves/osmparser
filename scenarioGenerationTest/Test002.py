from scenariogeneration import xodr

# 创建3条道路
roads = []
# 起始车道-100m直道
roads.append(xodr.create_road(xodr.Line(100), id=0, left_lanes=2, right_lanes=2))
# 终止车道-100m直道
roads.append(xodr.create_road(xodr.Line(100), id=1, left_lanes=2, right_lanes=2))
# 中间车道-100m螺旋车道
roads.append(xodr.create_road(xodr.Spiral(0.001, 0.02, 100), id=3, left_lanes=2, right_lanes=2, road_type=1))

# 定义螺旋车道的连接关系
roads[2].add_predecessor(xodr.ElementType.road, 0, xodr.ContactPoint.end)
roads[2].add_successor(xodr.ElementType.road, 1, xodr.ContactPoint.start)

# 创建junction
junction = xodr.create_junction(roads[2:], 1, roads[0:2])

# 创建OpenDrive对象，并添加进去所有道路
odr = xodr.OpenDrive("myroad")
for r in roads:
    odr.add_road(r)
odr.adjust_roads_and_lanes()
odr.add_junction(junction)

# 将上述道路保存下来
odr.write_xml("test_002_roads_connection.xodr")