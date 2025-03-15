from scenariogeneration import xodr

# 创建两条直道
road0 = xodr.create_road(
    geometry=[xodr.Line(100)],  # 几何形状列表
    id=0,
    left_lanes=1,
    right_lanes=1,
    lane_width=3.5,
    center_road_mark=xodr.RoadMark(xodr.RoadMarkType.solid, 0.2)
)

road1 = xodr.create_road(
    geometry=[xodr.Line(100)],
    id=1,
    left_lanes=1,
    right_lanes=1,
    lane_width=3.5,
    center_road_mark=xodr.RoadMark(xodr.RoadMarkType.solid, 0.2)
)

# 定义连接关系
road0.add_successor(xodr.ElementType.road, 1, xodr.ContactPoint.start)
road1.add_predecessor(xodr.ElementType.road, 0, xodr.ContactPoint.end)

# 创建 OpenDrive 对象
odr = xodr.OpenDrive("myroad")
odr.add_road(road0)
odr.add_road(road1)

# 关键步骤：调整道路和车道
odr.adjust_roads_and_lanes()

# 保存为 .xodr 文件
odr.write_xml("fixed_road_connection.xodr")