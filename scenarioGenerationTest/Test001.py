from scenariogeneration import xodr
# 创建一个OpenDrive实例
my_odr = xodr.OpenDrive("myroad")

# 创建一条直路
road_1 = xodr.create_road(geometry=[xodr.Line(100)],
                          id=0,
                          left_lanes=2,
                          right_lanes=3,
                          lane_width=3.5,
                          center_road_mark=xodr.RoadMark(xodr.RoadMarkType.solid, 0.2),
                          road_type=-1)

# 将道路添加到OpenDrive
my_odr.add_road(road_1)

# Adjust initial positions of the roads looking at succ-pred logic
my_odr.adjust_roads_and_lanes()

# 将上述道路保存下来
my_odr.write_xml("test_001_straight_road.xodr")