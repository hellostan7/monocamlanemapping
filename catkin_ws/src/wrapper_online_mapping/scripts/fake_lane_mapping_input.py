#!/usr/bin/env python
# license removed for brevity
import rospy
from vehicle_msgs.msg import ArenaInfoStatic
from vehicle_msgs.msg import LaneNet
from vehicle_msgs.msg import Lane
import geometry_msgs
from rospy import Time


class FakeOnlineLaneMappingInput:
    def __init__(self):
        self.pub = rospy.Publisher('arena_info_static', ArenaInfoStatic, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def publish_arena_info_static(self):
        # 创建并发布静态场地信息消息
        arena_info_static = ArenaInfoStatic()
        arena_info_static.header.frame_id = "XXX";
        arena_info_static.header.stamp = Time.now();

        # 初始化 lane_net
        arena_info_static.lane_net = LaneNet()  # 确保 lane_net 被初始化
        arena_info_static.lane_net.lanes = []  # 初始化 lanes 列表

        lane = Lane()
        lane.id = 123
        lane.length = 100

        # 设置起始点和结束点
        start_point = geometry_msgs.msg.Point()
        start_point.x = 0
        start_point.y = 0
        start_point.z = 0

        final_point = geometry_msgs.msg.Point()
        final_point.x = 100
        final_point.y = 0
        final_point.z = 0

        # 设置起始点和结束点
        lane.start_point = start_point
        lane.final_point = final_point

        # 插值点
        num_points = 100  # 插值点数量
        lane.points = []

        # 生成插值点
        for i in range(num_points + 1):
            point = geometry_msgs.msg.Point()
            point.x = start_point.x + (final_point.x - start_point.x) * (i / num_points)
            point.y = start_point.y + (final_point.y - start_point.y) * (i / num_points)
            point.z = start_point.z + (final_point.z - start_point.z) * (i / num_points)
            lane.points.append(point)

        arena_info_static.lane_net.lanes.append(lane)


        while not rospy.is_shutdown():
            self.pub.publish(arena_info_static)
            # self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('FakeOnlineLaneMappingInput')
    fake_online_mapping_input = FakeOnlineLaneMappingInput()
    fake_online_mapping_input.publish_arena_info_static()
