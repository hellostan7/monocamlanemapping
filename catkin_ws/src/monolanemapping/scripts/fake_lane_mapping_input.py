#!/usr/bin/env python3
# license removed for brevity
import os
import sys
import rospy
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
sys.path.append(ROOT_DIR)
msg_workspace_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../", "devel/lib/python3/dist-packages/"))
sys.path.append(msg_workspace_path)
from derived_object_msgs.msg import ObjectArray, Object
from vehicle_msgs.msg import ArenaInfoStatic
from vehicle_msgs.msg import LaneNet
from vehicle_msgs.msg import Lane
import geometry_msgs
from rospy import Time


class FakeOnlineLaneMappingInput:
    def __init__(self):
        self.pub1 = rospy.Publisher('arena_info_static', ArenaInfoStatic, queue_size=10)
        self.pub2 = rospy.Publisher('/carla/objects', ObjectArray, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def run(self):
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

        Objects_Array = ObjectArray()  # 创建 ObjectArray 实例
        Objects_Array.header.frame_id = "XXX";
        Objects_Array.header.stamp = Time.now();
        Objects_Array.objects = []  # 确保 objects 字段是一个列表

        # 创建一个 Object 实例并设置其位置
        obj = Object()
        obj.pose.position.x = 1
        obj.pose.position.y = 2
        obj.pose.position.z = 3
        obj.pose.orientation.x = 0.0
        obj.pose.orientation.y = 0.0
        obj.pose.orientation.z = 0.0
        obj.pose.orientation.w = 1
        Objects_Array.objects.append(obj)  # 将对象添加到 ObjectArray

            
        while not rospy.is_shutdown():
            self.pub1.publish(arena_info_static)
            self.pub2.publish(Objects_Array)
        

if __name__ == '__main__':
    rospy.init_node('FakeOnlineLaneMappingInput')
    fake_online_mapping_input = FakeOnlineLaneMappingInput()
    fake_online_mapping_input.run()