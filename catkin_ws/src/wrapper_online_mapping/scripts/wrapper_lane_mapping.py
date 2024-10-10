#!/usr/bin/env python3
# license removed for brevity
import rospy
from vehicle_msgs.msg import ArenaInfoStatic
from vehicle_msgs.msg import LaneNet
from vehicle_msgs.msg import Lane
from rospy import Time
import geometry_msgs
from visualization_msgs.msg import MarkerArray, Marker



class OnlineLaneMappingNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node('online_lane_mapping_node', anonymous=True)

        # 创建发布者
        self.publisher = rospy.Publisher('/markerarray_onlinelanemapping', MarkerArray, queue_size=10)

        # 创建订阅者
        self.subscriber = rospy.Subscriber('/arena_info_static', ArenaInfoStatic, self.callback)

    def callback(self, msg):
        # 打印 lanes 的数量
        # lanes_count = len(msg.lane_net.lanes)
        # rospy.loginfo(f"Number of lanes: {lanes_count}")

        marker_array = MarkerArray()

        id = 0
        for point in msg.lane_net.lanes[0].points:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "my_namespace"
            marker.id = id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = point.z
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0  # Alpha
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue
            id = id + 1

            marker_array.markers.append(marker)

        # 发布修改后的消息
        self.publisher.publish(marker_array)

    def run(self):
        rospy.spin()  # 保持节点运行

if __name__ == '__main__':
    try:
        node = OnlineLaneMappingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
