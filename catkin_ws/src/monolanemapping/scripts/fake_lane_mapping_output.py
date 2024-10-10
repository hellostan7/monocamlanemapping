#!/usr/bin/env python3
# license removed for brevity
import rospy
from vehicle_msgs.msg import ArenaInfoStatic
from vehicle_msgs.msg import LaneNet
from vehicle_msgs.msg import Lane
from rospy import Time
import geometry_msgs
from visualization_msgs.msg import MarkerArray, Marker

class FakeOnlineLaneMappingOutput:
    def __init__(self):
        # 初始化节点
        rospy.init_node('FakeOnlineLaneMappingOutput', anonymous=True)

        # 创建订阅者
        self.subscriber = rospy.Subscriber('/markerarray_onlinelanemapping', MarkerArray, self.callback)

    def callback(self, msg):
        # 打印接收到的消息
        rospy.loginfo("Received ArenaInfoStatic message:")
        
        # 打印 lanes 的数量
        markers_count = len(msg.markers)
        rospy.loginfo(f"Number of markers: {markers_count}")
        for i in range(markers_count):
            rospy.loginfo(f"  Point: (x: {msg.markers[i].pose.position.x}, y: {msg.markers[i].pose.position.y}, z: {msg.markers[i].pose.position.z})")
                


    def run(self):
        rospy.spin()  # 保持节点运行

if __name__ == '__main__':
    try:
        node = FakeOnlineLaneMappingOutput()
        node.run()
    except rospy.ROSInterruptException:
        pass
