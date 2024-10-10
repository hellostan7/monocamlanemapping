#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import TransformStamped

def publish_transform():
    # 初始化 ROS 节点
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    
    # 设置循环频率
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # 创建转换消息
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()  # 当前时间
        t.header.frame_id = "map"  # 父坐标系
        t.child_frame_id = "my_frame"  # 子坐标系
        t.transform.translation.x = 1.0  # x 方向的位移
        t.transform.translation.y = 2.0  # y 方向的位移
        t.transform.translation.z = 0.0  # z 方向的位移
        t.transform.rotation.x = 0.0  # 旋转四元数
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # 发布转换
        br.sendTransform((t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
                         (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w),
                         t.header.stamp,
                         t.child_frame_id,
                         t.header.frame_id)
        
        rate.sleep()  # 控制循环频率

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
