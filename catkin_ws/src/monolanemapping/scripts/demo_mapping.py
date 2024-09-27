#!/usr/bin/env python3
# coding: utf-8
# @author: Zhijian Qiao
# @email: zqiaoac@connect.ust.hk
import numpy as np
from tqdm import tqdm
import os
import sys
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
sys.path.append(ROOT_DIR)
from misc.utils import Logger
from datetime import datetime
import glob
from lane_slam.system.lane_mapping import LaneMapping
from misc.utils import mkdir_if_missing
from misc.config import define_args
from misc.config import cfg, cfg_from_yaml_file

import rospy
from vehicle_msgs.msg import ArenaInfoStatic
from vehicle_msgs.msg import LaneNet
from vehicle_msgs.msg import Lane
from rospy import Time
import geometry_msgs
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

def monolanmapping_main(input, bag_file):

    result = {}
    # initialize lane mapper
    lane_mapper = LaneMapping(input, bag_file, save_result=False)
    # process the bag file
    output = lane_mapper.process()


    # result[lane_mapper.segment] = stats

    # np.save(os.path.join(cfg.output_dir, 'stats.npy'), result)
    # all_stats = {}
    # for segment, stats in result.items():
    #     for interval, value in stats.items():
    #         if interval not in all_stats and type(value) == dict:
    #             all_stats[interval] = {}
    #         if type(value) == dict and len(value['error_rot']) > 0:
    #             all_stats[interval]['error_rot'] = all_stats[interval].get('error_rot', []) + value['error_rot']
    #             all_stats[interval]['error_rot_raw'] = all_stats[interval].get('error_rot_raw', []) + value['error_rot_raw']
    #             all_stats[interval]['error_trans'] = all_stats[interval].get('error_trans', []) + value['error_trans']
    #             all_stats[interval]['error_trans_raw'] = all_stats[interval].get('error_trans_raw', []) + value['error_trans_raw']
    #         if interval=='map_size':
    #             all_stats['map_size'] = all_stats.get('map_size', []) + [value]
    # print("Map size: ", np.mean(all_stats['map_size']))

    return output

    sys.stdout.close()

if __name__ == '__main__':

    args = define_args()
    np.random.seed(666)
    cfg_from_yaml_file(os.path.join(ROOT_DIR, args.cfg_file), cfg)

    print("Load config file: ", args.cfg_file)
    cfg.visualization = True
    cfg.eval_pose = args.eval_pose
    cfg_name = args.cfg_file.split('/')[-1].split('.')[0]
    cfg.output_dir = os.path.join(cfg.ROOT_DIR, "outputs", cfg_name)
    output_file = os.path.join(cfg.output_dir, 'logs', 'mapping-%s.txt' % datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
    mkdir_if_missing(os.path.join(cfg.output_dir, "logs"))
    mkdir_if_missing(os.path.join(cfg.output_dir, 'visualization'))
    mkdir_if_missing(os.path.join(cfg.output_dir, 'results'))
    mkdir_if_missing(os.path.join(cfg.output_dir, 'results_det'))
    mkdir_if_missing(os.path.join(cfg.output_dir, 'eval_results'))
    sys.stdout = Logger(output_file)

    bag_file = os.path.join(ROOT_DIR, 'examples/data/segment-14486517341017504003_3406_349_3426_349_with_camera_labels.bag')
    #main(bag_file)

class Inputlanes:
    class Point:
        def __init__(self, x, y, z, visibility):
            self.x = x
            self.y = y
            self.z = z
            self.visibility = visibility

    class Lane:
        def __init__(self, category, track_id, attribute):
            self.points = []
            self.num_points = 0
            self.category = category
            self.track_id = track_id
            self.attribute = attribute

#    class LaneList:
#        def __init__(self):
#            self.lane_list = {}

    class Timestamp:
        def __init__(self, sec, nsec):
            self.sec = sec
            self.nsec = nsec

    class Header:
        def __init__(self):
            self.seq = 0
            self.stamp = []
            self.frame_id = "camera"

    class lanes_predict:
        def __init__(self):
            self.header = Inputlanes.Header()
            self.lane_list = {}#Inputlanes.LaneList()
            self.num_lanes = 0

    def add_header(self, sec, nsec):
        self.lanes_predict_msg.header.stamp = Time.now()#self.Timestamp(sec, nsec)

    def __init__(self):
        self.lanes_predict_msg = self.lanes_predict()

    def add_lane(self, points, category, track_id, attribute):
        lane = self.Lane(category, track_id, attribute)
        for point in points:
            x, y ,z, visibility = point.x, point.y, point.z, 1
            point = self.Point(x, y, z, visibility)

            lane.points.append(point)
            #lane.points[lane.num_points] = point
            lane.num_points += 1
        #self.lanes_predict_msg.lane_list.lane_list[self.lanes_predict_msg.num_lanes] = lane
        self.lanes_predict_msg.lane_list[self.lanes_predict_msg.num_lanes] = lane
        self.lanes_predict_msg.num_lanes += 1

    def get_lanes_predict_msg(self):
        return self.lanes_predict_msg

class inputvehicle:

    class Timestamp:
        def __init__(self, sec, nsec):
            self.sec = sec
            self.nsec = nsec

    class Header:
        def __init__(self):
            self.seq = 0
            self.stamp = []
            self.frame_id = "map"
    
    class Position:
        def __init__(self, x ,y, z):
            self.x = x
            self.y = y
            self.z = z

    class Orientation:
        def __init__(self, x ,y, z, w):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class Pose:
        def __init__(self):
            self.position = []
            self.orientation = []

    class gt_pose:
        def __init__(self):
            self.header = inputvehicle.Header()
            self.pose = inputvehicle.Pose()

    def __init__(self):
        self.gt_pose_msg = self.gt_pose()

    def add_header(self, sec, nsec):
        self.gt_pose_msg.header.stamp = Time.now()#self.Timestamp(sec, nsec)

    def add_pose(self, x, y, z, a, b, c, d):
        self.gt_pose_msg.pose.position = self.Position(x,y,z)
        self.gt_pose_msg.pose.orientation = self.Orientation(a, b, c, d)

    def get_gt_pose_msg(self):
        return self.gt_pose_msg 
        

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

        input = {
            "lanes_predict" : [],
            "lanes_gt" : [],
            "gt_pose_wc" : []
        }

        self.inputlanes = Inputlanes()
        self.gt_pose_msg = inputvehicle()
        
        i = 0
        for lane in msg.lane_net.lanes:
            self.inputlanes.add_lane(lane.points, category=0, track_id=i, attribute=0)
            i += 1

        self.inputlanes.add_header(123, 0)         #????????
        self.gt_pose_msg.add_header(123, 0)        #???????? 
        self.gt_pose_msg.add_pose(0, 0, 0, 0.5, 0.5, 0.5, 0.5)                   
        
        Lanes_predict = self.inputlanes.get_lanes_predict_msg()
        Gt_pose_wc = self.gt_pose_msg.get_gt_pose_msg()


        input["lanes_predict"].append(Lanes_predict)
        input["lanes_gt"].append(Lanes_predict)
        input["gt_pose_wc"].append(Gt_pose_wc)

        #marker_array.markers.append(marker)
        bag_file = os.path.join(ROOT_DIR, 'examples/data/segment-14486517341017504003_3406_349_3426_349_with_camera_labels.bag')
        output = monolanmapping_main(input, bag_file)

        # Output Adaptor
        marker_array = MarkerArray()

        id = 0
        # print(output)
        # print(output[].shape)
        # print(output.ndim)
        for output_single in output:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "line_namespace"
            marker.id = id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0  # Alpha
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue
            for point in output_single:
                # print(point[1])
                # print(type(point[1]))

                # marker.pose.position.x = float(point[1])
                # marker.pose.position.y = float(point[2])
                # marker.pose.position.z = float(point[0])
                # marker.pose.orientation.x = 0.0
                # marker.pose.orientation.y = 0.0
                # marker.pose.orientation.z = 0.0
                # marker.pose.orientation.w = 1.0
                p = Point(float(point[1]), float(point[2]),float(point[0]))
                marker.points.append(p)

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
