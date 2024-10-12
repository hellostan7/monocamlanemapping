#!/usr/bin/env python3
# coding: utf-8
# @author: Zhijian Qiao
# @email: zqiaoac@connect.ust.hk
import numpy as np
from scipy.spatial.transform import Rotation as R
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
msg_workspace_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../", "devel/lib/python3/dist-packages/"))
sys.path.append(msg_workspace_path)
from derived_object_msgs.msg import ObjectArray
from vehicle_msgs.msg import ArenaInfoStatic
from vehicle_msgs.msg import LaneNet
from vehicle_msgs.msg import Lane
from rospy import Time
import geometry_msgs
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

def monolanmapping_main(lane_mapper, input, bag_file):

    result = {}
    # process the bag file
    output = lane_mapper.process(input)

    curframe_mappoints_laneassociation = lane_mapper.get_curframe_mappoints_laneassociation()
    curframe_mappoints_odometry = lane_mapper.get_curframe_mappoints_odometry()


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

    return output, curframe_mappoints_laneassociation, curframe_mappoints_odometry

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

    def add_lane(self, points, car_posex, car_posey, car_posez, orientation_x, orientation_y, orientation_z, orientation_w, category, track_id, attribute):
        lane = self.Lane(category, track_id, attribute)
        for point in points:
            #x, y ,z, visibility = point.x, point.y, point.z, 1
            # if point.x > 30 and point.y < -200 and point.y > -220 and point.x < 100:
            #    print(f"lane_point.x, lane_point.y, lane_point.z: {point.x, point.y, point.z}")
            #    print(f"cam lane_point.x, lane_point.y, lane_point.z: {point.x - car_posex, point.y - car_posey, point.z - car_posez}")

            x = point.x - car_posex
            y = point.y - car_posey
            z = point.z - car_posez
            target_position = np.array([x, y, z])
            rotation_matrix = R.from_quat([orientation_x, orientation_y, orientation_z, orientation_w]).as_matrix()
            T_wc_inv = np.linalg.inv(rotation_matrix)
            relative_position = target_position.dot(T_wc_inv[:3, :3].T)
            position_x = relative_position[0]
            position_y = relative_position[1]
            position_z = relative_position[2]
            visibility = 1

            point = self.Point(position_x, position_y, position_z, visibility)

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
        self.vehicleinfo_x = 0.0
        self.vehicleinfo_y = 0.0
        self.vehicleinfo_z = 0.0
        self.vehicleinfo_a = 0.0
        self.vehicleinfo_b = 0.0
        self.vehicleinfo_c = 0.0
        self.vehicleinfo_d = 0.0

        # initialize lane mapper
        input = {
            "lanes_predict" : [],
            "lanes_gt" : [],
            "gt_pose_wc" : []
        }
        bag_file = os.path.join(ROOT_DIR, 'examples/data/segment-14486517341017504003_3406_349_3426_349_with_camera_labels.bag')
        
        self.lane_mapper = LaneMapping(input, bag_file, save_result=False)

        # 初始化节点
        rospy.init_node('online_lane_mapping_node', anonymous=True)

        # 创建发布者
        self.publisher = rospy.Publisher('/markerarray_onlinelanemapping', MarkerArray, queue_size=10)
        self.publisher1 = rospy.Publisher('/markerarray_onlinelanemapping_points', MarkerArray, queue_size=10)
        self.publisher2 = rospy.Publisher('/markerarray_onlinelanemapping_text', MarkerArray, queue_size=10)
        self.publisher3 = rospy.Publisher('/markerarray_onlinelanemapping_inputpoints', MarkerArray, queue_size=10)
        self.publisher4 = rospy.Publisher('/markerarray_onlinelanemapping_laneassociation_curframepoints', MarkerArray, queue_size=10)
        self.publisher5 = rospy.Publisher('/markerarray_onlinelanemapping_odometry_curframepoints', MarkerArray, queue_size=10)

        # 创建订阅者
        self.subscriber1 = rospy.Subscriber('/carla/objects', ObjectArray, self.callback_DecodeEgoVehicleInfo)
        self.subscriber2 = rospy.Subscriber('/arena_info_static', ArenaInfoStatic, self.callback_DecodeArenaInfo)

    def callback_DecodeArenaInfo(self, msg):
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
            self.inputlanes.add_lane(lane.points, self.vehicleinfo_x, self.vehicleinfo_y, self.vehicleinfo_z,
                self.vehicleinfo_a, self.vehicleinfo_b, self.vehicleinfo_c, self.vehicleinfo_d,
                category=0, track_id=i, attribute=0)
            i += 1

        self.inputlanes.add_header(123, 0)
        self.gt_pose_msg.add_header(123, 0)
        self.gt_pose_msg.add_pose(self.vehicleinfo_x, self.vehicleinfo_y, self.vehicleinfo_z, 
            self.vehicleinfo_a, self.vehicleinfo_b, self.vehicleinfo_c, self.vehicleinfo_d)                   
        
        Lanes_predict = self.inputlanes.get_lanes_predict_msg()
        Gt_pose_wc = self.gt_pose_msg.get_gt_pose_msg()


        input["lanes_predict"].append(Lanes_predict)
        input["lanes_gt"].append(Lanes_predict)
        input["gt_pose_wc"].append(Gt_pose_wc)

        #marker_array.markers.append(marker)
        bag_file = os.path.join(ROOT_DIR, 'examples/data/segment-14486517341017504003_3406_349_3426_349_with_camera_labels.bag')
        output, curframePoints_laneassociation, curframePoints_odometry = monolanmapping_main(self.lane_mapper, input, bag_file)
        
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
            marker.ns = "output_line_MarkerArray"
            marker.id = id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.MODIFY
            marker.scale.x = 0.2
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
                # p = Point(float(point[1]), float(point[2]),float(point[0]))
                p = Point(float(point[0]), float(point[1]), float(point[2]))
                marker.points.append(p)

            id = id + 1

            marker_array.markers.append(marker)

        # 发布修改后的消息
        self.publisher.publish(marker_array)

        # publish POINTS
        marker_array1 = MarkerArray()
        id = 0
        for output_single in output:
            marker1 = Marker()
            marker1.header.frame_id = "map"
            marker1.header.stamp = rospy.Time.now()
            marker1.ns = "output_ctrlpoints_MarkerArray"
            marker1.id = id
            marker1.type = Marker.POINTS
            marker1.action = Marker.MODIFY
            marker1.scale.x = 0.2
            marker1.scale.y = 0.5
            marker1.scale.z = 0.5
            marker1.color.a = 1.0  # Alpha
            marker1.color.r = 0.0  # Red
            marker1.color.g = 1.0  # Green
            marker1.color.b = 0.0  # Blue
            for point in output_single:
                p = Point(float(point[0]), float(point[1]), float(point[2]))
                marker1.points.append(p)

            id = id + 1

            marker_array1.markers.append(marker1)

        # 发布修改后的消息
        self.publisher1.publish(marker_array1)

        # publish text
        marker_array2 = MarkerArray()
        id = 0
        for output_single in output:
            for point in output_single:
                marker2 = Marker()
                marker2.header.frame_id = "map"
                marker2.header.stamp = rospy.Time.now()
                marker2.ns = "output_text_MarkerArray"
                marker2.id = id
                marker2.type = Marker.TEXT_VIEW_FACING
                marker2.action = Marker.MODIFY
                # marker2.scale.x = 0.2
                # marker2.scale.y = 0.5
                marker2.scale.z = 0.2
                marker2.color.a = 1.0  # Alpha
                marker2.color.r = 0.0  # Red
                marker2.color.g = 0.5  # Green
                marker2.color.b = 0.5  # Blue
                marker2.pose.position.x = float(point[0])
                marker2.pose.position.y = float(point[1] + 0.5)
                marker2.pose.position.z = float(point[2])
                marker2.text = f"({point[0]:.2f}, {point[1]:.2f})"
                id = id + 1

                marker_array2.markers.append(marker2)

        # 发布修改后的消息
        self.publisher2.publish(marker_array2)

        # publish input lane points
        marker_array3 = MarkerArray()
        id = 0
        for lane in msg.lane_net.lanes:
            marker3 = Marker()
            marker3.header.frame_id = "map"
            marker3.header.stamp = rospy.Time.now()
            marker3.ns = "input_point"
            marker3.id = id
            marker3.type = Marker.POINTS
            marker3.action = Marker.MODIFY
            marker3.scale.x = 0.2
            marker3.scale.y = 0.5
            marker3.scale.z = 0.5
            marker3.color.a = 1.0  # Alpha
            marker3.color.r = 0.0  # Red
            marker3.color.g = 0.0  # Greendddddddd
            marker3.color.b = 1.0  # Blue
            for point in lane.points:

                p_input = Point(point.x, point.y, point.z)
                marker3.points.append(p_input)

                id = id + 1

            marker_array3.markers.append(marker3)

        # 发布修改后的消息
        self.publisher3.publish(marker_array3)
        
        # publish curframe lane_association points
        marker_array4 = MarkerArray()
        id = 0
        for row in curframePoints_laneassociation:
            marker4 = Marker()
            marker4.header.frame_id = "map"
            marker4.header.stamp = rospy.Time.now()
            marker4.ns = "lane_association_curframepoints_MarkerArray"
            marker4.id = id
            marker4.type = Marker.POINTS
            marker4.action = Marker.MODIFY
            marker4.scale.x = 0.2
            marker4.scale.y = 0.5
            marker4.scale.z = 0.5
            marker4.color.a = 1.0  # Alpha
            marker4.color.r = 0.5  # Red
            marker4.color.g = 0.5  # Greendddddddd
            marker4.color.b = 0.0  # Blue
            x,y,z = row
            p_curframe_laneassociation = Point(x, y, z)
            marker4.points.append(p_curframe_laneassociation)
            id = id + 1

            marker_array4.markers.append(marker4)

        # 发布修改后的消息
        self.publisher4.publish(marker_array4)

        # publish curframe odometry points
        marker_array5 = MarkerArray()
        id = 0
        for row in curframePoints_odometry:
            marker5 = Marker()
            marker5.header.frame_id = "map"
            marker5.header.stamp = rospy.Time.now()
            marker5.ns = "odometry_curframepoints_MarkerArray"
            marker5.id = id
            marker5.type = Marker.POINTS
            marker5.action = Marker.MODIFY
            marker5.scale.x = 0.2
            marker5.scale.y = 0.3
            marker5.scale.z = 0.5
            marker5.color.a = 1.0  # Alpha
            marker5.color.r = 0.5  # Red
            marker5.color.g = 0.0  # Greendddddddd
            marker5.color.b = 0.5  # Blue
            x,y,z = row
            p_curframe_odometry = Point(x, y, z)
            marker5.points.append(p_curframe_odometry)
            id = id + 1

            marker_array5.markers.append(marker5)

        # 发布修改后的消息
        self.publisher5.publish(marker_array5)

    def callback_DecodeEgoVehicleInfo(self, msg):
        # rospy.loginfo("Callback for /ego_vehicle_info triggered")
        # print("len(msg.objects)", len(msg.objects))
        print("msg.objects[0].pose.position.x ", msg.objects[0].pose.position.x)
        print("y ", msg.objects[0].pose.position.y)
        print("z ", msg.objects[0].pose.position.z)
        # print("msg.objects[0].pose.orientation.x", msg.objects[0].pose.orientation.x)
        # print("y ", msg.objects[0].pose.orientation.y)
        # print("z ", msg.objects[0].pose.orientation.z)
        # print("w ", msg.objects[0].pose.orientation.w)
        self.vehicleinfo_x = msg.objects[0].pose.position.x
        self.vehicleinfo_y = msg.objects[0].pose.position.y
        self.vehicleinfo_z = msg.objects[0].pose.position.z
        self.vehicleinfo_a = msg.objects[0].pose.orientation.x
        self.vehicleinfo_b = msg.objects[0].pose.orientation.y
        self.vehicleinfo_c = msg.objects[0].pose.orientation.z
        self.vehicleinfo_d = msg.objects[0].pose.orientation.w

    def run(self):
        rospy.spin()  # 保持节点运行

if __name__ == '__main__':
    try:
        node = OnlineLaneMappingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
