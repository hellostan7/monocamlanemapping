// Auto-generated. Do not edit!

// (in-package vehicle_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LaneNet = require('./LaneNet.js');
let ObstacleSet = require('./ObstacleSet.js');
let VehicleSet = require('./VehicleSet.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ArenaInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lane_net = null;
      this.obstacle_set = null;
      this.vehicle_set = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lane_net')) {
        this.lane_net = initObj.lane_net
      }
      else {
        this.lane_net = new LaneNet();
      }
      if (initObj.hasOwnProperty('obstacle_set')) {
        this.obstacle_set = initObj.obstacle_set
      }
      else {
        this.obstacle_set = new ObstacleSet();
      }
      if (initObj.hasOwnProperty('vehicle_set')) {
        this.vehicle_set = initObj.vehicle_set
      }
      else {
        this.vehicle_set = new VehicleSet();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArenaInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lane_net]
    bufferOffset = LaneNet.serialize(obj.lane_net, buffer, bufferOffset);
    // Serialize message field [obstacle_set]
    bufferOffset = ObstacleSet.serialize(obj.obstacle_set, buffer, bufferOffset);
    // Serialize message field [vehicle_set]
    bufferOffset = VehicleSet.serialize(obj.vehicle_set, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArenaInfo
    let len;
    let data = new ArenaInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lane_net]
    data.lane_net = LaneNet.deserialize(buffer, bufferOffset);
    // Deserialize message field [obstacle_set]
    data.obstacle_set = ObstacleSet.deserialize(buffer, bufferOffset);
    // Deserialize message field [vehicle_set]
    data.vehicle_set = VehicleSet.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += LaneNet.getMessageSize(object.lane_net);
    length += ObstacleSet.getMessageSize(object.obstacle_set);
    length += VehicleSet.getMessageSize(object.vehicle_set);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vehicle_msgs/ArenaInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9211a6cea105001098b4c5848c383814';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    LaneNet     lane_net
    ObstacleSet obstacle_set
    VehicleSet  vehicle_set
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: vehicle_msgs/LaneNet
    Header header
    
    Lane[] lanes
    
    ================================================================================
    MSG: vehicle_msgs/Lane
    Header header
    
    int32 id
    int32 dir
    
    int32[] child_id
    int32[] father_id
    
    int32 l_lane_id
    bool l_change_avbl
    
    int32 r_lane_id
    bool r_change_avbl
    
    string behavior
    
    float32 length
    
    geometry_msgs/Point start_point
    geometry_msgs/Point final_point
    
    geometry_msgs/Point[] points
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: vehicle_msgs/ObstacleSet
    Header header
    
    CircleObstacle[] obs_circle
    PolygonObstacle[] obs_polygon
    
    ================================================================================
    MSG: vehicle_msgs/CircleObstacle
    Header header
    
    int32 id
    Circle circle
    
    ================================================================================
    MSG: vehicle_msgs/Circle
    geometry_msgs/Point center
    float32 radius
    
    ================================================================================
    MSG: vehicle_msgs/PolygonObstacle
    Header header
    
    int32 id
    geometry_msgs/Polygon polygon
    
    ================================================================================
    MSG: geometry_msgs/Polygon
    #A specification of a polygon where the first and last points are assumed to be connected
    Point32[] points
    
    ================================================================================
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    ================================================================================
    MSG: vehicle_msgs/VehicleSet
    Header header
    
    Vehicle[] vehicles
    
    ================================================================================
    MSG: vehicle_msgs/Vehicle
    Header header
    std_msgs/Int32 id
    std_msgs/String subclass
    std_msgs/String type
    VehicleParam param
    State state
    
    ================================================================================
    MSG: std_msgs/Int32
    int32 data
    ================================================================================
    MSG: std_msgs/String
    string data
    
    ================================================================================
    MSG: vehicle_msgs/VehicleParam
    # Kinematic
    float32 width
    float32 length
    float32 wheel_base
    float32 front_suspension
    float32 rear_suspension
    float32 max_steering_angle
    
    float32 d_cr # Length between rear axle to center of vehicle
    
    # Dynamic
    float32 max_longitudinal_acc
    float32 max_lateral_acc
    
    ================================================================================
    MSG: vehicle_msgs/State
    Header header
    geometry_msgs/Point vec_position
    float64 angle
    float64 curvature
    float64 velocity
    float64 acceleration
    float64 steer
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArenaInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lane_net !== undefined) {
      resolved.lane_net = LaneNet.Resolve(msg.lane_net)
    }
    else {
      resolved.lane_net = new LaneNet()
    }

    if (msg.obstacle_set !== undefined) {
      resolved.obstacle_set = ObstacleSet.Resolve(msg.obstacle_set)
    }
    else {
      resolved.obstacle_set = new ObstacleSet()
    }

    if (msg.vehicle_set !== undefined) {
      resolved.vehicle_set = VehicleSet.Resolve(msg.vehicle_set)
    }
    else {
      resolved.vehicle_set = new VehicleSet()
    }

    return resolved;
    }
};

module.exports = ArenaInfo;
