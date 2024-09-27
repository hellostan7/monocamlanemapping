// Auto-generated. Do not edit!

// (in-package openlane_bag.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Lane = require('./Lane.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class LaneList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lane_list = null;
      this.num_lanes = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lane_list')) {
        this.lane_list = initObj.lane_list
      }
      else {
        this.lane_list = [];
      }
      if (initObj.hasOwnProperty('num_lanes')) {
        this.num_lanes = initObj.num_lanes
      }
      else {
        this.num_lanes = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LaneList
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lane_list]
    // Serialize the length for message field [lane_list]
    bufferOffset = _serializer.uint32(obj.lane_list.length, buffer, bufferOffset);
    obj.lane_list.forEach((val) => {
      bufferOffset = Lane.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [num_lanes]
    bufferOffset = _serializer.int32(obj.num_lanes, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LaneList
    let len;
    let data = new LaneList(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lane_list]
    // Deserialize array length for message field [lane_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.lane_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.lane_list[i] = Lane.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [num_lanes]
    data.num_lanes = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.lane_list.forEach((val) => {
      length += Lane.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'openlane_bag/LaneList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c734ba2a0461f54efa410edc16ce01f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    Lane[] lane_list
    int32 num_lanes
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
    MSG: openlane_bag/Lane
    LanePoint[] lane
    int32 num_points
    int32 attribute  # attribute: the left-left lane is 1, the left lane is 2, the right lane is 3, and the right-right lane is 4.
    int32 track_id
    int32 category
    
    ================================================================================
    MSG: openlane_bag/LanePoint
    float32 x
    float32 y
    float32 z
    float32 visibility
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LaneList(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lane_list !== undefined) {
      resolved.lane_list = new Array(msg.lane_list.length);
      for (let i = 0; i < resolved.lane_list.length; ++i) {
        resolved.lane_list[i] = Lane.Resolve(msg.lane_list[i]);
      }
    }
    else {
      resolved.lane_list = []
    }

    if (msg.num_lanes !== undefined) {
      resolved.num_lanes = msg.num_lanes;
    }
    else {
      resolved.num_lanes = 0
    }

    return resolved;
    }
};

module.exports = LaneList;
