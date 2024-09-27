// Auto-generated. Do not edit!

// (in-package openlane_bag.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LanePoint = require('./LanePoint.js');

//-----------------------------------------------------------

class Lane {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lane = null;
      this.num_points = null;
      this.attribute = null;
      this.track_id = null;
      this.category = null;
    }
    else {
      if (initObj.hasOwnProperty('lane')) {
        this.lane = initObj.lane
      }
      else {
        this.lane = [];
      }
      if (initObj.hasOwnProperty('num_points')) {
        this.num_points = initObj.num_points
      }
      else {
        this.num_points = 0;
      }
      if (initObj.hasOwnProperty('attribute')) {
        this.attribute = initObj.attribute
      }
      else {
        this.attribute = 0;
      }
      if (initObj.hasOwnProperty('track_id')) {
        this.track_id = initObj.track_id
      }
      else {
        this.track_id = 0;
      }
      if (initObj.hasOwnProperty('category')) {
        this.category = initObj.category
      }
      else {
        this.category = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Lane
    // Serialize message field [lane]
    // Serialize the length for message field [lane]
    bufferOffset = _serializer.uint32(obj.lane.length, buffer, bufferOffset);
    obj.lane.forEach((val) => {
      bufferOffset = LanePoint.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [num_points]
    bufferOffset = _serializer.int32(obj.num_points, buffer, bufferOffset);
    // Serialize message field [attribute]
    bufferOffset = _serializer.int32(obj.attribute, buffer, bufferOffset);
    // Serialize message field [track_id]
    bufferOffset = _serializer.int32(obj.track_id, buffer, bufferOffset);
    // Serialize message field [category]
    bufferOffset = _serializer.int32(obj.category, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lane
    let len;
    let data = new Lane(null);
    // Deserialize message field [lane]
    // Deserialize array length for message field [lane]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.lane = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.lane[i] = LanePoint.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [num_points]
    data.num_points = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [attribute]
    data.attribute = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [track_id]
    data.track_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [category]
    data.category = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.lane.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'openlane_bag/Lane';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bc501fcdae255d97e8770b24dbfa3d94';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Lane(null);
    if (msg.lane !== undefined) {
      resolved.lane = new Array(msg.lane.length);
      for (let i = 0; i < resolved.lane.length; ++i) {
        resolved.lane[i] = LanePoint.Resolve(msg.lane[i]);
      }
    }
    else {
      resolved.lane = []
    }

    if (msg.num_points !== undefined) {
      resolved.num_points = msg.num_points;
    }
    else {
      resolved.num_points = 0
    }

    if (msg.attribute !== undefined) {
      resolved.attribute = msg.attribute;
    }
    else {
      resolved.attribute = 0
    }

    if (msg.track_id !== undefined) {
      resolved.track_id = msg.track_id;
    }
    else {
      resolved.track_id = 0
    }

    if (msg.category !== undefined) {
      resolved.category = msg.category;
    }
    else {
      resolved.category = 0
    }

    return resolved;
    }
};

module.exports = Lane;
