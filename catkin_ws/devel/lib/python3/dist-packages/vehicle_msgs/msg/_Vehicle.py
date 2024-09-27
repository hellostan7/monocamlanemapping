# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vehicle_msgs/Vehicle.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg
import vehicle_msgs.msg

class Vehicle(genpy.Message):
  _md5sum = "0372a2fa6275905a037fbc1ca20c1ad6"
  _type = "vehicle_msgs/Vehicle"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
std_msgs/Int32 id
std_msgs/String subclass
std_msgs/String type
VehicleParam param
State state

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
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['header','id','subclass','type','param','state']
  _slot_types = ['std_msgs/Header','std_msgs/Int32','std_msgs/String','std_msgs/String','vehicle_msgs/VehicleParam','vehicle_msgs/State']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,id,subclass,type,param,state

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Vehicle, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.id is None:
        self.id = std_msgs.msg.Int32()
      if self.subclass is None:
        self.subclass = std_msgs.msg.String()
      if self.type is None:
        self.type = std_msgs.msg.String()
      if self.param is None:
        self.param = vehicle_msgs.msg.VehicleParam()
      if self.state is None:
        self.state = vehicle_msgs.msg.State()
    else:
      self.header = std_msgs.msg.Header()
      self.id = std_msgs.msg.Int32()
      self.subclass = std_msgs.msg.String()
      self.type = std_msgs.msg.String()
      self.param = vehicle_msgs.msg.VehicleParam()
      self.state = vehicle_msgs.msg.State()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.id.data
      buff.write(_get_struct_i().pack(_x))
      _x = self.subclass.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.type.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_9f3I().pack(_x.param.width, _x.param.length, _x.param.wheel_base, _x.param.front_suspension, _x.param.rear_suspension, _x.param.max_steering_angle, _x.param.d_cr, _x.param.max_longitudinal_acc, _x.param.max_lateral_acc, _x.state.header.seq, _x.state.header.stamp.secs, _x.state.header.stamp.nsecs))
      _x = self.state.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_8d().pack(_x.state.vec_position.x, _x.state.vec_position.y, _x.state.vec_position.z, _x.state.angle, _x.state.curvature, _x.state.velocity, _x.state.acceleration, _x.state.steer))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.id is None:
        self.id = std_msgs.msg.Int32()
      if self.subclass is None:
        self.subclass = std_msgs.msg.String()
      if self.type is None:
        self.type = std_msgs.msg.String()
      if self.param is None:
        self.param = vehicle_msgs.msg.VehicleParam()
      if self.state is None:
        self.state = vehicle_msgs.msg.State()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.id.data,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.subclass.data = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.subclass.data = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.type.data = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.type.data = str[start:end]
      _x = self
      start = end
      end += 48
      (_x.param.width, _x.param.length, _x.param.wheel_base, _x.param.front_suspension, _x.param.rear_suspension, _x.param.max_steering_angle, _x.param.d_cr, _x.param.max_longitudinal_acc, _x.param.max_lateral_acc, _x.state.header.seq, _x.state.header.stamp.secs, _x.state.header.stamp.nsecs,) = _get_struct_9f3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.state.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.state.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 64
      (_x.state.vec_position.x, _x.state.vec_position.y, _x.state.vec_position.z, _x.state.angle, _x.state.curvature, _x.state.velocity, _x.state.acceleration, _x.state.steer,) = _get_struct_8d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.id.data
      buff.write(_get_struct_i().pack(_x))
      _x = self.subclass.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.type.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_9f3I().pack(_x.param.width, _x.param.length, _x.param.wheel_base, _x.param.front_suspension, _x.param.rear_suspension, _x.param.max_steering_angle, _x.param.d_cr, _x.param.max_longitudinal_acc, _x.param.max_lateral_acc, _x.state.header.seq, _x.state.header.stamp.secs, _x.state.header.stamp.nsecs))
      _x = self.state.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_8d().pack(_x.state.vec_position.x, _x.state.vec_position.y, _x.state.vec_position.z, _x.state.angle, _x.state.curvature, _x.state.velocity, _x.state.acceleration, _x.state.steer))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.id is None:
        self.id = std_msgs.msg.Int32()
      if self.subclass is None:
        self.subclass = std_msgs.msg.String()
      if self.type is None:
        self.type = std_msgs.msg.String()
      if self.param is None:
        self.param = vehicle_msgs.msg.VehicleParam()
      if self.state is None:
        self.state = vehicle_msgs.msg.State()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.id.data,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.subclass.data = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.subclass.data = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.type.data = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.type.data = str[start:end]
      _x = self
      start = end
      end += 48
      (_x.param.width, _x.param.length, _x.param.wheel_base, _x.param.front_suspension, _x.param.rear_suspension, _x.param.max_steering_angle, _x.param.d_cr, _x.param.max_longitudinal_acc, _x.param.max_lateral_acc, _x.state.header.seq, _x.state.header.stamp.secs, _x.state.header.stamp.nsecs,) = _get_struct_9f3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.state.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.state.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 64
      (_x.state.vec_position.x, _x.state.vec_position.y, _x.state.vec_position.z, _x.state.angle, _x.state.curvature, _x.state.velocity, _x.state.acceleration, _x.state.steer,) = _get_struct_8d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_8d = None
def _get_struct_8d():
    global _struct_8d
    if _struct_8d is None:
        _struct_8d = struct.Struct("<8d")
    return _struct_8d
_struct_9f3I = None
def _get_struct_9f3I():
    global _struct_9f3I
    if _struct_9f3I is None:
        _struct_9f3I = struct.Struct("<9f3I")
    return _struct_9f3I
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
