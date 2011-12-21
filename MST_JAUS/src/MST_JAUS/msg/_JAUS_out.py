"""autogenerated by genmsg_py from JAUS_out.msg. Do not edit."""
import roslib.message
import struct


class JAUS_out(roslib.message.Message):
  _md5sum = "01898af3e2cf8253a92a72b79e09e3b3"
  _type = "MST_JAUS/JAUS_out"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool        request_control
bool        request_resume
bool        request_standby
bool        request_shutdown
bool        execute_waypoints
float64     speed
bool        set_waypoints
bool        set_local_pose
uint16[]    waypoint_id
uint16[]    waypoint_previous_id
uint16[]    waypoint_next_id
float64[]   waypoint_pose_x
float64[]   waypoint_pose_y
float64[]   pose_yaw

"""
  __slots__ = ['request_control','request_resume','request_standby','request_shutdown','execute_waypoints','speed','set_waypoints','set_local_pose','waypoint_id','waypoint_previous_id','waypoint_next_id','waypoint_pose_x','waypoint_pose_y','pose_yaw']
  _slot_types = ['bool','bool','bool','bool','bool','float64','bool','bool','uint16[]','uint16[]','uint16[]','float64[]','float64[]','float64[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       request_control,request_resume,request_standby,request_shutdown,execute_waypoints,speed,set_waypoints,set_local_pose,waypoint_id,waypoint_previous_id,waypoint_next_id,waypoint_pose_x,waypoint_pose_y,pose_yaw
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(JAUS_out, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.request_control is None:
        self.request_control = False
      if self.request_resume is None:
        self.request_resume = False
      if self.request_standby is None:
        self.request_standby = False
      if self.request_shutdown is None:
        self.request_shutdown = False
      if self.execute_waypoints is None:
        self.execute_waypoints = False
      if self.speed is None:
        self.speed = 0.
      if self.set_waypoints is None:
        self.set_waypoints = False
      if self.set_local_pose is None:
        self.set_local_pose = False
      if self.waypoint_id is None:
        self.waypoint_id = []
      if self.waypoint_previous_id is None:
        self.waypoint_previous_id = []
      if self.waypoint_next_id is None:
        self.waypoint_next_id = []
      if self.waypoint_pose_x is None:
        self.waypoint_pose_x = []
      if self.waypoint_pose_y is None:
        self.waypoint_pose_y = []
      if self.pose_yaw is None:
        self.pose_yaw = []
    else:
      self.request_control = False
      self.request_resume = False
      self.request_standby = False
      self.request_shutdown = False
      self.execute_waypoints = False
      self.speed = 0.
      self.set_waypoints = False
      self.set_local_pose = False
      self.waypoint_id = []
      self.waypoint_previous_id = []
      self.waypoint_next_id = []
      self.waypoint_pose_x = []
      self.waypoint_pose_y = []
      self.pose_yaw = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_5Bd2B.pack(_x.request_control, _x.request_resume, _x.request_standby, _x.request_shutdown, _x.execute_waypoints, _x.speed, _x.set_waypoints, _x.set_local_pose))
      length = len(self.waypoint_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(struct.pack(pattern, *self.waypoint_id))
      length = len(self.waypoint_previous_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(struct.pack(pattern, *self.waypoint_previous_id))
      length = len(self.waypoint_next_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(struct.pack(pattern, *self.waypoint_next_id))
      length = len(self.waypoint_pose_x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.waypoint_pose_x))
      length = len(self.waypoint_pose_y)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.waypoint_pose_y))
      length = len(self.pose_yaw)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.pose_yaw))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 15
      (_x.request_control, _x.request_resume, _x.request_standby, _x.request_shutdown, _x.execute_waypoints, _x.speed, _x.set_waypoints, _x.set_local_pose,) = _struct_5Bd2B.unpack(str[start:end])
      self.request_control = bool(self.request_control)
      self.request_resume = bool(self.request_resume)
      self.request_standby = bool(self.request_standby)
      self.request_shutdown = bool(self.request_shutdown)
      self.execute_waypoints = bool(self.execute_waypoints)
      self.set_waypoints = bool(self.set_waypoints)
      self.set_local_pose = bool(self.set_local_pose)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.waypoint_id = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.waypoint_previous_id = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.waypoint_next_id = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.waypoint_pose_x = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.waypoint_pose_y = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.pose_yaw = struct.unpack(pattern, str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_5Bd2B.pack(_x.request_control, _x.request_resume, _x.request_standby, _x.request_shutdown, _x.execute_waypoints, _x.speed, _x.set_waypoints, _x.set_local_pose))
      length = len(self.waypoint_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(self.waypoint_id.tostring())
      length = len(self.waypoint_previous_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(self.waypoint_previous_id.tostring())
      length = len(self.waypoint_next_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(self.waypoint_next_id.tostring())
      length = len(self.waypoint_pose_x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.waypoint_pose_x.tostring())
      length = len(self.waypoint_pose_y)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.waypoint_pose_y.tostring())
      length = len(self.pose_yaw)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.pose_yaw.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 15
      (_x.request_control, _x.request_resume, _x.request_standby, _x.request_shutdown, _x.execute_waypoints, _x.speed, _x.set_waypoints, _x.set_local_pose,) = _struct_5Bd2B.unpack(str[start:end])
      self.request_control = bool(self.request_control)
      self.request_resume = bool(self.request_resume)
      self.request_standby = bool(self.request_standby)
      self.request_shutdown = bool(self.request_shutdown)
      self.execute_waypoints = bool(self.execute_waypoints)
      self.set_waypoints = bool(self.set_waypoints)
      self.set_local_pose = bool(self.set_local_pose)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.waypoint_id = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.waypoint_previous_id = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.waypoint_next_id = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.waypoint_pose_x = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.waypoint_pose_y = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.pose_yaw = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_5Bd2B = struct.Struct("<5Bd2B")
