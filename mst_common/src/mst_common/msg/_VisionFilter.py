"""autogenerated by genmsg_py from VisionFilter.msg. Do not edit."""
import roslib.message
import struct

import mst_common.msg
import std_msgs.msg

class VisionFilter(roslib.message.Message):
  _md5sum = "444ae704995b1a5f6fd91cb7494a14b0"
  _type = "mst_common/VisionFilter"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header          header
ImageFilter[]   color

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: mst_common/ImageFilter
string      type
Filter[3]   filter

================================================================================
MSG: mst_common/Filter
uint8[256]  gain

"""
  __slots__ = ['header','color']
  _slot_types = ['Header','mst_common/ImageFilter[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,color
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(VisionFilter, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.color is None:
        self.color = []
    else:
      self.header = std_msgs.msg._Header.Header()
      self.color = []

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      length = len(self.color)
      buff.write(_struct_I.pack(length))
      for val1 in self.color:
        _x = val1.type
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
        for val2 in val1.filter:
          _x = val2.gain
          # - if encoded as a list instead, serialize as bytes instead of string
          if type(_x) in [list, tuple]:
            buff.write(_struct_256B.pack(*_x))
          else:
            buff.write(_struct_256s.pack(_x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.color = []
      for i in range(0, length):
        val1 = mst_common.msg.ImageFilter()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.type = str[start:end]
        val1.filter = []
        for i in range(0, 3):
          val2 = mst_common.msg.Filter()
          start = end
          end += 256
          val2.gain = str[start:end]
          val1.filter.append(val2)
        self.color.append(val1)
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      length = len(self.color)
      buff.write(_struct_I.pack(length))
      for val1 in self.color:
        _x = val1.type
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
        for val2 in val1.filter:
          _x = val2.gain
          # - if encoded as a list instead, serialize as bytes instead of string
          if type(_x) in [list, tuple]:
            buff.write(_struct_256B.pack(*_x))
          else:
            buff.write(_struct_256s.pack(_x))
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
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.color = []
      for i in range(0, length):
        val1 = mst_common.msg.ImageFilter()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.type = str[start:end]
        val1.filter = []
        for i in range(0, 3):
          val2 = mst_common.msg.Filter()
          start = end
          end += 256
          val2.gain = str[start:end]
          val1.filter.append(val2)
        self.color.append(val1)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3I = struct.Struct("<3I")
_struct_256B = struct.Struct("<256B")
_struct_256s = struct.Struct("<256s")