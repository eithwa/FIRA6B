# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vision/center.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class center(genpy.Message):
  _md5sum = "072f613cc7a98f1701799d9959c2aeb3"
  _type = "vision/center"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int64 CenterX
int64 CenterY
int64 Inner
int64 Outer
int64 Front
int64 Camera_High
"""
  __slots__ = ['CenterX','CenterY','Inner','Outer','Front','Camera_High']
  _slot_types = ['int64','int64','int64','int64','int64','int64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       CenterX,CenterY,Inner,Outer,Front,Camera_High

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(center, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.CenterX is None:
        self.CenterX = 0
      if self.CenterY is None:
        self.CenterY = 0
      if self.Inner is None:
        self.Inner = 0
      if self.Outer is None:
        self.Outer = 0
      if self.Front is None:
        self.Front = 0
      if self.Camera_High is None:
        self.Camera_High = 0
    else:
      self.CenterX = 0
      self.CenterY = 0
      self.Inner = 0
      self.Outer = 0
      self.Front = 0
      self.Camera_High = 0

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
      buff.write(_struct_6q.pack(_x.CenterX, _x.CenterY, _x.Inner, _x.Outer, _x.Front, _x.Camera_High))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 48
      (_x.CenterX, _x.CenterY, _x.Inner, _x.Outer, _x.Front, _x.Camera_High,) = _struct_6q.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_6q.pack(_x.CenterX, _x.CenterY, _x.Inner, _x.Outer, _x.Front, _x.Camera_High))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 48
      (_x.CenterX, _x.CenterY, _x.Inner, _x.Outer, _x.Front, _x.Camera_High,) = _struct_6q.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_6q = struct.Struct("<6q")
