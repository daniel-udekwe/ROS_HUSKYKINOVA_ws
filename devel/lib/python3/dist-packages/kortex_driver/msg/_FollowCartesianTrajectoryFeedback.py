# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/FollowCartesianTrajectoryFeedback.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class FollowCartesianTrajectoryFeedback(genpy.Message):
  _md5sum = "42254cfafb6dcdba4875e87f418b2459"
  _type = "kortex_driver/FollowCartesianTrajectoryFeedback"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#feedback
CartesianWaypoint actual

================================================================================
MSG: kortex_driver/CartesianWaypoint

Pose pose
uint32 reference_frame
float32 maximum_linear_velocity
float32 maximum_angular_velocity
float32 blending_radius
================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z"""
  __slots__ = ['actual']
  _slot_types = ['kortex_driver/CartesianWaypoint']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       actual

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(FollowCartesianTrajectoryFeedback, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.actual is None:
        self.actual = kortex_driver.msg.CartesianWaypoint()
    else:
      self.actual = kortex_driver.msg.CartesianWaypoint()

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
      buff.write(_get_struct_6fI3f().pack(_x.actual.pose.x, _x.actual.pose.y, _x.actual.pose.z, _x.actual.pose.theta_x, _x.actual.pose.theta_y, _x.actual.pose.theta_z, _x.actual.reference_frame, _x.actual.maximum_linear_velocity, _x.actual.maximum_angular_velocity, _x.actual.blending_radius))
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
      if self.actual is None:
        self.actual = kortex_driver.msg.CartesianWaypoint()
      end = 0
      _x = self
      start = end
      end += 40
      (_x.actual.pose.x, _x.actual.pose.y, _x.actual.pose.z, _x.actual.pose.theta_x, _x.actual.pose.theta_y, _x.actual.pose.theta_z, _x.actual.reference_frame, _x.actual.maximum_linear_velocity, _x.actual.maximum_angular_velocity, _x.actual.blending_radius,) = _get_struct_6fI3f().unpack(str[start:end])
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
      buff.write(_get_struct_6fI3f().pack(_x.actual.pose.x, _x.actual.pose.y, _x.actual.pose.z, _x.actual.pose.theta_x, _x.actual.pose.theta_y, _x.actual.pose.theta_z, _x.actual.reference_frame, _x.actual.maximum_linear_velocity, _x.actual.maximum_angular_velocity, _x.actual.blending_radius))
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
      if self.actual is None:
        self.actual = kortex_driver.msg.CartesianWaypoint()
      end = 0
      _x = self
      start = end
      end += 40
      (_x.actual.pose.x, _x.actual.pose.y, _x.actual.pose.z, _x.actual.pose.theta_x, _x.actual.pose.theta_y, _x.actual.pose.theta_z, _x.actual.reference_frame, _x.actual.maximum_linear_velocity, _x.actual.maximum_angular_velocity, _x.actual.blending_radius,) = _get_struct_6fI3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6fI3f = None
def _get_struct_6fI3f():
    global _struct_6fI3f
    if _struct_6fI3f is None:
        _struct_6fI3f = struct.Struct("<6fI3f")
    return _struct_6fI3f
