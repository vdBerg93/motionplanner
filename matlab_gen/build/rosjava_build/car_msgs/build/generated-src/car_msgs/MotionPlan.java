package car_msgs;

public interface MotionPlan extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "car_msgs/MotionPlan";
  static final java.lang.String _DEFINITION = "# Message containing a series of reference paths\nReference[] refArray";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.util.List<car_msgs.Reference> getRefArray();
  void setRefArray(java.util.List<car_msgs.Reference> value);
}
