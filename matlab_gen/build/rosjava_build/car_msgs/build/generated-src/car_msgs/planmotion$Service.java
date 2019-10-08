package car_msgs;

public interface planmotion$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "car_msgs/planmotion$Service";
  static final java.lang.String _DEFINITION = "float64[] state\nfloat64[] goal\nfloat64 vmax\nbool bend\nfloat64[] Cxy\nfloat64[] Cxs\n---\ncar_msgs/Reference[] ref\ncar_msgs/Trajectory[] tra\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
