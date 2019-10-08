package car_msgs;

public interface State extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "car_msgs/State";
  static final java.lang.String _DEFINITION = "float64[] state";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double[] getState();
  void setState(double[] value);
}
