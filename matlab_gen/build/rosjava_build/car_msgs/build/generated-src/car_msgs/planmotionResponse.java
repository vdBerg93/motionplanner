package car_msgs;

public interface planmotionResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "car_msgs/planmotionResponse";
  static final java.lang.String _DEFINITION = "car_msgs/Reference[] ref\ncar_msgs/Trajectory[] tra";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.util.List<car_msgs.Reference> getRef();
  void setRef(java.util.List<car_msgs.Reference> value);
  java.util.List<car_msgs.Trajectory> getTra();
  void setTra(java.util.List<car_msgs.Trajectory> value);
}
