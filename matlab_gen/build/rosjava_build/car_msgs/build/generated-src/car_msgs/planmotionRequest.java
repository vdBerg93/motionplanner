package car_msgs;

public interface planmotionRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "car_msgs/planmotionRequest";
  static final java.lang.String _DEFINITION = "float64[] state\nfloat64[] goal\nfloat64 vmax\nbool bend\nfloat64[] Cxy\nfloat64[] Cxs\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double[] getState();
  void setState(double[] value);
  double[] getGoal();
  void setGoal(double[] value);
  double getVmax();
  void setVmax(double value);
  boolean getBend();
  void setBend(boolean value);
  double[] getCxy();
  void setCxy(double[] value);
  double[] getCxs();
  void setCxs(double[] value);
}
