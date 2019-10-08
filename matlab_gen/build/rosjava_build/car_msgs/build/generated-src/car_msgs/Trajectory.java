package car_msgs;

public interface Trajectory extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "car_msgs/Trajectory";
  static final java.lang.String _DEFINITION = "float64[] x\nfloat64[] y\nfloat64[] d\nfloat64[] th\nfloat64[] v\n#State[] tra";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double[] getX();
  void setX(double[] value);
  double[] getY();
  void setY(double[] value);
  double[] getD();
  void setD(double[] value);
  double[] getTh();
  void setTh(double[] value);
  double[] getV();
  void setV(double[] value);
}
