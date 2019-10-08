package car_msgs;

public interface Reference extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "car_msgs/Reference";
  static final java.lang.String _DEFINITION = "float64[] x\nfloat64[] y\nfloat64[] v\nint32 dir";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double[] getX();
  void setX(double[] value);
  double[] getY();
  void setY(double[] value);
  double[] getV();
  void setV(double[] value);
  int getDir();
  void setDir(int value);
}
