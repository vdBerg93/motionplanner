#ifndef datatypes_h
#define datatypes_h

struct ControlCommand{
    double dc, ac;
    ControlCommand(double _dc, double _ac):dc(_dc),ac(_ac){};
};

typedef std::vector<double> state_type;
struct veh_state{
    float x,y,head,d,v,a;
    // veh_state();
    // veh_state(float _x, float _y, float _head): x(_x), y(_y), head(_head){};
};

// For conversion from quaternion to Euler angles

struct Quaternion
{
    double w, x, y, z;
};

struct EulerAngles
{
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}

#endif 