#ifndef PID_H_
#define PID_H_

class PID{

  public:
    PID();
    ~PID();
    void Init();
    void SetGains(double Kp, double Ki, double Kd);
    void CalculateAngle(double setpoint, double pv, double dt);
    double Angle();
//    void CalculateThrottle(double setpoint, double pv, double dt);
//    double Out();

  private:
    double Kp_, Ki_, Kd_, integral_, prev_error_, angle_, throttle_;
};

#endif
