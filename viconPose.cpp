#include <thread>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <HAL/Posys/PosysDevice.h>
#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <chrono>
#include <Eigen/Eigen>
#include <math.h>

#include "PID.h"

#define PI 3.14159265359
#define PI_QUART 0.78539816339

typedef Eigen::Transform<double,3,Eigen::Affine> Pose;
typedef Eigen::Quaterniond Rotation;
typedef Eigen::Vector3d TranslationVector;
typedef Eigen::Vector3d LinVel;
typedef Eigen::Vector3d RotVel;

struct PoseState{
  Pose prev_pose;
  Pose pose;
  LinVel lin_vel;
  RotVel rot_vel;
  double prev_dev_time;
  double dev_time;
  bool initialized = false;
  double x, y;
};

PoseState pose_state;
hal::CarCommandMsg commandMSG;
PID steering;
//PID throttle;

void ViconPoseHandler(hal::PoseMsg& PoseData){

  pose_state.dev_time = PoseData.device_time();
  pose_state.x = PoseData.pose().data(0);
  pose_state.y = PoseData.pose().data(1);
  //x-y translation
  pose_state.pose = Pose::Identity();
  pose_state.pose.translate(TranslationVector(PoseData.pose().data(0),PoseData.pose().data(1),0));

  //yaw rotation
  Rotation rotMat(PoseData.pose().data(6),PoseData.pose().data(3),PoseData.pose().data(4),PoseData.pose().data(5));
  pose_state.pose.rotate(rotMat);
  Eigen::AngleAxisd curr_rot(pose_state.pose.rotation());
  double yaw = curr_rot.angle() * curr_rot.axis()[2];

  if(pose_state.initialized){
    double inv_delta_t = 1/(PoseData.device_time() - pose_state.prev_dev_time);
    pose_state.lin_vel[0] = (pose_state.pose.translation()[0] - pose_state.prev_pose.translation()[0]) * inv_delta_t;
    pose_state.lin_vel[1] = (pose_state.pose.translation()[1] - pose_state.prev_pose.translation()[1]) * inv_delta_t;
    pose_state.lin_vel[2] = 0;
    pose_state.rot_vel[0] = 0;
    pose_state.rot_vel[1] = 0;
    Eigen::AngleAxisd prev_rot(pose_state.prev_pose.rotation());
    double delta_yaw = yaw - prev_rot.angle() * prev_rot.axis()[2];
    if(delta_yaw > PI){
        delta_yaw = 2*PI - delta_yaw;
    } else if(delta_yaw < -PI){
        delta_yaw = -2*PI - delta_yaw;
    }
    pose_state.rot_vel[2] = delta_yaw * inv_delta_t;

  } else{
    pose_state.lin_vel = LinVel(0,0,0);
    pose_state.rot_vel = RotVel(0,0,0);
  }
  pose_state.prev_pose = pose_state.pose;
  pose_state.prev_dev_time = PoseData.device_time();
  pose_state.initialized = true;
}

void GamepadCallback(hal::GamepadMsg& _msg){
  commandMSG.set_steering_angle(-_msg.axes().data(0));
//  commandMSG.set_steering_angle(steering.Angle());
  commandMSG.set_throttle_percent(_msg.axes().data(3)*20);
//  commandMSG.set_throttle_percent(throttle.Out());
}

double Circle_CTE(double Px, double Py, double radius);
double TrackCircle_CTE(double Px, double Py, double radius);
double Track_CTE(double Px, double Py);

int main(int argc, char** argv){

  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to Optitrack system
  hal::Posys optitrack("vicon://tracker:[dummy]");
  optitrack.RegisterPosysDataCallback(&ViconPoseHandler);

//  double dt = 0;

//  throttle.SetGains(0,0,0);
//  double velocity = 0.05;

//  //for line path
//  double intercept = 0;
//  steering.SetGains(0,0,0);

//  //for circle path
//  double radius = 1;

  while(1){

//    dt = pose_state.dev_time - pose_state.prev_dev_time;

//    //line path
//    steering.CalculateAngle(pose_state.x,intercept,dt);

//    //circular path
//    steering.CalculateAngle(Circle_CTE(pose_state.x,pose_state.y,radius),0,dt);

//    //track
//    steering.CalculateAngle(Track_CTE(pose_state.x,pose_state.y),0,dt);

//    //throttle control
//    throttle.CalculateThrottle(velocity,pose_state.lin_vel.norm(),dt);

    std::cout << "x-pose: " << pose_state.x << ", " << "y-pose: " << pose_state.y << std::endl;

//    std::cout << "x-velocity: " << pose_state.lin_vel[0] << ", " << "y-velocity: " << pose_state.lin_vel[1]
//              << ", " << "yaw rot-velocity: " << pose_state.rot_vel[2] << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}

double Circle_CTE(double Px, double Py, double radius){
//creates circular track, for testing purposes

  double Kp = 25;
  double Ki = 0;
  double Kd = 20;
  steering.SetGains(Kp,Ki,Kd);

  double theta = atan(Py/Px);
  double rX = radius*cos(theta);
  double rY = radius*sin(theta);

  return sqrt(pow((pow(rX,2)-pow(Px,2)),2) + pow((pow(rY,2)-pow(Py,2)),2));
}

double TrackCircle_CTE(double Px, double Py, double radius){
//calculate error on circular paths for track

  double theta = atan(Py/Px);
  double rX = radius*cos(theta);
  double rY = radius*sin(theta);

  return sqrt(pow((pow(rX,2)-pow(Px,2)),2) + pow((pow(rY,2)-pow(Py,2)),2));
}

double Track_CTE(double Px, double Py){
//creates track to be used for demo

  double linear_Kp = 8;
  double linear_Ki = 0;
  double linear_Kd = 1;
  double circ_Kp = 25;
  double circ_Ki = 0;
  double circ_Kd = 20;

  int intercept = 1;
  double radius = intercept;

  if(Px > 0 && Py > -1 && Py < .99){ //straight segment on right side of track
    steering.SetGains(linear_Kp,linear_Ki,linear_Kd);
    return Px-intercept;
  }
  else if(Py > 1){ //half circle on positive y-pose side
    steering.SetGains(circ_Kp,circ_Ki,circ_Kd);
    return TrackCircle_CTE(Px,Py-1,radius);
  }
  else if(Px < 0 && Py < 1 && Py > -1){ //straight segment on left side of track
    steering.SetGains(linear_Kp,linear_Ki,linear_Kd);
    return (-1*Px)-intercept;
  }
  else if(Py < -.99){ //half circle on negative y-pose side
    steering.SetGains(circ_Kp,circ_Ki,circ_Kd);
    return TrackCircle_CTE(Px,Py+1,radius);
  }
}
