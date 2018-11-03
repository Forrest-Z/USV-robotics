/*
***********************************************************************
* realtimedata.h:
* header file to define the realtime parameters, dependent on each
* autonomous system.
* This header file can be read by both C and C++ compilers
*
*  by Hu.ZH(Mr. SJTU)
***********************************************************************
*/

#ifndef _REALTIMEDATA_H_
#define _REALTIMEDATA_H_
#include <Eigen/Core>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6, 6> Matrix66d;
typedef Eigen::Matrix<double, 6, 3> Matrix63d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// real time data of the first vessel (K class-I)
struct realtimevessel_first {
  /********************* raw motion data ***********************************/
  // x(surge: m), y(sway: m), z(heave: m), roll(deg), pitch(deg), yaw(theta:
  // deg)  data wroten by motion capture system
  Vector6d Position;  // raw motion data

  /********************* measured data after low pass  *********************/
  // x(m), y(m), orientation(theta: rad), u, v, r (next time stamp)
  // data wroten by motion capture system
  Vector6d Measurement;  // measured data after low pass

  /********************* state *********************************************/
  // x(surge: m), y(sway: m), yaw(theta: rad), u, v, r
  // data wroten by Kalman
  Vector6d State;  // state

  /********************* state for control  ********************************/
  // xb(surge: m), yb(sway: m), yaw(theta: rad), u, v, r
  // in the body-fixed coordinate
  Vector6d State4control;  // state for control

  /********************* setPoints   ***************************************/
  // x(surge: m), y(sway: m), orientation(theta: rad)
  // in the global coordinate
  Eigen::Vector3d setPoints;  // setPoints

  /********************* coordinate transform matrix  **********************/
  Eigen::Matrix3d CTG2B;  // global --> body
  Eigen::Matrix3d CTB2G;  // body   --> global

  /********************* PID force   ***************************************/
  // << Fx, Fy, Mz (desired force) --- in the body coordinate
  Eigen::Vector3d tau;

  /********************* thruster allocation *******************************/
  // << Fx, Fy, Mz (estimated force)--- in the body coordinate
  Eigen::Vector3d BalphaU;
  Eigen::Vector3d alpha;      // rad, <<  bow_alpha, left_alpha, right_alpha
  Eigen::Vector3i alpha_deg;  // deg, <<  bow_alpha, left_alpha, right_alpha
  Eigen::Vector3d u;          // << bow_thrust, left_thrust, right_thrust
  Eigen::Vector3i rotation;   // rpm, << bow_n, left_n, right_n
};

// real time data of the second vessel (K class-II)
struct realtimevessel_second {
  /********************* raw motion data ***********************************/
  // x(surge: m), y(sway: m), z(heave: m), roll(deg), pitch(deg), yaw(theta:
  // deg)  data wroten by motion capture system
  Vector6d Position;  // raw motion data

  /********************* measured data after low pass  *********************/
  // x(m), y(m), orientation(theta: rad), u, v, r (next time stamp)
  // data wroten by motion capture system
  Vector6d Measurement;  // measured data after low pass

  /********************* state *********************************************/
  // x(surge: m), y(sway: m), yaw(theta: rad), u, v, r
  // data wroten by Kalman
  Vector6d State;  // state

  /********************* state for control  ********************************/
  // xb(surge: m), yb(sway: m), yaw(theta: rad), u, v, r
  // in the body-fixed coordinate
  Vector6d State4control;  // state for control

  /********************* setPoints   ***************************************/
  // x(surge: m), y(sway: m), orientation(theta: rad)
  // in the global coordinate
  Eigen::Vector3d setPoints;  // setPoints

  /********************* coordinate transform matrix  **********************/
  Eigen::Matrix3d CTG2B;  // global --> body
  Eigen::Matrix3d CTB2G;  // body   --> global

  /********************* PID force   ***************************************/
  // << Fx, Fy, Mz (desired force) --- in the body coordinate
  Eigen::Vector3d tau;

  /********************* thruster allocation *******************************/
  // << Fx, Fy, Mz (estimated force)--- in the body coordinate
  Eigen::Vector3d BalphaU;
  Eigen::Vector3d alpha;      // rad, <<  bow_alpha, left_alpha, right_alpha
  Eigen::Vector3i alpha_deg;  // deg, <<  bow_alpha, left_alpha, right_alpha
  Eigen::Vector3d u;          // << bow_thrust, left_thrust, right_thrust
  Eigen::Vector3i rotation;   // rpm, << bow_n, left_n, right_n
};

// real time data of the third vessel (X class)
struct realtimevessel_third {
  /********************* raw motion data ***********************************/
  // x(surge: m), y(sway: m), z(heave: m), roll(deg), pitch(deg), yaw(theta:
  // deg)  data wroten by motion capture system
  Vector6d Position;  // raw motion data

  /********************* measured data after low pass  *********************/
  // x(m), y(m), orientation(theta: rad), u, v, r (next time stamp)
  // data wroten by motion capture system
  Vector6d Measurement;  // measured data after low pass

  /********************* state *********************************************/
  // x(surge: m), y(sway: m), yaw(theta: rad), u, v, r
  // data wroten by Kalman
  Vector6d State;  // state

  /********************* state for control  ********************************/
  // xb(surge: m), yb(sway: m), yaw(theta: rad), u, v, r
  // in the body-fixed coordinate
  Vector6d State4control;  // state for control

  /********************* setPoints   ***************************************/
  // x(surge: m), y(sway: m), orientation(theta: rad)
  // in the global coordinate
  Eigen::Vector3d setPoints;  // setPoints

  /********************* coordinate transform matrix  **********************/
  Eigen::Matrix3d CTG2B;  // global --> body
  Eigen::Matrix3d CTB2G;  // body   --> global

  /********************* PID force   ***************************************/
  // << Fx, Fy, Mz (desired force) --- in the body coordinate
  Eigen::Vector3d tau;

  /********************* thruster allocation *******************************/
  // << Fx, Fy, Mz (estimated force)--- in the body coordinate
  Eigen::Vector3d BalphaU;
  Eigen::Vector3d alpha;      // rad, <<  bow_alpha, left_alpha, right_alpha
  Eigen::Vector3i alpha_deg;  // deg, <<  bow_alpha, left_alpha, right_alpha
  Eigen::Vector3d u;          // << bow_thrust, left_thrust, right_thrust
  Eigen::Vector3i rotation;   // rpm, << bow_n, left_n, right_n
};

#endif /* _REALTIMEDATA_H_ */