/*
***********************************************************************
* pidcontroller.h:
* function for pid controller
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

#include <cstdlib>
#include <iostream>
#include "constants.h"
#include "realtimedata.h"

const int IntegralLength = 300;
// class of pid controller for the first vessel
class pidcontroller_first {
 public:
  explicit pidcontroller_first(const vessel_first &_vessel) {
    initializePIDmatrix(_vessel);
    initializepidcontroller();
  }
  pidcontroller_first() = delete;
  ~pidcontroller_first() {}

  // calculate the desired force using PID controller
  void calculategeneralizeforce(realtimevessel_first &_realtimedata) {
    // convert setpoint from global to body coordinate
    setsetpoints(_realtimedata);
    // calculate error
    position_error = setpoints_body - _realtimedata.State4control.head(3);
    if (compareerror(position_error)) {
      _realtimedata.tau.setZero();
    } else {  // proportional term
      Eigen::Vector3d Pout = matrix_P * position_error;
      // integral term
      // updateIntegralMatrix(position_error);
      position_error_integral += sample_time * position_error;
      Eigen::Vector3d Iout = matrix_I * position_error_integral;
      // derivative term
      Eigen::Vector3d Dout = -matrix_D * _realtimedata.State4control.tail(3);
      // output
      _realtimedata.tau = Pout + Iout + Dout;
      // restrict desired force
      restrictdesiredforce(_realtimedata.tau);
    }
  }

  // change the P parameters
  void setPmatrix(double _P_x, double _P_y, double _P_theta) {
    matrix_P(0, 0) = _P_x;
    matrix_P(1, 1) = _P_y;
    matrix_P(2, 2) = _P_theta;
  }
  // change the I parameters
  void setImatrix(double _I_x, double _I_y, double _I_theta) {
    matrix_I(0, 0) = _I_x;
    matrix_I(1, 1) = _I_y;
    matrix_I(2, 2) = _I_theta;
  }
  // change the D parameters
  void setDmatrix(double _D_x, double _D_y, double _D_theta) {
    matrix_D(0, 0) = _D_x;
    matrix_D(1, 1) = _D_y;
    matrix_D(2, 2) = _D_theta;
  }

 private:
  // constant
  Eigen::Matrix3d matrix_P;
  Eigen::Matrix3d matrix_I;
  Eigen::Matrix3d matrix_D;
  Eigen::Vector3d allowed_error;

  // real time data
  Eigen::Vector3d position_error;
  Eigen::Vector3d position_error_integral;
  Eigen::Vector3d setpoints_body;
  Eigen::MatrixXd _IntegralMatrix;

  // max value of desired force
  double maxpositive_x_thruster;
  double maxnegative_x_thruster;
  double maxpositive_y_thruster;
  double maxnegative_y_thruster;
  double maxpositive_Mz_thruster;
  double maxnegative_Mz_thruster;
  // initialize the PID matrix
  void initializePIDmatrix(const vessel_first &_vessel) {
    // pid matrix
    matrix_P.setZero();
    matrix_I.setZero();
    matrix_D.setZero();
    matrix_P(0, 0) = _vessel.P_x;
    matrix_P(1, 1) = _vessel.P_y;
    matrix_P(2, 2) = _vessel.P_theta;
    matrix_I(0, 0) = _vessel.I_x;
    matrix_I(1, 1) = _vessel.I_y;
    matrix_I(2, 2) = _vessel.I_theta;
    matrix_D(0, 0) = _vessel.D_x;
    matrix_D(1, 1) = _vessel.D_y;
    matrix_D(2, 2) = _vessel.D_theta;
    // error
    allowed_error(0) = _vessel.allowed_error_x;
    allowed_error(1) = _vessel.allowed_error_y;
    allowed_error(2) = _vessel.allowed_error_orientation;

    // contraint
    maxpositive_x_thruster = _vessel.maxpositive_x_thruster;
    maxnegative_x_thruster = _vessel.maxnegative_x_thruster;
    maxpositive_y_thruster = _vessel.maxpositive_y_thruster;
    maxnegative_y_thruster = _vessel.maxnegative_y_thruster;
    maxpositive_Mz_thruster = _vessel.maxpositive_Mz_thruster;
    maxnegative_Mz_thruster = _vessel.maxnegative_Mz_thruster;
  }
  void initializepidcontroller() {
    position_error.setZero();
    position_error_integral.setZero();
    setpoints_body.setZero();
    _IntegralMatrix.setZero(IntegralLength, 3);
  }
  // specify the real time setpoints
  void setsetpoints(const realtimevessel_first &_realtimedata) {
    setpoints_body = _realtimedata.CTG2B * _realtimedata.setPoints;
  }
  // compare the real time error with the allowed error
  bool compareerror(const Eigen::Vector3d &_realtime_error) {
    for (int i = 0; i != 3; ++i)
      if (std::abs(_realtime_error(i)) > allowed_error(i)) return false;
    return true;
  }
  // restrict the desired force to some value
  void restrictdesiredforce(Eigen::Vector3d &_desiredforce) {
    if (_desiredforce(0) > maxpositive_x_thruster)
      _desiredforce(0) = maxpositive_x_thruster;
    if (_desiredforce(0) < -maxnegative_x_thruster)
      _desiredforce(0) = -maxnegative_x_thruster;
    if (_desiredforce(1) > maxpositive_y_thruster)
      _desiredforce(1) = maxpositive_y_thruster;
    if (_desiredforce(1) < -maxnegative_y_thruster)
      _desiredforce(1) = -maxnegative_y_thruster;
    if (_desiredforce(2) > maxpositive_Mz_thruster)
      _desiredforce(2) = maxpositive_Mz_thruster;
    if (_desiredforce(2) < -maxnegative_Mz_thruster)
      _desiredforce(2) = -maxnegative_Mz_thruster;
  }
  // calculate the Integral error with moving window
  void updateIntegralMatrix(const Eigen::Vector3d &_position_error) {
    Eigen::MatrixXd t_IntegralMatrix;
    t_IntegralMatrix.setZero(IntegralLength, 3);
    int index = IntegralLength - 1;
    t_IntegralMatrix.leftCols(index) = _IntegralMatrix.rightCols(index);
    t_IntegralMatrix.col(index) = sample_time * _position_error;
    _IntegralMatrix = t_IntegralMatrix;
    position_error_integral = _IntegralMatrix.rowwise().sum();
  }
};

// class of pid controller for the second vessel
class pidcontroller_second {
 public:
  explicit pidcontroller_second(const vessel_second &_vessel) {
    initializePIDmatrix(_vessel);
    initializepidcontroller();
  }
  pidcontroller_second() = delete;
  ~pidcontroller_second() {}

  // calculate the desired force using PID controller
  void calculategeneralizeforce(realtimevessel_second &_realtimedata) {
    // convert setpoint from global to body coordinate
    setsetpoints(_realtimedata);
    // calculate error
    position_error = setpoints_body - _realtimedata.State4control.head(3);
    if (compareerror(position_error)) {
      _realtimedata.tau.setZero();
    } else {  // proportional term
      Eigen::Vector3d Pout = matrix_P * position_error;
      // integral term
      position_error_integral += sample_time * position_error;
      Eigen::Vector3d Iout = matrix_I * position_error_integral;
      // derivative term
      Eigen::Vector3d Dout = -matrix_D * _realtimedata.State4control.tail(3);
      // output
      _realtimedata.tau = Pout + Iout + Dout;
      // restrict desired force
      restrictdesiredforce(_realtimedata.tau);
    }
  }

  // change the P parameters
  void setPmatrix(double _P_x, double _P_y, double _P_theta) {
    matrix_P(0, 0) = _P_x;
    matrix_P(1, 1) = _P_y;
    matrix_P(2, 2) = _P_theta;
  }
  // change the I parameters
  void setImatrix(double _I_x, double _I_y, double _I_theta) {
    matrix_I(0, 0) = _I_x;
    matrix_I(1, 1) = _I_y;
    matrix_I(2, 2) = _I_theta;
  }
  // change the D parameters
  void setDmatrix(double _D_x, double _D_y, double _D_theta) {
    matrix_D(0, 0) = _D_x;
    matrix_D(1, 1) = _D_y;
    matrix_D(2, 2) = _D_theta;
  }

 private:
  // constant
  Eigen::Matrix3d matrix_P;
  Eigen::Matrix3d matrix_I;
  Eigen::Matrix3d matrix_D;
  Eigen::Vector3d allowed_error;
  // real time data
  Eigen::Vector3d position_error;
  Eigen::Vector3d position_error_integral;
  Eigen::Vector3d setpoints_body;
  // max value of desired force
  double maxpositive_x_thruster;
  double maxnegative_x_thruster;
  double maxpositive_y_thruster;
  double maxnegative_y_thruster;
  double maxpositive_Mz_thruster;
  double maxnegative_Mz_thruster;
  // initialize the PID matrix
  void initializePIDmatrix(const vessel_second &_vessel) {
    // pid matrix
    matrix_P.setZero();
    matrix_I.setZero();
    matrix_D.setZero();
    matrix_P(0, 0) = _vessel.P_x;
    matrix_P(1, 1) = _vessel.P_y;
    matrix_P(2, 2) = _vessel.P_theta;
    matrix_I(0, 0) = _vessel.I_x;
    matrix_I(1, 1) = _vessel.I_y;
    matrix_I(2, 2) = _vessel.I_theta;
    matrix_D(0, 0) = _vessel.D_x;
    matrix_D(1, 1) = _vessel.D_y;
    matrix_D(2, 2) = _vessel.D_theta;
    // error
    allowed_error(0) = _vessel.allowed_error_x;
    allowed_error(1) = _vessel.allowed_error_y;
    allowed_error(2) = _vessel.allowed_error_orientation;
    // contraint
    maxpositive_x_thruster = _vessel.maxpositive_x_thruster;
    maxnegative_x_thruster = _vessel.maxnegative_x_thruster;
    maxpositive_y_thruster = _vessel.maxpositive_y_thruster;
    maxnegative_y_thruster = _vessel.maxnegative_y_thruster;
    maxpositive_Mz_thruster = _vessel.maxpositive_Mz_thruster;
    maxnegative_Mz_thruster = _vessel.maxnegative_Mz_thruster;
  }
  void initializepidcontroller() {
    position_error.setZero();
    position_error_integral.setZero();
    setpoints_body.setZero();
  }
  // specify the real time setpoints
  void setsetpoints(const realtimevessel_second &_realtimedata) {
    setpoints_body = _realtimedata.CTG2B * _realtimedata.setPoints;
  }
  // compare the real time error with the allowed error
  bool compareerror(const Eigen::Vector3d &_realtime_error) {
    for (int i = 0; i != 3; ++i)
      if (std::abs(_realtime_error(i)) > allowed_error(i)) return false;
    return true;
  }
  // restrict the desired force to some value
  void restrictdesiredforce(Eigen::Vector3d &_desiredforce) {
    if (_desiredforce(0) > maxpositive_x_thruster)
      _desiredforce(0) = maxpositive_x_thruster;
    if (_desiredforce(0) < -maxnegative_x_thruster)
      _desiredforce(0) = -maxnegative_x_thruster;
    if (_desiredforce(1) > maxpositive_y_thruster)
      _desiredforce(1) = maxpositive_y_thruster;
    if (_desiredforce(1) < -maxnegative_y_thruster)
      _desiredforce(1) = -maxnegative_y_thruster;
    if (_desiredforce(2) > maxpositive_Mz_thruster)
      _desiredforce(2) = maxpositive_Mz_thruster;
    if (_desiredforce(2) < -maxnegative_Mz_thruster)
      _desiredforce(2) = -maxnegative_Mz_thruster;
  }
};

// class of pid controller for the third vessel
class pidcontroller_third {
 public:
  explicit pidcontroller_third(const vessel_third &_vessel) {
    initializePIDmatrix(_vessel);
    initializepidcontroller();
  }
  pidcontroller_third() = delete;
  ~pidcontroller_third() {}

  // calculate the desired force using PID controller
  void calculategeneralizeforce(realtimevessel_third &_realtimedata) {
    // convert setpoint from global to body coordinate
    setsetpoints(_realtimedata);
    // calculate error
    position_error = setpoints_body - _realtimedata.State4control.head(3);
    if (compareerror(position_error)) {
      _realtimedata.tau.setZero();
    } else {  // proportional term
      Eigen::Vector3d Pout = matrix_P * position_error;
      // integral term
      position_error_integral += sample_time * position_error;
      Eigen::Vector3d Iout = matrix_I * position_error_integral;
      // derivative term
      Eigen::Vector3d Dout = -matrix_D * _realtimedata.State4control.tail(3);
      // output
      _realtimedata.tau = Pout + Iout + Dout;
      // restrict desired force
      restrictdesiredforce(_realtimedata.tau);
    }
  }

  // change the P parameters
  void setPmatrix(double _P_x, double _P_y, double _P_theta) {
    matrix_P(0, 0) = _P_x;
    matrix_P(1, 1) = _P_y;
    matrix_P(2, 2) = _P_theta;
  }
  // change the I parameters
  void setImatrix(double _I_x, double _I_y, double _I_theta) {
    matrix_I(0, 0) = _I_x;
    matrix_I(1, 1) = _I_y;
    matrix_I(2, 2) = _I_theta;
  }
  // change the D parameters
  void setDmatrix(double _D_x, double _D_y, double _D_theta) {
    matrix_D(0, 0) = _D_x;
    matrix_D(1, 1) = _D_y;
    matrix_D(2, 2) = _D_theta;
  }

 private:
  // constant
  Eigen::Matrix3d matrix_P;
  Eigen::Matrix3d matrix_I;
  Eigen::Matrix3d matrix_D;
  Eigen::Vector3d allowed_error;
  // real time data
  Eigen::Vector3d position_error;
  Eigen::Vector3d position_error_integral;
  Eigen::Vector3d setpoints_body;

  // max value of desired force
  double maxpositive_x_thruster;
  double maxnegative_x_thruster;
  double maxpositive_y_thruster;
  double maxnegative_y_thruster;
  double maxpositive_Mz_thruster;
  double maxnegative_Mz_thruster;
  // initialize the PID matrix
  void initializePIDmatrix(const vessel_third &_vessel) {
    // pid matrix
    matrix_P.setZero();
    matrix_I.setZero();
    matrix_D.setZero();
    matrix_P(0, 0) = _vessel.P_x;
    matrix_P(1, 1) = _vessel.P_y;
    matrix_P(2, 2) = _vessel.P_theta;
    matrix_I(0, 0) = _vessel.I_x;
    matrix_I(1, 1) = _vessel.I_y;
    matrix_I(2, 2) = _vessel.I_theta;
    matrix_D(0, 0) = _vessel.D_x;
    matrix_D(1, 1) = _vessel.D_y;
    matrix_D(2, 2) = _vessel.D_theta;
    // error
    allowed_error(0) = _vessel.allowed_error_x;
    allowed_error(1) = _vessel.allowed_error_y;
    allowed_error(2) = _vessel.allowed_error_orientation;

    // contraint
    maxpositive_x_thruster = _vessel.maxpositive_x_thruster;
    maxnegative_x_thruster = _vessel.maxnegative_x_thruster;
    maxpositive_y_thruster = _vessel.maxpositive_y_thruster;
    maxnegative_y_thruster = _vessel.maxnegative_y_thruster;
    maxpositive_Mz_thruster = _vessel.maxpositive_Mz_thruster;
    maxnegative_Mz_thruster = _vessel.maxnegative_Mz_thruster;
  }
  void initializepidcontroller() {
    position_error.setZero();
    position_error_integral.setZero();
    setpoints_body.setZero();
  }
  // specify the real time setpoints
  void setsetpoints(const realtimevessel_third &_realtimedata) {
    setpoints_body = _realtimedata.CTG2B * _realtimedata.setPoints;
  }
  // compare the real time error with the allowed error
  bool compareerror(const Eigen::Vector3d &_realtime_error) {
    for (int i = 0; i != 3; ++i)
      if (std::abs(_realtime_error(i)) > allowed_error(i)) return false;
    return true;
  }
  // restrict the desired force to some value
  void restrictdesiredforce(Eigen::Vector3d &_desiredforce) {
    if (_desiredforce(0) > maxpositive_x_thruster)
      _desiredforce(0) = maxpositive_x_thruster;
    if (_desiredforce(0) < -maxnegative_x_thruster)
      _desiredforce(0) = -maxnegative_x_thruster;
    if (_desiredforce(1) > maxpositive_y_thruster)
      _desiredforce(1) = maxpositive_y_thruster;
    if (_desiredforce(1) < -maxnegative_y_thruster)
      _desiredforce(1) = -maxnegative_y_thruster;
    if (_desiredforce(2) > maxpositive_Mz_thruster)
      _desiredforce(2) = maxpositive_Mz_thruster;
    if (_desiredforce(2) < -maxnegative_Mz_thruster)
      _desiredforce(2) = -maxnegative_Mz_thruster;
  }
};
#endif /* _PIDCONTROLLER_H_ */
