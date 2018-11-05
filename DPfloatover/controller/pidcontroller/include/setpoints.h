/*
***********************************************************************
* setpoints.h:
* function for real time setpoints for controller
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/
#ifndef _SETPOINTS_H_
#define _SETPOINTS_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer.hpp>
#include <chrono>
#include <cmath>
#include <thread>
#include "constants.h"
#include "realtimedata.h"

// go to fixed point
struct fixedpointdata {
  double desired_finalx;  // m
  double desired_finaly;  // m
  double desired_theta;   // rad
};

// go straight line
struct strightlinedata {
  double desired_velocity;  // m/s
  double desired_theta;     // rad
  double desired_finalx;    // m
  double desired_finaly;    // m
  double desired_initialx;  // m
  double desired_initialy;  // m

  int orientation_adjustment_time;  // elapsed time for orientation
                                    // adjustment(seconds)
};

// go box
struct boxdata {
  double desired_velocity;          // m/s
  double desired_theta;             // rad
  double desired_initialx;          // m
  double desired_initialy;          // m
  double deltax;                    // m
  double deltay;                    // m
  int orientation_adjustment_time;  // second
};

// go rotation
struct rotationaroundpoint {
  double rotation_center_x;  // m
  double rotation_center_y;  // m
  double rotation_speed;     // rad/s
};

// go straight line
struct strightlinedata_both {
  // common value
  double desired_velocity;          // m/s
  double desired_theta;             // rad
  int orientation_adjustment_time;  // adjustment(seconds)

  // I vessel
  double desired_initialx_first;  // m
  double desired_initialy_first;  // m
  double desired_finalx_first;    // m
  double desired_finaly_first;    // m

  // II vessel
  double desired_initialx_second;  // m
  double desired_initialy_second;  // m
  double desired_finalx_second;    // m
  double desired_finaly_second;    // m
};

class setpoints {
 public:
  setpoints() {}
  ~setpoints() {}
  // Enable cooperation control for two vesesls to follow a straight line
  void followstraightline_both(
      realtimevessel_first &_realtimevessel_first,
      realtimevessel_second &_realtimevessel_second, double _desired_velocity,
      double _desired_theta, double _desired_initialx_first,
      double _desired_initialy_first, double _desired_finalx_first,
      double _desired_finaly_first, double _desired_initialx_second,
      double _desired_initialy_second, double _desired_finalx_second,
      double _desired_finaly_second) {
    setstraightlinedata_both(mystrightlinedata_both, _desired_velocity,
                             _desired_theta, _desired_initialx_first,
                             _desired_initialy_first, _desired_finalx_first,
                             _desired_finaly_first, _desired_initialx_second,
                             _desired_initialy_second, _desired_finalx_second,
                             _desired_finaly_second);
    gostraightline_both(_realtimevessel_first.setPoints,
                        _realtimevessel_second.setPoints,
                        mystrightlinedata_both);
  }
  // Enable each vessel to reach a fixed point independently
  void gofixedpoint_first(realtimevessel_first &_realtimevessel,
                          double _desired_finalx, double _desired_finaly,
                          double _desired_theta) {
    setfixedpointdata(myfixedpointdata_first, _desired_finalx, _desired_finaly,
                      _desired_theta);
    gofixedpoint(_realtimevessel.setPoints, myfixedpointdata_first);
  }
  void gofixedpoint_second(realtimevessel_second &_realtimevessel,
                           double _desired_finalx, double _desired_finaly,
                           double _desired_theta) {
    setfixedpointdata(myfixedpointdata_second, _desired_finalx, _desired_finaly,
                      _desired_theta);
    gofixedpoint(_realtimevessel.setPoints, myfixedpointdata_second);
  }
  void gofixedpoint_third(realtimevessel_third &_realtimevessel,
                          double _desired_finalx, double _desired_finaly,
                          double _desired_theta) {
    setfixedpointdata(myfixedpointdata_third, _desired_finalx, _desired_finaly,
                      _desired_theta);
    gofixedpoint(_realtimevessel.setPoints, myfixedpointdata_third);
  }
  // Enable each vessel to go with a stright line independently
  void gostraightline_first(realtimevessel_first &_realtimevessel,
                            double _initialx, double _initialy,
                            double _desired_velocity, double _finalx,
                            double _finaly, double _desired_theta) {
    setstraightlinedata(mystrightlinedata_first, _initialx, _initialy,
                        _desired_velocity, _finalx, _finaly, _desired_theta);
    gostraightline(_realtimevessel.setPoints, mystrightlinedata_first);
  }
  void gostraightline_second(realtimevessel_second &_realtimevessel,
                             double _initialx, double _initialy,
                             double _desired_velocity, double _finalx,
                             double _finaly, double _desired_theta) {
    setstraightlinedata(mystrightlinedata_second, _initialx, _initialy,
                        _desired_velocity, _finalx, _finaly, _desired_theta);
    gostraightline(_realtimevessel.setPoints, mystrightlinedata_second);
  }
  void gostraightline_third(realtimevessel_third &_realtimevessel,
                            double _initialx, double _initialy,
                            double _desired_velocity, double _finalx,
                            double _finaly, double _desired_theta) {
    setstraightlinedata(mystrightlinedata_third, _initialx, _initialy,
                        _desired_velocity, _finalx, _finaly, _desired_theta);
    gostraightline(_realtimevessel.setPoints, mystrightlinedata_third);
  }

  // enable each vessel to go like a box
  void gobox_first(realtimevessel_first &_realtimevessel,
                   double _desired_velocity, double _desired_theta,
                   double _desired_initialx, double _desired_initialy,
                   double _deltax, double _deltay) {
    setboxdata(mybox_first, _desired_velocity, _desired_theta,
               _desired_initialx, _desired_initialy, _deltax, _deltay);
    gobox(_realtimevessel.setPoints, mybox_first);
  }
  void gobox_second(realtimevessel_second &_realtimevessel,
                    double _desired_velocity, double _desired_theta,
                    double _desired_initialx, double _desired_initialy,
                    double _deltax, double _deltay) {
    setboxdata(mybox_second, _desired_velocity, _desired_theta,
               _desired_initialx, _desired_initialy, _deltax, _deltay);
    gobox(_realtimevessel.setPoints, mybox_second);
  }

  fixedpointdata getfixedpointdata_first() const {
    return myfixedpointdata_first;
  }
  fixedpointdata getfixedpointdata_second() const {
    return myfixedpointdata_second;
  }
  fixedpointdata getfixedpointdata_third() const {
    return myfixedpointdata_third;
  }

  strightlinedata getstraightlinedata_first() const {
    return mystrightlinedata_first;
  }
  strightlinedata getstraightlinedata_second() const {
    return mystrightlinedata_second;
  }
  strightlinedata getstraightlinedata_third() const {
    return mystrightlinedata_third;
  }

  boxdata getboxdata_first() const { return mybox_first; }
  boxdata getboxdata_second() const { return mybox_second; }

  strightlinedata_both getstraightlinedata_both() const {
    return mystrightlinedata_both;
  }

 private:
  fixedpointdata myfixedpointdata_first{
      0.6,       // desired_finalx
      2,         // desired_finaly
      M_PI / 18  // desired_theta
  };
  fixedpointdata myfixedpointdata_second{
      0.0,  // desired_finalx
      -6,   // desired_finaly
      0     // desired_theta
  };
  fixedpointdata myfixedpointdata_third{
      0.0,       // desired_finalx
      -6,        // desired_finaly
      -M_PI / 3  // desired_theta
  };
  strightlinedata mystrightlinedata_first{
      0.01,      // desired_velocity
      M_PI / 2,  // desired_theta
      0.6,       // desired_finalx
      2,         // desired_finaly
      0.6,       // desired_initialx
      0,         // desired_initialy
      10         // orientation_adjustment_time
  };
  strightlinedata mystrightlinedata_second{
      0.1,        // desired_velocity
      -M_PI / 3,  // desired_theta
      0.0,        // desired_finalx
      -2,         // desired_finaly
      0.0,        // desired_initialx
      0,          // desired_initialy
      10          // orientation_adjustment_time
  };
  strightlinedata mystrightlinedata_third{
      0.1,       // desired_velocity
      M_PI / 2,  // desired_theta
      0.0,       // desired_finalx
      -2,        // desired_finaly
      0.0,       // desired_initialx
      0,         // desired_initialy
      10         // orientation_adjustment_time
  };

  boxdata mybox_first{
      0.1,       // desired_velocity
      M_PI / 2,  // desired_theta
      0.0,       // desired_initialx
      -2,        // desired_initialy
      0.0,       // deltax
      0,         // deltay
      10         // orientation_adjustment_time
  };

  boxdata mybox_second{
      0.1,       // desired_velocity
      M_PI / 2,  // desired_theta
      0.0,       // desired_initialx
      -2,        // desired_initialy
      0.0,       // deltax
      0,         // deltay
      10         // orientation_adjustment_time
  };

  strightlinedata_both mystrightlinedata_both{
      0.05,      // desired_velocity
      M_PI / 2,  // desired_theta
      10,        // orientation_adjustment_time
      0.0,       // desired_initialx_first
      0.1,       // desired_initialy_first
      0.0,       // desired_finalx_first
      -2,        // desired_finaly_first
      0.0,       // desired_initialx_second
      0,         // desired_initialy_second
      7.0,       // desired_finalx_second
      -3         // desired_finaly_second
  };
  // setup the fixedpoint data
  void setfixedpointdata(fixedpointdata &_fixedpointdata,
                         double _desired_finalx, double _desired_finaly,
                         double _desired_theta) {
    _fixedpointdata.desired_finalx = _desired_finalx;
    _fixedpointdata.desired_finaly = _desired_finaly;
    _fixedpointdata.desired_theta = _desired_theta;
  }

  // setup the box data
  void setboxdata(boxdata &_boxdata, double _desired_velocity,
                  double _desired_theta, double _desired_initialx,
                  double _desired_initialy, double _deltax, double _deltay) {
    _boxdata.desired_velocity = _desired_velocity;
    _boxdata.desired_theta = _desired_theta;
    _boxdata.desired_initialx = _desired_initialx;
    _boxdata.desired_initialy = _desired_initialy;
    _boxdata.deltax = _deltax;
    _boxdata.deltay = _deltay;
  }
  // setup the straight line data
  void setstraightlinedata(strightlinedata &_strightlinedata, double _initialx,
                           double _initialy, double _desired_velocity,
                           double _finalx, double _finaly,
                           double _desired_theta) {
    _strightlinedata.desired_velocity = _desired_velocity;
    _strightlinedata.desired_theta = _desired_theta;
    _strightlinedata.desired_finalx = _finalx;
    _strightlinedata.desired_finaly = _finaly;
    _strightlinedata.desired_initialx = _initialx;
    _strightlinedata.desired_initialy = _initialy;
  }
  // setup the straight line data for cooperation control (two vessels)
  void setstraightlinedata_both(
      strightlinedata_both &_strightlinedata_both, double _desired_velocity,
      double _desired_theta, double _desired_initialx_first,
      double _desired_initialy_first, double _desired_finalx_first,
      double _desired_finaly_first, double _desired_initialx_second,
      double _desired_initialy_second, double _desired_finalx_second,
      double _desired_finaly_second) {
    _strightlinedata_both.desired_velocity = _desired_velocity;
    _strightlinedata_both.desired_theta = _desired_theta;
    _strightlinedata_both.desired_initialx_first = _desired_initialx_first;
    _strightlinedata_both.desired_initialy_first = _desired_initialy_first;
    _strightlinedata_both.desired_finalx_first = _desired_finalx_first;
    _strightlinedata_both.desired_finaly_first = _desired_finaly_first;
    _strightlinedata_both.desired_initialx_second = _desired_initialx_second;
    _strightlinedata_both.desired_initialy_second = _desired_initialy_second;
    _strightlinedata_both.desired_finalx_second = _desired_finalx_second;
    _strightlinedata_both.desired_finaly_second = _desired_finaly_second;
  }
  // go to a fixed point with any trajectory
  void gofixedpoint(Eigen::Vector3d &_setpoints,
                    const fixedpointdata &_fixedpointdata) {
    _setpoints << _fixedpointdata.desired_finalx,
        _fixedpointdata.desired_finaly, _fixedpointdata.desired_theta;
  }

  // we keep the orientation and velocity to go a straight line, to a final
  // points
  void gostraightline(Eigen::Vector3d &_setpoints,
                      const strightlinedata &_strightlinedata) {
    // We reach the desired orientation first.
    _setpoints << _strightlinedata.desired_initialx,
        _strightlinedata.desired_initialy, _strightlinedata.desired_theta;
    std::this_thread::sleep_for(
        std::chrono::seconds(_strightlinedata.orientation_adjustment_time));

    // then we keep the straight line and reach the desired points
    double total_delta_x =
        _strightlinedata.desired_finalx - _strightlinedata.desired_initialx;
    double total_delta_y =
        _strightlinedata.desired_finaly - _strightlinedata.desired_initialy;
    double total_length = std::sqrt(total_delta_x * total_delta_x +
                                    total_delta_y * total_delta_y);

    // setup timer
    boost::posix_time::ptime t_start =
        boost::posix_time::second_clock::local_time();
    boost::posix_time::ptime t_end =
        boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;

    long int total_mt_elapsed =
        (long int)(1000 * total_length / _strightlinedata.desired_velocity);
    // update the desired position step by step
    do {
      t_end = boost::posix_time::second_clock::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed +
                      _strightlinedata.desired_initialx;
      _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed +
                      _strightlinedata.desired_initialy;
      _setpoints(2) = _strightlinedata.desired_theta;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (mt_elapsed < total_mt_elapsed);
  }

  // box for each vessel
  void gobox(Eigen::Vector3d &_setpoints, const boxdata &_boxdata) {
    // We reach the desired orientation first. (A)
    double xA = _boxdata.desired_initialx;
    double yA = _boxdata.desired_initialy;
    _setpoints << xA, yA, _boxdata.desired_theta;
    std::this_thread::sleep_for(
        std::chrono::seconds(_boxdata.orientation_adjustment_time));

    // then we keep the straight line and reach the desired points (B)
    double cvalue = std::cos(_boxdata.desired_theta);
    double svalue = std::sin(_boxdata.desired_theta);
    double total_delta_x = -svalue * _boxdata.deltay;
    double total_delta_y = cvalue * _boxdata.deltay;

    // setup timer
    boost::posix_time::ptime t_start =
        boost::posix_time::second_clock::local_time();
    boost::posix_time::ptime t_end =
        boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;

    long int total_mt_elapsed =
        (long int)(1000 * _boxdata.deltay / _boxdata.desired_velocity);
    // update the desired position step by step
    do {
      t_end = boost::posix_time::second_clock::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed + xA;
      _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed + yA;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (mt_elapsed < total_mt_elapsed);

    // B
    double xB = xA + total_delta_x;
    double yB = yA + total_delta_y;
    _setpoints << xB, yB, _boxdata.desired_theta;
    std::this_thread::sleep_for(
        std::chrono::seconds(_boxdata.orientation_adjustment_time));

    total_delta_x = cvalue * _boxdata.deltax;
    total_delta_y = svalue * _boxdata.deltax;

    // setup timer
    t_start = boost::posix_time::second_clock::local_time();

    total_mt_elapsed =
        (long int)(1000 * _boxdata.deltax / _boxdata.desired_velocity);
    // update the desired position step by step
    do {
      t_end = boost::posix_time::second_clock::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed + xB;
      _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed + yB;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (mt_elapsed < total_mt_elapsed);

    // C
    double xC = xB + total_delta_x;
    double yC = yB + total_delta_y;
    _setpoints << xC, yC, _boxdata.desired_theta;
    std::this_thread::sleep_for(
        std::chrono::seconds(_boxdata.orientation_adjustment_time));

    total_delta_x = svalue * _boxdata.deltay;
    total_delta_y = -cvalue * _boxdata.deltay;

    // setup timer
    t_start = boost::posix_time::second_clock::local_time();

    total_mt_elapsed =
        (long int)(1000 * _boxdata.deltay / _boxdata.desired_velocity);
    // update the desired position step by step
    do {
      t_end = boost::posix_time::second_clock::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed + xC;
      _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed + yC;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (mt_elapsed < total_mt_elapsed);

    // D
    double xD = xC + total_delta_x;
    double yD = yC + total_delta_y;
    _setpoints << xD, yD, _boxdata.desired_theta;
    std::this_thread::sleep_for(
        std::chrono::seconds(_boxdata.orientation_adjustment_time));

    total_delta_x = -cvalue * _boxdata.deltax;
    total_delta_y = -svalue * _boxdata.deltax;

    // setup timer
    t_start = boost::posix_time::second_clock::local_time();

    total_mt_elapsed =
        (long int)(1000 * _boxdata.deltax / _boxdata.desired_velocity);
    // update the desired position step by step
    do {
      t_end = boost::posix_time::second_clock::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed + xD;
      _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed + yD;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (mt_elapsed < total_mt_elapsed);
  }
  // cooperation control for two vessels
  void gostraightline_both(Eigen::Vector3d &_setpoints_first,
                           Eigen::Vector3d &_setpoints_second,
                           const strightlinedata_both &_strightlinedata_both) {
    // We reach the desired orientation first.
    _setpoints_first << _strightlinedata_both.desired_initialx_first,
        _strightlinedata_both.desired_initialy_first,
        _strightlinedata_both.desired_theta;
    _setpoints_second << _strightlinedata_both.desired_initialx_second,
        _strightlinedata_both.desired_initialy_second,
        _strightlinedata_both.desired_theta;
    std::this_thread::sleep_for(std::chrono::seconds(
        _strightlinedata_both.orientation_adjustment_time));

    // then we keep the straight line and reach the desired points
    double total_delta_x_first = _strightlinedata_both.desired_finalx_first -
                                 _strightlinedata_both.desired_initialx_first;
    double total_delta_y_first = _strightlinedata_both.desired_finaly_first -
                                 _strightlinedata_both.desired_initialy_first;
    double total_length_first =
        std::sqrt(total_delta_x_first * total_delta_x_first +
                  total_delta_y_first * total_delta_y_first);

    double total_delta_x_second = _strightlinedata_both.desired_finalx_second -
                                  _strightlinedata_both.desired_initialx_second;
    double total_delta_y_second = _strightlinedata_both.desired_finaly_second -
                                  _strightlinedata_both.desired_initialy_second;
    double total_length_second =
        std::sqrt(total_delta_x_second * total_delta_x_second +
                  total_delta_y_second * total_delta_y_second);
    // setup timer
    boost::posix_time::ptime t_start =
        boost::posix_time::second_clock::local_time();
    boost::posix_time::ptime t_end =
        boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;

    long int total_mt_elapsed =
        (long int)(500 * (total_length_first + total_length_second) /
                   _strightlinedata_both.desired_velocity);
    // update the desired position step by step
    do {
      t_end = boost::posix_time::second_clock::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      _setpoints_first(0) =
          total_delta_x_first * mt_elapsed / total_mt_elapsed +
          _strightlinedata_both.desired_initialx_first;
      _setpoints_first(1) =
          total_delta_y_first * mt_elapsed / total_mt_elapsed +
          _strightlinedata_both.desired_initialy_first;
      _setpoints_first(2) = _strightlinedata_both.desired_theta;

      _setpoints_second(0) =
          total_delta_x_second * mt_elapsed / total_mt_elapsed +
          _strightlinedata_both.desired_initialx_second;
      _setpoints_second(1) =
          total_delta_y_second * mt_elapsed / total_mt_elapsed +
          _strightlinedata_both.desired_initialy_second;
      _setpoints_second(2) = _strightlinedata_both.desired_theta;

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (mt_elapsed < total_mt_elapsed);
  }  // gostraightline_both

  // cooperation control for two vessels, considering the vessel state
  void gostraightline_both(Eigen::Vector3d &_setpoints_first,
                           Eigen::Vector3d &_setpoints_second,
                           const strightlinedata &_strightlinedata_first,
                           const strightlinedata &_strightlinedata_second,
                           const Vector6d &_state_first,
                           const Vector6d &_state_second) {}
};
#endif /* _SETPOINTS_H_ */