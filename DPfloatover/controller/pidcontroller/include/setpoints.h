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

// straightline move or rotation
struct SingleDimensionMove {
  double desired_initialx;          // m
  double desired_initialy;          // m
  double desired_initialtheta;      // rad
  int orientation_adjustment_time;  // elapsed time for orientation
                                    // adjustment(seconds)
  double delta_value;               // delta value (distance or angle)
  double desired_velocity;          // velocity (m/s or rad/s)
  int indicator;                    // where to go
  // 0 ---- move forward
  // 1 ---- move starboard
  // 2 ---- move backward
  // 3 ---- move port
  // 4 ---- rotate clockwise
  // 5 ---- rotate anti-clockwise
};

class setpoints {
 public:
  setpoints() {}
  ~setpoints() {}

  void gosingledimension_first(realtimevessel_first &_realtimevessel,
                               double _desired_initialx,
                               double _desired_initialy,
                               double _desired_initialtheta,
                               double _delta_value, double _desired_velocity,
                               int _indicator) {
    setSingleDimensionMovedata(mySingleDimensionMove_first, _desired_initialx,
                               _desired_initialy, _desired_initialtheta,
                               _delta_value, _desired_velocity, _indicator);
    singledimension_first(_realtimevessel, mySingleDimensionMove_first);
  }
  void gosingledimension_second(realtimevessel_second &_realtimevessel,
                                double _desired_initialx,
                                double _desired_initialy,
                                double _desired_initialtheta,
                                double _delta_value, double _desired_velocity,
                                int _indicator) {
    setSingleDimensionMovedata(mySingleDimensionMove_second, _desired_initialx,
                               _desired_initialy, _desired_initialtheta,
                               _delta_value, _desired_velocity, _indicator);
    singledimension_second(_realtimevessel, mySingleDimensionMove_second);
  }
  void gosingledimension_third(realtimevessel_third &_realtimevessel,
                               double _desired_initialx,
                               double _desired_initialy,
                               double _desired_initialtheta,
                               double _delta_value, double _desired_velocity,
                               int _indicator) {
    setSingleDimensionMovedata(mySingleDimensionMove_third, _desired_initialx,
                               _desired_initialy, _desired_initialtheta,
                               _delta_value, _desired_velocity, _indicator);
    singledimension_third(_realtimevessel, mySingleDimensionMove_third);
  }
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
    _realtimevessel_first.index_step =
        checksteppoint(_realtimevessel_first.setPoints, _desired_theta);
    _realtimevessel_second.index_step =
        checksteppoint(_realtimevessel_second.setPoints, _desired_theta);
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
    _realtimevessel.index_step =
        checksteppoint(_realtimevessel.setPoints, _desired_theta);
    gofixedpoint(_realtimevessel.setPoints, myfixedpointdata_first);
  }
  void gofixedpoint_second(realtimevessel_second &_realtimevessel,
                           double _desired_finalx, double _desired_finaly,
                           double _desired_theta) {
    setfixedpointdata(myfixedpointdata_second, _desired_finalx, _desired_finaly,
                      _desired_theta);
    _realtimevessel.index_step =
        checksteppoint(_realtimevessel.setPoints, _desired_theta);
    gofixedpoint(_realtimevessel.setPoints, myfixedpointdata_second);
  }
  void gofixedpoint_third(realtimevessel_third &_realtimevessel,
                          double _desired_finalx, double _desired_finaly,
                          double _desired_theta) {
    setfixedpointdata(myfixedpointdata_third, _desired_finalx, _desired_finaly,
                      _desired_theta);
    _realtimevessel.index_step =
        checksteppoint(_realtimevessel.setPoints, _desired_theta);
    gofixedpoint(_realtimevessel.setPoints, myfixedpointdata_third);
  }

  // enable each vessel to go like a box
  void gobox_first(realtimevessel_first &_realtimevessel,
                   double _desired_velocity, double _desired_theta,
                   double _desired_initialx, double _desired_initialy,
                   double _deltax, double _deltay) {
    setboxdata(mybox_first, _desired_velocity, _desired_theta,
               _desired_initialx, _desired_initialy, _deltax, _deltay);
    _realtimevessel.index_step =
        checksteppoint(_realtimevessel.setPoints, _desired_theta);
    gobox(_realtimevessel.setPoints, mybox_first);
  }
  void gobox_second(realtimevessel_second &_realtimevessel,
                    double _desired_velocity, double _desired_theta,
                    double _desired_initialx, double _desired_initialy,
                    double _deltax, double _deltay) {
    setboxdata(mybox_second, _desired_velocity, _desired_theta,
               _desired_initialx, _desired_initialy, _deltax, _deltay);
    _realtimevessel.index_step =
        checksteppoint(_realtimevessel.setPoints, _desired_theta);
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

  SingleDimensionMove getSingleDimensionMove_first() const {
    return mySingleDimensionMove_first;
  }
  SingleDimensionMove getSingleDimensionMove_second() const {
    return mySingleDimensionMove_second;
  }
  SingleDimensionMove getSingleDimensionMove_third() const {
    return mySingleDimensionMove_third;
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

  SingleDimensionMove mySingleDimensionMove_first{
      0,     // desired_initialx;
      2,     // desired_initialy;
      0,     // desired_theta;
      10,    // orientation_adjustment_time
      2,     // delta_value;
      0.05,  // desired_velocity;
      0      // indicator;
  };
  SingleDimensionMove mySingleDimensionMove_second{
      0,     // desired_initialx;
      -3,    // desired_initialy;
      0,     // desired_theta;
      10,    // orientation_adjustment_time
      3,     // delta_value;
      0.05,  // desired_velocity;
      0      // indicator;
  };
  SingleDimensionMove mySingleDimensionMove_third{
      0,     // desired_initialx;
      2,     // desired_initialy;
      0,     // desired_theta;
      10,    // orientation_adjustment_time
      2,     // delta_value;
      0.05,  // desired_velocity;
      0      // indicator;
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
  void setSingleDimensionMovedata(SingleDimensionMove &_SingleDimensionMovedata,
                                  double _desired_initialx,
                                  double _desired_initialy,
                                  double _desired_initialtheta,
                                  double _delta_value, double _desired_velocity,
                                  int _indicator) {
    _SingleDimensionMovedata.desired_initialx = _desired_initialx;
    _SingleDimensionMovedata.desired_initialy = _desired_initialy;
    _SingleDimensionMovedata.desired_initialtheta = _desired_initialtheta;
    _SingleDimensionMovedata.delta_value = _delta_value;
    _SingleDimensionMovedata.desired_velocity = _desired_velocity;
    _SingleDimensionMovedata.indicator = _indicator;
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
  // go alongwith single dimension of the I vessel
  void singledimension_first(realtimevessel_first &_realtimevessel,
                             const SingleDimensionMove &_SingleDimensionMove) {
    // We reach the desired orientation first.
    double xA = _SingleDimensionMove.desired_initialx;
    double yA = _SingleDimensionMove.desired_initialy;
    _realtimevessel.setPoints(0) = xA;
    _realtimevessel.setPoints(1) = yA;
    _realtimevessel.index_step = checksteppoint(
        _realtimevessel.setPoints, _SingleDimensionMove.desired_initialtheta);

    std::this_thread::sleep_for(
        std::chrono::seconds(_SingleDimensionMove.orientation_adjustment_time));

    long int total_mt_elapsed =
        (long int)(1000 * _SingleDimensionMove.delta_value /
                   _SingleDimensionMove.desired_velocity);
    // setup timer
    boost::posix_time::ptime t_start(T_BOOST_CLOCK::local_time());
    boost::posix_time::ptime t_end(T_BOOST_CLOCK::local_time());
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;

    if (_SingleDimensionMove.indicator <= 3) {  // translation
      // then we keep the straight line and reach the desired points (B)
      double theta = _SingleDimensionMove.desired_initialtheta +
                     M_PI * _SingleDimensionMove.indicator / 2;
      double total_delta_x = std::cos(theta) * _SingleDimensionMove.delta_value;
      double total_delta_y = std::sin(theta) * _SingleDimensionMove.delta_value;

      // update the desired position step by step
      do {
        t_end = T_BOOST_CLOCK::local_time();
        t_elapsed = t_end - t_start;
        mt_elapsed = t_elapsed.total_milliseconds();
        _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed + xA;
        _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed + yA;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } while (mt_elapsed < total_mt_elapsed);
    } else {  // rotation
      double _desired_total_theta = (9 - 2 * _SingleDimensionMove.indicator) *
                                    _SingleDimensionMove.delta_value;
      double t_desired_theta = 0;
      // update the desired position step by step
      do {
        t_end = T_BOOST_CLOCK::local_time();
        t_elapsed = t_end - t_start;
        mt_elapsed = t_elapsed.total_milliseconds();
        t_desired_theta = _desired_total_theta * mt_elapsed / total_mt_elapsed +
                          _SingleDimensionMove.desired_initialtheta;
        _realtimevessel.index_step =
            checksteppoint(_realtimevessel.setPoints, t_desired_theta);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } while (mt_elapsed < total_mt_elapsed);
    }
  }
  // go alongwith single dimension of the II vessel
  void singledimension_second(realtimevessel_second &_realtimevessel,
                              const SingleDimensionMove &_SingleDimensionMove) {
    // We reach the desired orientation first.
    double xA = _SingleDimensionMove.desired_initialx;
    double yA = _SingleDimensionMove.desired_initialy;
    _realtimevessel.setPoints(0) = xA;
    _realtimevessel.setPoints(1) = yA;
    _realtimevessel.index_step = checksteppoint(
        _realtimevessel.setPoints, _SingleDimensionMove.desired_initialtheta);

    std::this_thread::sleep_for(
        std::chrono::seconds(_SingleDimensionMove.orientation_adjustment_time));

    long int total_mt_elapsed =
        (long int)(1000 * _SingleDimensionMove.delta_value /
                   _SingleDimensionMove.desired_velocity);
    // setup timer
    boost::posix_time::ptime t_start(T_BOOST_CLOCK::local_time());
    boost::posix_time::ptime t_end(T_BOOST_CLOCK::local_time());
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;

    if (_SingleDimensionMove.indicator <= 3) {  // translation
      // then we keep the straight line and reach the desired points (B)
      double theta = _SingleDimensionMove.desired_initialtheta +
                     M_PI * _SingleDimensionMove.indicator / 2;
      double total_delta_x = std::cos(theta) * _SingleDimensionMove.delta_value;
      double total_delta_y = std::sin(theta) * _SingleDimensionMove.delta_value;

      // update the desired position step by step
      do {
        t_end = T_BOOST_CLOCK::local_time();
        t_elapsed = t_end - t_start;
        mt_elapsed = t_elapsed.total_milliseconds();
        _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed + xA;
        _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed + yA;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } while (mt_elapsed < total_mt_elapsed);
    } else {  // rotation
      double _desired_total_theta = (9 - 2 * _SingleDimensionMove.indicator) *
                                    _SingleDimensionMove.delta_value;
      double t_desired_theta = 0;
      // update the desired position step by step
      do {
        t_end = T_BOOST_CLOCK::local_time();
        t_elapsed = t_end - t_start;
        mt_elapsed = t_elapsed.total_milliseconds();
        t_desired_theta = _desired_total_theta * mt_elapsed / total_mt_elapsed +
                          _SingleDimensionMove.desired_initialtheta;
        _realtimevessel.index_step =
            checksteppoint(_realtimevessel.setPoints, t_desired_theta);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } while (mt_elapsed < total_mt_elapsed);
    }
  }
  // go alongwith single dimension of the III vessel
  void singledimension_third(realtimevessel_third &_realtimevessel,
                             const SingleDimensionMove &_SingleDimensionMove) {
    // We reach the desired orientation first.
    double xA = _SingleDimensionMove.desired_initialx;
    double yA = _SingleDimensionMove.desired_initialy;
    _realtimevessel.setPoints(0) = xA;
    _realtimevessel.setPoints(1) = yA;
    _realtimevessel.index_step = checksteppoint(
        _realtimevessel.setPoints, _SingleDimensionMove.desired_initialtheta);

    std::this_thread::sleep_for(
        std::chrono::seconds(_SingleDimensionMove.orientation_adjustment_time));

    long int total_mt_elapsed =
        (long int)(1000 * _SingleDimensionMove.delta_value /
                   _SingleDimensionMove.desired_velocity);
    // setup timer
    boost::posix_time::ptime t_start(T_BOOST_CLOCK::local_time());
    boost::posix_time::ptime t_end(T_BOOST_CLOCK::local_time());
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;

    if (_SingleDimensionMove.indicator <= 3) {  // translation
      // then we keep the straight line and reach the desired points (B)
      double theta = _SingleDimensionMove.desired_initialtheta +
                     M_PI * _SingleDimensionMove.indicator / 2;
      double total_delta_x = std::cos(theta) * _SingleDimensionMove.delta_value;
      double total_delta_y = std::sin(theta) * _SingleDimensionMove.delta_value;

      // update the desired position step by step
      do {
        t_end = T_BOOST_CLOCK::local_time();
        t_elapsed = t_end - t_start;
        mt_elapsed = t_elapsed.total_milliseconds();
        _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed + xA;
        _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed + yA;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } while (mt_elapsed < total_mt_elapsed);
    } else {  // rotation
      double _desired_total_theta = (9 - 2 * _SingleDimensionMove.indicator) *
                                    _SingleDimensionMove.delta_value;
      double t_desired_theta = 0;
      // update the desired position step by step
      do {
        t_end = T_BOOST_CLOCK::local_time();
        t_elapsed = t_end - t_start;
        mt_elapsed = t_elapsed.total_milliseconds();
        t_desired_theta = _desired_total_theta * mt_elapsed / total_mt_elapsed +
                          _SingleDimensionMove.desired_initialtheta;
        _realtimevessel.index_step =
            checksteppoint(_realtimevessel.setPoints, t_desired_theta);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } while (mt_elapsed < total_mt_elapsed);
    }
  }
  // go to a fixed point with any trajectory
  void gofixedpoint(Eigen::Vector3d &_setpoints,
                    const fixedpointdata &_fixedpointdata) {
    _setpoints(0) = _fixedpointdata.desired_finalx;
    _setpoints(1) = _fixedpointdata.desired_finaly;
  }

  // box for each vessel
  void gobox(Eigen::Vector3d &_setpoints, const boxdata &_boxdata) {
    // We reach the desired orientation first. (A)
    double xA = _boxdata.desired_initialx;
    double yA = _boxdata.desired_initialy;
    _setpoints(0) = xA;
    _setpoints(1) = yA;
    std::this_thread::sleep_for(
        std::chrono::seconds(_boxdata.orientation_adjustment_time));

    // then we keep the straight line and reach the desired points (B)
    double cvalue = std::cos(_boxdata.desired_theta);
    double svalue = std::sin(_boxdata.desired_theta);
    double total_delta_x = -svalue * _boxdata.deltay;
    double total_delta_y = cvalue * _boxdata.deltay;

    // setup timer
    boost::posix_time::ptime t_start(T_BOOST_CLOCK::local_time());
    boost::posix_time::ptime t_end(T_BOOST_CLOCK::local_time());
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;

    long int total_mt_elapsed =
        (long int)(1000 * _boxdata.deltay / _boxdata.desired_velocity);
    // update the desired position step by step
    do {
      t_end = T_BOOST_CLOCK::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed + xA;
      _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed + yA;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (mt_elapsed < total_mt_elapsed);

    // B
    double xB = xA + total_delta_x;
    double yB = yA + total_delta_y;
    _setpoints(0) = xB;
    _setpoints(1) = yB;
    std::this_thread::sleep_for(
        std::chrono::seconds(_boxdata.orientation_adjustment_time));

    total_delta_x = cvalue * _boxdata.deltax;
    total_delta_y = svalue * _boxdata.deltax;

    // setup timer
    t_start = T_BOOST_CLOCK::local_time();

    total_mt_elapsed =
        (long int)(1000 * _boxdata.deltax / _boxdata.desired_velocity);
    // update the desired position step by step
    do {
      t_end = T_BOOST_CLOCK::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed + xB;
      _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed + yB;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (mt_elapsed < total_mt_elapsed);

    // C
    double xC = xB + total_delta_x;
    double yC = yB + total_delta_y;
    _setpoints(0) = xC;
    _setpoints(1) = yC;
    std::this_thread::sleep_for(
        std::chrono::seconds(_boxdata.orientation_adjustment_time));

    total_delta_x = svalue * _boxdata.deltay;
    total_delta_y = -cvalue * _boxdata.deltay;

    // setup timer
    t_start = T_BOOST_CLOCK::local_time();
    total_mt_elapsed =
        (long int)(1000 * _boxdata.deltay / _boxdata.desired_velocity);
    // update the desired position step by step
    do {
      t_end = T_BOOST_CLOCK::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      _setpoints(0) = total_delta_x * mt_elapsed / total_mt_elapsed + xC;
      _setpoints(1) = total_delta_y * mt_elapsed / total_mt_elapsed + yC;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (mt_elapsed < total_mt_elapsed);

    // D
    double xD = xC + total_delta_x;
    double yD = yC + total_delta_y;
    _setpoints(0) = xD;
    _setpoints(1) = yD;
    std::this_thread::sleep_for(
        std::chrono::seconds(_boxdata.orientation_adjustment_time));

    total_delta_x = -cvalue * _boxdata.deltax;
    total_delta_y = -svalue * _boxdata.deltax;

    // setup timer
    t_start = T_BOOST_CLOCK::local_time();

    total_mt_elapsed =
        (long int)(1000 * _boxdata.deltax / _boxdata.desired_velocity);
    // update the desired position step by step
    do {
      t_end = T_BOOST_CLOCK::local_time();
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
    _setpoints_first(0) = _strightlinedata_both.desired_initialx_first;
    _setpoints_first(1) = _strightlinedata_both.desired_initialy_first;
    _setpoints_second(0) = _strightlinedata_both.desired_initialx_second;
    _setpoints_second(1) = _strightlinedata_both.desired_initialy_second;
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
    boost::posix_time::ptime t_start(T_BOOST_CLOCK::local_time());
    boost::posix_time::ptime t_end(T_BOOST_CLOCK::local_time());
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;

    long int total_mt_elapsed =
        (long int)(500 * (total_length_first + total_length_second) /
                   _strightlinedata_both.desired_velocity);
    // update the desired position step by step
    do {
      t_end = T_BOOST_CLOCK::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      _setpoints_first(0) =
          total_delta_x_first * mt_elapsed / total_mt_elapsed +
          _strightlinedata_both.desired_initialx_first;
      _setpoints_first(1) =
          total_delta_y_first * mt_elapsed / total_mt_elapsed +
          _strightlinedata_both.desired_initialy_first;

      _setpoints_second(0) =
          total_delta_x_second * mt_elapsed / total_mt_elapsed +
          _strightlinedata_both.desired_initialx_second;
      _setpoints_second(1) =
          total_delta_y_second * mt_elapsed / total_mt_elapsed +
          _strightlinedata_both.desired_initialy_second;

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

  // judge the step point based on the setpoint orientation
  // index_step = 1, we use the 0~360 (add 360 when orientation is less than
  // zero)
  // index_step = 0, we use the -180~180 (use QTM raw data by default)
  int checksteppoint(Eigen::Vector3d &_setpoint, double _setpoint_orientation) {
    int index_step = 0;
    if ((M_PI / 2 < _setpoint_orientation) && (_setpoint_orientation <= M_PI)) {
      index_step = 1;
      _setpoint(2) = _setpoint_orientation;
    }
    if ((-M_PI <= _setpoint_orientation) &&
        (_setpoint_orientation < -M_PI / 2)) {
      index_step = 1;
      _setpoint(2) = 2 * M_PI + _setpoint_orientation;
    }
    if ((-M_PI / 2 <= _setpoint_orientation) &&
        (_setpoint_orientation <= M_PI / 2)) {
      index_step = 0;
      _setpoint(2) = _setpoint_orientation;
    }
    return index_step;
  }
};
#endif /* _SETPOINTS_H_ */