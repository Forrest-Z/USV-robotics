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

// go rotation
struct rotationaroundpoint {
  double rotation_center_x;  // m
  double rotation_center_y;  // m
  double rotation_speed;     // rad/s
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

// straightline move or rotation
struct SingleDimensionMove_triple {
  int orientation_adjustment_time;    // elapsed time for orientation
  int total_time_spent;               // total time for this task (s)
  double desired_initialx_first;      // m
  double desired_initialy_first;      // m
  double desired_initialtheta_first;  // rad
  double delta_value_first;           // delta value (distance or angle)
  int indicator_first;                // where to go

  double desired_initialx_second;      // m
  double desired_initialy_second;      // m
  double desired_initialtheta_second;  // rad
  double delta_value_second;           // delta value (distance or angle)
  int indicator_second;                // where to go

  double desired_initialx_third;      // m
  double desired_initialy_third;      // m
  double desired_initialtheta_third;  // rad
  double delta_value_third;           // delta value (distance or angle)
  int indicator_third;                // where to go

  // 0 ---- move forward
  // 1 ---- move starboard
  // 2 ---- move backward
  // 3 ---- move port
  // 4 ---- rotate clockwise
  // 5 ---- rotate anti-clockwise
  // 6 ---- fixed point
};

class setpoints {
 public:
  setpoints() {}
  ~setpoints() {}

  void gosingledimension_triple(
      realtimevessel_first &_realtimevessel_first,
      realtimevessel_second &_realtimevessel_second,
      realtimevessel_third &_realtimevessel_third, int _total_time_spent,
      double _desired_initialx_first, double _desired_initialy_first,
      double _desired_initialtheta_first, double _delta_value_first,
      int _indicator_first, double _desired_initialx_second,
      double _desired_initialy_second, double _desired_initialtheta_second,
      double _delta_value_second, int _indicator_second,
      double _desired_initialx_third, double _desired_initialy_third,
      double _desired_initialtheta_third, double _delta_value_third,
      int _indicator_third) {
    setSingleDimensionMovetripledata(
        mysingledimensionmove_triple, _total_time_spent,
        _desired_initialx_first, _desired_initialy_first,
        _desired_initialtheta_first, _delta_value_first, _indicator_first,
        _desired_initialx_second, _desired_initialy_second,
        _desired_initialtheta_second, _delta_value_second, _indicator_second,
        _desired_initialx_third, _desired_initialy_third,
        _desired_initialtheta_third, _delta_value_third, _indicator_third);
    singledimension_triple(_realtimevessel_first, _realtimevessel_second,
                           _realtimevessel_third, mysingledimensionmove_triple);
  }
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

  SingleDimensionMove_triple getsingledimensionmove_triple() const {
    return mysingledimensionmove_triple;
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
  SingleDimensionMove_triple mysingledimensionmove_triple{
      1,         //  orientation_adjustment_time
      100,       // total_time_spent
      0,         // desired_initialx_first
      -3,        // desired_initialy_first
      0,         // desired_initialtheta_first
      2,         // delta_value_first
      6,         // indicator_first
      0,         // desired_initialx_second
      3,         // desired_initialy_second
      0,         // desired_initialtheta_second
      3,         // delta_value_second
      6,         // indicator_second
      2,         // desired_initialx_third
      3,         // desired_initialy_third
      M_PI / 3,  // desired_initialtheta_third
      3,         // delta_value_third
      6          // indicator_third
  };
  // setup the fixedpoint data
  void setfixedpointdata(fixedpointdata &_fixedpointdata,
                         double _desired_finalx, double _desired_finaly,
                         double _desired_theta) {
    _fixedpointdata.desired_finalx = _desired_finalx;
    _fixedpointdata.desired_finaly = _desired_finaly;
    _fixedpointdata.desired_theta = _desired_theta;
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

  void setSingleDimensionMovetripledata(
      SingleDimensionMove_triple &_SingleDimensionMove_triple,
      int _total_time_spent, double _desired_initialx_first,
      double _desired_initialy_first, double _desired_initialtheta_first,
      double _delta_value_first, int _indicator_first,
      double _desired_initialx_second, double _desired_initialy_second,
      double _desired_initialtheta_second, double _delta_value_second,
      int _indicator_second, double _desired_initialx_third,
      double _desired_initialy_third, double _desired_initialtheta_third,
      double _delta_value_third, int _indicator_third) {
    _SingleDimensionMove_triple.total_time_spent = _total_time_spent;
    _SingleDimensionMove_triple.desired_initialx_first =
        _desired_initialx_first;
    _SingleDimensionMove_triple.desired_initialy_first =
        _desired_initialy_first;
    _SingleDimensionMove_triple.desired_initialtheta_first =
        _desired_initialtheta_first;
    _SingleDimensionMove_triple.delta_value_first = _delta_value_first;
    _SingleDimensionMove_triple.indicator_first = _indicator_first;

    _SingleDimensionMove_triple.desired_initialx_second =
        _desired_initialx_second;
    _SingleDimensionMove_triple.desired_initialy_second =
        _desired_initialy_second;
    _SingleDimensionMove_triple.desired_initialtheta_second =
        _desired_initialtheta_second;
    _SingleDimensionMove_triple.delta_value_second = _delta_value_second;
    _SingleDimensionMove_triple.indicator_second = _indicator_second;

    _SingleDimensionMove_triple.desired_initialx_third =
        _desired_initialx_third;
    _SingleDimensionMove_triple.desired_initialy_third =
        _desired_initialy_third;
    _SingleDimensionMove_triple.desired_initialtheta_third =
        _desired_initialtheta_third;
    _SingleDimensionMove_triple.delta_value_third = _delta_value_third;
    _SingleDimensionMove_triple.indicator_third = _indicator_third;
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
        _realtimevessel.setPoints(0) =
            total_delta_x * mt_elapsed / total_mt_elapsed + xA;
        _realtimevessel.setPoints(1) =
            total_delta_y * mt_elapsed / total_mt_elapsed + yA;
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
        restrictorientation(t_desired_theta);
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
        _realtimevessel.setPoints(0) =
            total_delta_x * mt_elapsed / total_mt_elapsed + xA;
        _realtimevessel.setPoints(1) =
            total_delta_y * mt_elapsed / total_mt_elapsed + yA;
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
        restrictorientation(t_desired_theta);
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
        _realtimevessel.setPoints(0) =
            total_delta_x * mt_elapsed / total_mt_elapsed + xA;
        _realtimevessel.setPoints(1) =
            total_delta_y * mt_elapsed / total_mt_elapsed + yA;
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
        restrictorientation(t_desired_theta);
        _realtimevessel.index_step =
            checksteppoint(_realtimevessel.setPoints, t_desired_theta);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } while (mt_elapsed < total_mt_elapsed);
    }
  }

  void singledimension_triple(
      realtimevessel_first &_realtimevessel_first,
      realtimevessel_second &_realtimevessel_second,
      realtimevessel_third &_realtimevessel_third,
      const SingleDimensionMove_triple &_SingleDimensionMove_triple) {
    // We reach the desired orientation first.
    double xA_first = _SingleDimensionMove_triple.desired_initialx_first;
    double yA_first = _SingleDimensionMove_triple.desired_initialy_first;
    double xA_second = _SingleDimensionMove_triple.desired_initialx_second;
    double yA_second = _SingleDimensionMove_triple.desired_initialy_second;
    double xA_third = _SingleDimensionMove_triple.desired_initialx_third;
    double yA_third = _SingleDimensionMove_triple.desired_initialy_third;
    _realtimevessel_first.setPoints(0) = xA_first;
    _realtimevessel_first.setPoints(1) = yA_first;
    _realtimevessel_second.setPoints(0) = xA_second;
    _realtimevessel_second.setPoints(1) = yA_second;
    _realtimevessel_third.setPoints(0) = xA_third;
    _realtimevessel_third.setPoints(1) = yA_third;
    _realtimevessel_first.index_step =
        checksteppoint(_realtimevessel_first.setPoints,
                       _SingleDimensionMove_triple.desired_initialtheta_first);
    _realtimevessel_second.index_step =
        checksteppoint(_realtimevessel_second.setPoints,
                       _SingleDimensionMove_triple.desired_initialtheta_second);
    _realtimevessel_third.index_step =
        checksteppoint(_realtimevessel_third.setPoints,
                       _SingleDimensionMove_triple.desired_initialtheta_third);

    std::this_thread::sleep_for(std::chrono::seconds(
        _SingleDimensionMove_triple.orientation_adjustment_time));

    long int total_mt_elapsed =
        (long int)(1000 * _SingleDimensionMove_triple.total_time_spent);
    // setup timer
    boost::posix_time::ptime t_start(T_BOOST_CLOCK::local_time());
    boost::posix_time::ptime t_end(T_BOOST_CLOCK::local_time());
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;

    // then we keep the straight line and reach the desired points (B)
    double theta_first =
        _SingleDimensionMove_triple.desired_initialtheta_first +
        M_PI * _SingleDimensionMove_triple.indicator_first / 2;
    double total_delta_x_first =
        std::cos(theta_first) * _SingleDimensionMove_triple.delta_value_first;
    double total_delta_y_first =
        std::sin(theta_first) * _SingleDimensionMove_triple.delta_value_first;

    double theta_second =
        _SingleDimensionMove_triple.desired_initialtheta_second +
        M_PI * _SingleDimensionMove_triple.indicator_second / 2;
    double total_delta_x_second =
        std::cos(theta_second) * _SingleDimensionMove_triple.delta_value_second;
    double total_delta_y_second =
        std::sin(theta_second) * _SingleDimensionMove_triple.delta_value_second;

    double theta_third =
        _SingleDimensionMove_triple.desired_initialtheta_third +
        M_PI * _SingleDimensionMove_triple.indicator_third / 2;
    double total_delta_x_third =
        std::cos(theta_third) * _SingleDimensionMove_triple.delta_value_third;
    double total_delta_y_third =
        std::sin(theta_third) * _SingleDimensionMove_triple.delta_value_third;

    double _desired_total_theta_first =
        (9 - 2 * _SingleDimensionMove_triple.indicator_first) *
        _SingleDimensionMove_triple.delta_value_first;

    double _desired_total_theta_second =
        (9 - 2 * _SingleDimensionMove_triple.indicator_second) *
        _SingleDimensionMove_triple.delta_value_second;

    double _desired_total_theta_third =
        (9 - 2 * _SingleDimensionMove_triple.indicator_third) *
        _SingleDimensionMove_triple.delta_value_third;
    double t_desired_theta_first = 0;
    double t_desired_theta_second = 0;
    double t_desired_theta_third = 0;

    do {
      t_end = T_BOOST_CLOCK::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      // the first vessel
      if (_SingleDimensionMove_triple.indicator_first <= 3) {
        _realtimevessel_first.setPoints(0) =
            total_delta_x_first * mt_elapsed / total_mt_elapsed + xA_first;
        _realtimevessel_first.setPoints(1) =
            total_delta_y_first * mt_elapsed / total_mt_elapsed + yA_first;
      } else if (_SingleDimensionMove_triple.indicator_first == 6) {
        _realtimevessel_first.setPoints(0) = xA_first;
        _realtimevessel_first.setPoints(1) = yA_first;
      } else {
        t_desired_theta_first =
            _desired_total_theta_first * mt_elapsed / total_mt_elapsed +
            _SingleDimensionMove_triple.desired_initialtheta_first;
        restrictorientation(t_desired_theta_first);
        _realtimevessel_first.index_step = checksteppoint(
            _realtimevessel_first.setPoints, t_desired_theta_first);
      }
      // the second vessel
      if (_SingleDimensionMove_triple.indicator_second <= 3) {
        _realtimevessel_second.setPoints(0) =
            total_delta_x_second * mt_elapsed / total_mt_elapsed + xA_second;
        _realtimevessel_second.setPoints(1) =
            total_delta_y_second * mt_elapsed / total_mt_elapsed + yA_second;
      } else if (_SingleDimensionMove_triple.indicator_second == 6) {
        _realtimevessel_second.setPoints(0) = xA_second;
        _realtimevessel_second.setPoints(1) = yA_second;
      } else {
        t_desired_theta_second =
            _desired_total_theta_second * mt_elapsed / total_mt_elapsed +
            _SingleDimensionMove_triple.desired_initialtheta_second;
        restrictorientation(t_desired_theta_second);
        _realtimevessel_second.index_step = checksteppoint(
            _realtimevessel_second.setPoints, t_desired_theta_second);
      }
      // the third vessel
      if (_SingleDimensionMove_triple.indicator_third <= 3) {
        _realtimevessel_third.setPoints(0) =
            total_delta_x_third * mt_elapsed / total_mt_elapsed + xA_third;
        _realtimevessel_third.setPoints(1) =
            total_delta_y_third * mt_elapsed / total_mt_elapsed + yA_third;
      } else if (_SingleDimensionMove_triple.indicator_third == 6) {
        _realtimevessel_third.setPoints(0) = xA_third;
        _realtimevessel_third.setPoints(1) = yA_third;
      } else {
        t_desired_theta_third =
            _desired_total_theta_third * mt_elapsed / total_mt_elapsed +
            _SingleDimensionMove_triple.desired_initialtheta_third;
        restrictorientation(t_desired_theta_third);
        _realtimevessel_third.index_step = checksteppoint(
            _realtimevessel_third.setPoints, t_desired_theta_third);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (mt_elapsed < total_mt_elapsed);
  }
  // go to a fixed point with any trajectory
  void gofixedpoint(Eigen::Vector3d &_setpoints,
                    const fixedpointdata &_fixedpointdata) {
    _setpoints(0) = _fixedpointdata.desired_finalx;
    _setpoints(1) = _fixedpointdata.desired_finaly;
  }

  // cooperation control for two vessels, considering the vessel state
  // void gostraightline_both(Eigen::Vector3d &_setpoints_first,
  //                          Eigen::Vector3d &_setpoints_second,
  //                          const strightlinedata &_strightlinedata_first,
  //                          const strightlinedata &_strightlinedata_second,
  //                          const Vector6d &_state_first,
  //                          const Vector6d &_state_second) {}

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
  // restrict the orientation to -M_PI ~ M_PI
  void restrictorientation(double &_orientation) {
    if (_orientation <= -M_PI) _orientation += (2 * M_PI);
    if (_orientation >= M_PI) _orientation -= (2 * M_PI);
  }
};
#endif /* _SETPOINTS_H_ */