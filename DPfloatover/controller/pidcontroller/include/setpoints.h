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
  double desired_finalx;
  double desired_finaly;
  double desired_theta;
};

// go straight line
struct strightlinedata {
  double desired_velocity;
  double desired_theta;
  double desired_finalx;
  double desired_finaly;
  double desired_initialx;
  double desired_initialy;

  int orientation_adjustment_time;  // elapsed time for orientation
                                    // adjustment(seconds)
};

// go rotation
struct rotationaroundpoint {
  double rotation_center_x;
  double rotation_center_y;
  double rotation_speed;
};

class setpoints {
 public:
  setpoints() {}
  ~setpoints() {}
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

 private:
  fixedpointdata myfixedpointdata_first{
      0.6,  // desired_finalx
      2,    // desired_finaly
      0     // desired_theta
  };
  fixedpointdata myfixedpointdata_second{
      0.0,  // desired_finalx
      -6,   // desired_finaly
      0     // desired_theta
  };
  fixedpointdata myfixedpointdata_third{
      0.0,  // desired_finalx
      -6,   // desired_finaly
      0     // desired_theta
  };
  strightlinedata mystrightlinedata_first{
      0.01,  // desired_velocity
      0,     // desired_theta
      0.6,   // desired_finalx
      2,     // desired_finaly
      0.6,   // desired_initialx
      0,     // desired_initialy
      100    // orientation_adjustment_time
  };
  strightlinedata mystrightlinedata_second{
      0.1,  // desired_velocity
      0,    // desired_theta
      0.0,  // desired_finalx
      -2,   // desired_finaly
      0.0,  // desired_initialx
      0,    // desired_initialy
      10    // orientation_adjustment_time
  };
  strightlinedata mystrightlinedata_third{
      0.1,  // desired_velocity
      0,    // desired_theta
      0.0,  // desired_finalx
      -2,   // desired_finaly
      0.0,  // desired_initialx
      0,    // desired_initialy
      10    // orientation_adjustment_time
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
    } while (mt_elapsed < total_mt_elapsed);
  }
};
#endif /* _SETPOINTS_H_ */