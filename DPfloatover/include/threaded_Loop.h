/*
***********************************************************************
* thread_Loop.hpp: thread-based DP controller and network
* function to run the whole loop on server (including PN server,
* 6D motion capture, Kalman, PID, thruster allocation, joystick,
* save2sqlite, viewer).
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _THREADLOOP_HPP_
#define _THREADLOOP_HPP_

#include <pthread.h>
#include <sys/prctl.h>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer.hpp>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <unordered_map>
#include "../controller/pidcontroller/include/controller.h"
#include "../controller/pidcontroller/include/setpoints.h"
#include "../joystick/include/gamepadmonitor.h"
#include "../motioncapture/include/motioncapture.h"
#include "../network/include/crccheck.h"
#include "../network/include/datapack.h"
#include "../network/include/pnserver_t.h"
#include "../sql/include/databasecpp.h"
#include "constants.h"

class threadloop {
 public:
  threadloop()
      : pnserver_status(1),
        mydb(defaultdbsavepath),
        index_controlmode_first(0),
        index_controlmode_second(0),
        index_controlmode_third(0),
        _controller_first(_vessel_first, _realtimevessel_first),
        _controller_second(_vessel_second, _realtimevessel_second),
        _controller_third(_vessel_third, _realtimevessel_third) {}
  ~threadloop() {}

  void initializelooop() {
    // initialize pn server
    pnd_init();
    mydb = databasecpp(dbsavepath);
    // sqlite to create master table
    mydb.create_mastertable();
  }
  // start socket server and wait for clients
  void start_connnection_t() {
    if (_SERV_CP_Startup() == 0) {
      _openpncontroller();
      pnserver_status = 0;
      pnd_test_set_mode(PNIO_MODE_OPERATE);
      createtables();
    }
  }
  // loop for send and recive data using socket
  void controller_t() {
    if (MAXCONNECTION > 0) {
      // thread for the first vessel
      std::thread _threadfirst(&threadloop::controller_first_pn, this);
      if (FILEORNOT) {  // join for terminal, detach for QT
        _tmclients[0] = _threadfirst.native_handle();
        _threadfirst.detach();
      } else
        _threadfirst.join();
    }
    if (MAXCONNECTION > 1) {
      // thread for the second vessel
      std::thread _threadsecond(&threadloop::controller_second_pn, this);
      if (FILEORNOT) {  // join for terminal, detach for QT
        _tmclients[1] = _threadsecond.native_handle();
        _threadsecond.detach();
      } else
        _threadsecond.join();
    }
    if (MAXCONNECTION > 2) {
      // thread for the third vessel
      std::thread _threadthird(&threadloop::controller_third_pn, this);
      if (FILEORNOT) {  // join for terminal, detach for QT
        _tmclients[2] = _threadthird.native_handle();
        _threadthird.detach();
      } else
        _threadthird.join();
    }
  }
  // profinet
  void send2allclients_pn_t() {
    std::thread _threadpnsend(&threadloop::send2allclients_pn, this);
    if (FILEORNOT) {  // join for terminal, detach for QT
      _threadid_pnsend = _threadpnsend.native_handle();
      _threadpnsend.detach();
    } else
      _threadpnsend.join();
  }
  // gampad
  void updategamepad_t() {
    std::thread _threadgamepad(&threadloop::updategamepad, this);
    if (FILEORNOT) {  // join for terminal, detach for QT
      _threadid_gamepad = _threadgamepad.native_handle();
      _threadgamepad.detach();
    } else
      _threadgamepad.join();
  }
  // motion capture
  void updatemotioncapture_t() {
    std::thread _threadmotioncapure(&threadloop::updatemotion, this);
    if (FILEORNOT) {  // join for terminal, detach for QT
      _threadid_motion = _threadmotioncapure.native_handle();
      _threadmotioncapure.detach();
    } else
      _threadmotioncapure.join();
  }
  // save data into sqlite database
  void save2database_t() {
    std::thread _thread(&threadloop::save2database, this);
    _threadid_database = _thread.native_handle();
    _thread.detach();
  }
  // update straightline setpoint using a thread
  void updatestraightlinesetpoints_t(double _desired_initialx,
                                     double _desired_initialy,
                                     double _desired_initialtheta,
                                     double _delta_value,
                                     double _desired_velocity, int _indicator,
                                     int index) {
    switch (index) {
      case 0: {
        std::thread _thread(&threadloop::setStraightline_first, this,
                            _desired_initialx, _desired_initialy,
                            _desired_initialtheta, _delta_value,
                            _desired_velocity, _indicator);
        _threadid_setpoints = _thread.native_handle();
        _thread.detach();
        break;
      }
      case 1: {
        std::thread _thread(&threadloop::setStraightline_second, this,
                            _desired_initialx, _desired_initialy,
                            _desired_initialtheta, _delta_value,
                            _desired_velocity, _indicator);
        _threadid_setpoints = _thread.native_handle();
        _thread.detach();
        break;
      }
      case 2: {
        std::thread _thread(&threadloop::setStraightline_third, this,
                            _desired_initialx, _desired_initialy,
                            _desired_initialtheta, _delta_value,
                            _desired_velocity, _indicator);
        _threadid_setpoints = _thread.native_handle();
        _thread.detach();
        break;
      }
    }
  }

  // update straightline setpoint using a thread
  void updatecooperationset_t(
      int _total_time_spent, double _desired_initialx_first,
      double _desired_initialy_first, double _desired_initialtheta_first,
      double _delta_value_first, int _indicator_first,
      double _desired_initialx_second, double _desired_initialy_second,
      double _desired_initialtheta_second, double _delta_value_second,
      int _indicator_second, double _desired_initialx_third,
      double _desired_initialy_third, double _desired_initialtheta_third,
      double _delta_value_third, int _indicator_third) {
    std::thread _thread(
        &threadloop::setStraightline_triple, this, _total_time_spent,
        _desired_initialx_first, _desired_initialy_first,
        _desired_initialtheta_first, _delta_value_first, _indicator_first,
        _desired_initialx_second, _desired_initialy_second,
        _desired_initialtheta_second, _delta_value_second, _indicator_second,
        _desired_initialx_third, _desired_initialy_third,
        _desired_initialtheta_third, _delta_value_third, _indicator_third);
    _threadid_setpoints = _thread.native_handle();
    _thread.detach();
  }

  // kill the thread of setpoints
  void closeupdatesetpoints() { pthread_cancel(_threadid_setpoints); }

  // reset all data, close controller, database and PN driver, and kill all
  // threads
  void closelooop() {
    for (int i = 0; i != MAXCONNECTION; ++i) stopmosekthread(i);
    resetallvessels();  // set zero of each vessel
    std::this_thread::sleep_for(std::chrono::seconds(3));
    pthread_cancel(_threadid_pnsend);

    closemotioncapture();  // close qtm clients

    // close all thread
    pthread_cancel(_threadid_gamepad);
    pthread_cancel(_threadid_motion);
    pthread_cancel(_threadid_database);

    _closepncontroller();  // close pn server
  }

  // setup the fixed setpoint of each vessel
  void setFixedpoint_first(double _setx, double _sety, double _settheta) {
    mysetpoints.gofixedpoint_first(_realtimevessel_first, _setx, _sety,
                                   _settheta);
  }
  void setFixedpoint_second(double _setx, double _sety, double _settheta) {
    mysetpoints.gofixedpoint_second(_realtimevessel_second, _setx, _sety,
                                    _settheta);
  }
  void setFixedpoint_third(double _setx, double _sety, double _settheta) {
    mysetpoints.gofixedpoint_third(_realtimevessel_third, _setx, _sety,
                                   _settheta);
  }

  // set pid of I vessel
  void setPID_first(double _P_x, double _P_y, double _P_theta, double _I_x,
                    double _I_y, double _I_theta, double _D_x, double _D_y,
                    double _D_theta) {
    _controller_first.setPID(_P_x, _P_y, _P_theta, _I_x, _I_y, _I_theta, _D_x,
                             _D_y, _D_theta);
  }
  // set pid of II vessel
  void setPID_second(double _P_x, double _P_y, double _P_theta, double _I_x,
                     double _I_y, double _I_theta, double _D_x, double _D_y,
                     double _D_theta) {
    _controller_second.setPID(_P_x, _P_y, _P_theta, _I_x, _I_y, _I_theta, _D_x,
                              _D_y, _D_theta);
  }
  // set pid of III vessel
  void setPID_third(double _P_x, double _P_y, double _P_theta, double _I_x,
                    double _I_y, double _I_theta, double _D_x, double _D_y,
                    double _D_theta) {
    _controller_third.setPID(_P_x, _P_y, _P_theta, _I_x, _I_y, _I_theta, _D_x,
                             _D_y, _D_theta);
  }
  // setup the control mode of I vessel
  void setcontrolmode_first(int _controlmode) {
    index_controlmode_first = _controlmode;
  }
  // setup the control mode of II vessel
  void setcontrolmode_second(int _controlmode) {
    index_controlmode_second = _controlmode;
  }
  // setup the control mode of III vessel
  void setcontrolmode_third(int _controlmode) {
    index_controlmode_third = _controlmode;
  }
  // setup the sqlite db path
  void setdbsavepath(const std::string &_projectname) {
    dbsavepath = "/home/skloe/Coding/CPP1X/USV/DPfloatover/QT/build/data/" +
                 _projectname + ".db";
  }

  int getgamepadstatus_first() const {
    return mygamepad_first.getGamepadStatus();
  }
  int getgamepadstatus_second() const {
    return mygamepad_second.getGamepadStatus();
  }
  int getgamepadstatus_third() const {
    return mygamepad_third.getGamepadStatus();
  }
  int getqtmstatus() const { return mymotioncapture.getmcapstatus(); }
  Vector6d getrealtimestate_first() const {
    return _realtimevessel_first.State;
  }
  int getpnserver_status() const { return pnserver_status; }
  Vector6d getrealtimestate_second() const {
    return _realtimevessel_second.State;
  }
  Vector6d getrealtimestate_third() const {
    return _realtimevessel_third.State;
  }
  Vector6d getrealtime6dmotionmeasurement_first() const {
    return _realtimevessel_first.Measurement;
  }
  Vector6d getrealtime6dmotionmeasurement_second() const {
    return _realtimevessel_second.Measurement;
  }
  Vector6d getrealtime6dmotionmeasurement_third() const {
    return _realtimevessel_third.Measurement;
  }
  Vector6d getrealtime6dmotion_first() const {
    return _realtimevessel_first.Position;
  }
  Vector6d getrealtime6dmotion_second() const {
    return _realtimevessel_second.Position;
  }
  Vector6d getrealtime6dmotion_third() const {
    return _realtimevessel_third.Position;
  }
  Eigen::Vector3i getrealtimealphadeg_first() const {
    return _realtimevessel_first.alpha_deg;
  }
  Eigen::Vector3i getrealtimealphadeg_second() const {
    return _realtimevessel_second.alpha_deg;
  }
  Eigen::Vector3i getrealtimealphadeg_third() const {
    return _realtimevessel_third.alpha_deg;
  }
  Eigen::Vector3i getrealtimerotation_first() const {
    return _realtimevessel_first.rotation;
  }
  Eigen::Vector3i getrealtimerotation_second() const {
    return _realtimevessel_second.rotation;
  }
  Eigen::Vector3i getrealtimerotation_third() const {
    return _realtimevessel_third.rotation;
  }
  Eigen::Vector3d getSetpoints_first() const {
    return _realtimevessel_first.setPoints;
  }
  Eigen::Vector3d getSetpoints_second() const {
    return _realtimevessel_second.setPoints;
  }
  Eigen::Vector3d getSetpoints_third() const {
    return _realtimevessel_third.setPoints;
  }
  Eigen::Vector3d getpmatrix_first() const {
    Eigen::Vector3d pmatrix = Eigen::Vector3d::Zero();
    pmatrix << _vessel_first.P_x, _vessel_first.P_y, _vessel_first.P_theta;
    return pmatrix;
  }
  Eigen::Vector3d getpmatrix_second() const {
    Eigen::Vector3d pmatrix = Eigen::Vector3d::Zero();
    pmatrix << _vessel_second.P_x, _vessel_second.P_y, _vessel_second.P_theta;
    return pmatrix;
  }
  Eigen::Vector3d getpmatrix_third() const {
    Eigen::Vector3d pmatrix = Eigen::Vector3d::Zero();
    pmatrix << _vessel_third.P_x, _vessel_third.P_y, _vessel_third.P_theta;
    return pmatrix;
  }
  Eigen::Vector3d getImatrix_first() const {
    Eigen::Vector3d pmatrix = Eigen::Vector3d::Zero();
    pmatrix << _vessel_first.I_x, _vessel_first.I_y, _vessel_first.I_theta;
    return pmatrix;
  }
  Eigen::Vector3d getImatrix_second() const {
    Eigen::Vector3d pmatrix = Eigen::Vector3d::Zero();
    pmatrix << _vessel_second.I_x, _vessel_second.I_y, _vessel_second.I_theta;
    return pmatrix;
  }
  Eigen::Vector3d getImatrix_third() const {
    Eigen::Vector3d pmatrix = Eigen::Vector3d::Zero();
    pmatrix << _vessel_third.I_x, _vessel_third.I_y, _vessel_third.I_theta;
    return pmatrix;
  }
  Eigen::Vector3d getDmatrix_first() const {
    Eigen::Vector3d pmatrix = Eigen::Vector3d::Zero();
    pmatrix << _vessel_first.D_x, _vessel_first.D_y, _vessel_first.D_theta;
    return pmatrix;
  }
  Eigen::Vector3d getDmatrix_second() const {
    Eigen::Vector3d pmatrix = Eigen::Vector3d::Zero();
    pmatrix << _vessel_second.D_x, _vessel_second.D_y, _vessel_second.D_theta;
    return pmatrix;
  }
  Eigen::Vector3d getDmatrix_third() const {
    Eigen::Vector3d pmatrix = Eigen::Vector3d::Zero();
    pmatrix << _vessel_third.D_x, _vessel_third.D_y, _vessel_third.D_theta;
    return pmatrix;
  }

  // get fixed setpoints data
  void getfixedpointdata_first(double &_desired_finalx, double &_desired_finaly,
                               double &_desired_theta) const {
    fixedpointdata _fixedpointdata = mysetpoints.getfixedpointdata_first();
    _desired_finalx = _fixedpointdata.desired_finalx;
    _desired_finaly = _fixedpointdata.desired_finaly;
    _desired_theta = _fixedpointdata.desired_theta * 180 / M_PI;
  }
  void getfixedpointdata_second(double &_desired_finalx,
                                double &_desired_finaly,
                                double &_desired_theta) const {
    fixedpointdata _fixedpointdata = mysetpoints.getfixedpointdata_second();
    _desired_finalx = _fixedpointdata.desired_finalx;
    _desired_finaly = _fixedpointdata.desired_finaly;
    _desired_theta = _fixedpointdata.desired_theta * 180 / M_PI;
  }
  void getfixedpointdata_third(double &_desired_finalx, double &_desired_finaly,
                               double &_desired_theta) const {
    fixedpointdata _fixedpointdata = mysetpoints.getfixedpointdata_third();
    _desired_finalx = _fixedpointdata.desired_finalx;
    _desired_finaly = _fixedpointdata.desired_finaly;
    _desired_theta = _fixedpointdata.desired_theta * 180 / M_PI;
  }
  // get straightline data
  void getstraightlinedata_first(double &_desired_initialx,
                                 double &_desired_initialy,
                                 double &_desired_initialtheta,
                                 double &_delta_value,
                                 double &_desired_velocity,
                                 int &_indicator) const {
    SingleDimensionMove _SingleDimensionMove =
        mysetpoints.getSingleDimensionMove_first();
    _desired_initialx = _SingleDimensionMove.desired_initialx;
    _desired_initialy = _SingleDimensionMove.desired_initialy;
    _desired_initialtheta =
        _SingleDimensionMove.desired_initialtheta * 180 / M_PI;
    _delta_value = _SingleDimensionMove.delta_value;
    _desired_velocity = _SingleDimensionMove.desired_velocity;
    _indicator = _SingleDimensionMove.indicator;
  }
  void getstraightlinedata_second(double &_desired_initialx,
                                  double &_desired_initialy,
                                  double &_desired_initialtheta,
                                  double &_delta_value,
                                  double &_desired_velocity,
                                  int &_indicator) const {
    SingleDimensionMove _SingleDimensionMove =
        mysetpoints.getSingleDimensionMove_second();
    _desired_initialx = _SingleDimensionMove.desired_initialx;
    _desired_initialy = _SingleDimensionMove.desired_initialy;
    _desired_initialtheta =
        _SingleDimensionMove.desired_initialtheta * 180 / M_PI;
    _delta_value = _SingleDimensionMove.delta_value;
    _desired_velocity = _SingleDimensionMove.desired_velocity;
    _indicator = _SingleDimensionMove.indicator;
  }
  void getstraightlinedata_third(double &_desired_initialx,
                                 double &_desired_initialy,
                                 double &_desired_initialtheta,
                                 double &_delta_value,
                                 double &_desired_velocity,
                                 int &_indicator) const {
    SingleDimensionMove _SingleDimensionMove =
        mysetpoints.getSingleDimensionMove_third();
    _desired_initialx = _SingleDimensionMove.desired_initialx;
    _desired_initialy = _SingleDimensionMove.desired_initialy;
    _desired_initialtheta =
        _SingleDimensionMove.desired_initialtheta * 180 / M_PI;
    _delta_value = _SingleDimensionMove.delta_value;
    _desired_velocity = _SingleDimensionMove.desired_velocity;
    _indicator = _SingleDimensionMove.indicator;
  }

  void getstraightlinedata_triple(
      int &_total_time_spent, double &_desired_initialx_first,
      double &_desired_initialy_first, double &_desired_initialtheta_first,
      double &_delta_value_first, int &_indicator_first,
      double &_desired_initialx_second, double &_desired_initialy_second,
      double &_desired_initialtheta_second, double &_delta_value_second,
      int &_indicator_second, double &_desired_initialx_third,
      double &_desired_initialy_third, double &_desired_initialtheta_third,
      double &_delta_value_third, int &_indicator_third) const {
    SingleDimensionMove_triple _SingleDimensionMove_triple =
        mysetpoints.getsingledimensionmove_triple();
    _total_time_spent = _SingleDimensionMove_triple.total_time_spent;

    _desired_initialx_first =
        _SingleDimensionMove_triple.desired_initialx_first;
    _desired_initialy_first =
        _SingleDimensionMove_triple.desired_initialy_first;
    _desired_initialtheta_first =
        _SingleDimensionMove_triple.desired_initialtheta_first * 180 / M_PI;
    _delta_value_first = _SingleDimensionMove_triple.delta_value_first;
    _indicator_first = _SingleDimensionMove_triple.indicator_first;

    _desired_initialx_second =
        _SingleDimensionMove_triple.desired_initialx_second;
    _desired_initialy_second =
        _SingleDimensionMove_triple.desired_initialy_second;
    _desired_initialtheta_second =
        _SingleDimensionMove_triple.desired_initialtheta_second * 180 / M_PI;
    _delta_value_second = _SingleDimensionMove_triple.delta_value_second;
    _indicator_second = _SingleDimensionMove_triple.indicator_second;

    _desired_initialx_third =
        _SingleDimensionMove_triple.desired_initialx_third;
    _desired_initialy_third =
        _SingleDimensionMove_triple.desired_initialy_third;
    _desired_initialtheta_third =
        _SingleDimensionMove_triple.desired_initialtheta_third * 180 / M_PI;
    _delta_value_third = _SingleDimensionMove_triple.delta_value_third;
    _indicator_third = _SingleDimensionMove_triple.indicator_third;
  }

 private:
  // thread pool
  typedef std::unordered_map<int, pthread_t> ThreadMap;
  ThreadMap _tmclients;  // the id array of thread for socket connection
  pthread_t _threadid_setpoints;  // the id of thread for setpoints
  pthread_t _threadid_database;   // the id of thread for saving data
  pthread_t _threadid_motion;     // the id of thread for motion capture
  pthread_t _threadid_gamepad;    // the id of thread for gamepad
  pthread_t _threadid_pnsend;     // the id of thread for gamepad
  //
  int pnserver_status;
  databasecpp mydb;
  gamepadmonitor_first mygamepad_first;
  gamepadmonitor_second mygamepad_second;
  gamepadmonitor_third mygamepad_third;
  motioncapture mymotioncapture;
  FILE *myfile;
  FILE *myfile_first;
  FILE *myfile_second;
  FILE *myfile_third;
  std::string dbsavepath;
  // index for control mode
  // 0: manual control
  // 1: heading control
  // 2: automatic control
  int index_controlmode_first;
  int index_controlmode_second;
  int index_controlmode_third;
  // constant parameters of the first vessel
  vessel_first _vessel_first{
      {623, 0, 0, 0, 706, 444, 0, 444, 1298},  // mass
      {17, 0, 0, 0, 20, 0, 0, 0, 100},         // damping
      20.0,                                    // P_x
      10.0,                                    // P_y
      50.0,                                    // P_theta
      0.0,                                     // I_x
      0.0,                                     // I_y
      0.0,                                     // I_theta
      300.0,                                   // D_x
      200.0,                                   // D_y
      300.0,                                   // D_theta
      0.01,                                    // allowed_error_x
      0.01,                                    // allowed_error_y;
      0.01,                                    // allowed_error_orientation;
      6.0,                                     // maxpositive_x_thrust(N)
      5.0,                                     // maxnegative_x_thrust(N)
      3,                                       // maxpositive_y_thrust(N)
      2,                                       // maxnegative_y_thrust(N)
      5,                                       // maxpositive_Mz_thrust(N*m)
      3,                                       // maxnegative_Mz_thrust(N*m)
      3,                                       // m
      3,                                       // n
      9,                                       // numvar
      3,                                       // num_constraints
      5.5e-7,                                  // Kbar_positive
      1.3e-7,                                  // Kbar_negative
      100,                                     // max_delta_rotation_bow
      3000,                                    // max_rotation_bow
      4.95,                                    // max_thrust_bow_positive
      1.17,                                    // max_thrust_bow_negative
      2e-5,                                    // K_left
      2e-5,                                    // K_right
      20,                                      // max_delta_rotation_azimuth
      1000,                                    // max_rotation_azimuth
      50,                                      // min_rotation_azimuth
      20,                                      // max_thrust_azimuth_left
      20,                                      // max_thrust_azimuth_right
      0.05,                                    // min_thrust_azimuth_left
      0.05,                                    // min_thrust_azimuth_right
      0.1277,                                  // max_delta_alpha_azimuth
      M_PI * 175 / 180,                        // max_alpha_azimuth_left
      M_PI / 18,                               // min_alpha_azimuth_left
      -M_PI / 18,                              // max_alpha_azimuth_right
      -M_PI * 175 / 180,                       // min_alpha_azimuth_right
      1.9,                                     // bow_x
      0,                                       // bow_y
      -1.893,                                  // left_x
      -0.216,                                  // left_y
      -1.893,                                  // right_x
      0.216                                    // right_y
  };
  // constant parameters of the second vessel
  vessel_second _vessel_second{
      {623, 0, 0, 0, 706, 444, 0, 444, 1298},  // mass
      {17, 0, 0, 0, 20, 0, 0, 0, 100},         // damping
      20,                                      // P_x
      10,                                      // P_y
      50.0,                                    // P_theta
      0,                                       // I_x
      0,                                       // I_y
      0.0,                                     // I_theta
      300.0,                                   // D_x
      200.0,                                   // D_y
      300.0,                                   // D_theta
      0.01,                                    // allowed_error_x
      0.01,                                    // allowed_error_y;
      0.02,                                    // allowed_error_orientation;
      6.0,                                     // maxpositive_x_thrust(N)
      5.0,                                     // maxnegative_x_thrust(N)
      3,                                       // maxpositive_y_thrust(N)
      2,                                       // maxnegative_y_thrust(N)
      5,                                       // maxpositive_Mz_thrust(N*m)
      3,                                       // maxnegative_Mz_thrust(N*m)
      3,                                       // m
      3,                                       // n
      9,                                       // numvar
      3,                                       // num_constraints
      3.7e-7,                                  // Kbar_positive
      1.7e-7,                                  // Kbar_negative
      100,                                     // max_delta_rotation_bow
      3000,                                    // max_rotation_bow
      3.33,                                    // max_thrust_bow_positive
      1.53,                                    // max_thrust_bow_negative
      2e-5,                                    // K_left
      2e-5,                                    // K_right
      20,                                      // max_delta_rotation_azimuth
      1000,                                    // max_rotation_azimuth
      50,                                      // min_rotation_azimuth
      20,                                      // max_thrust_azimuth_left
      20,                                      // max_thrust_azimuth_right
      0.05,                                    // min_thrust_azimuth_left
      0.05,                                    // min_thrust_azimuth_right
      0.1277,                                  // max_delta_alpha_azimuth
      M_PI * 175 / 180,                        // max_alpha_azimuth_left
      M_PI / 18,                               // min_alpha_azimuth_left
      -M_PI / 18,                              // max_alpha_azimuth_right
      -M_PI * 175 / 180,                       // min_alpha_azimuth_right
      1.9,                                     // bow_x
      0,                                       // bow_y
      -1.893,                                  // left_x
      -0.216,                                  // left_y
      -1.893,                                  // right_x
      0.216                                    // right_y
  };

  // constant parameters of the third vessel
  vessel_third _vessel_third{
      {1157, 0, 0, 0, 1311, 825, 0, 825, 2411},  // mass
      {34, 0, 0, 0, 40, 0, 0, 0, 200},           // damping
      20,                                        // P_x
      10,                                        // P_y
      50.0,                                      // P_theta
      0,                                         // I_x
      0,                                         // I_y
      0.0,                                       // I_theta
      300.0,                                     // D_x
      250.0,                                     // D_y
      300.0,                                     // D_theta
      0.01,                                      // allowed_error_x
      0.01,                                      // allowed_error_y;
      0.002,                                     // allowed_error_orientation;
      6.0,                                       // maxpositive_x_thrust(N)
      5.0,                                       // maxnegative_x_thrust(N)
      3,                                         // maxpositive_y_thrust(N)
      3,                                         // maxnegative_y_thrust(N)
      5,                                         // maxpositive_Mz_thrust(N*m)
      4,                                         // maxnegative_Mz_thrust(N*m)
      3,                                         // m
      3,                                         // n
      9,                                         // numvar
      3,                                         // num_constraints
      7.9e-7,                                    // Kbar_positive
      6.2e-7,                                    // Kbar_negative
      100,                                       // max_delta_rotation_bow
      3000,                                      // max_rotation_bow
      7.11,                                      // max_thrust_bow_positive
      5.58,                                      // max_thrust_bow_negative
      8e-7,                                      // K_left
      8e-7,                                      // K_right
      100,                                       // max_delta_rotation_azimuth
      3000,                                      // max_rotation_azimuth
      200,                                       // min_rotation_azimuth
      7.2,                                       // max_thrust_azimuth_left
      7.2,                                       // max_thrust_azimuth_right
      0.032,                                     // min_thrust_azimuth_left
      0.032,                                     // min_thrust_azimuth_right
      0.1277,                                    // max_delta_alpha_azimuth
      M_PI * 170 / 180,                          // max_alpha_azimuth_left
      M_PI / 18,                                 // min_alpha_azimuth_left
      -M_PI / 18,                                // max_alpha_azimuth_right
      -M_PI * 170 / 180,                         // min_alpha_azimuth_right
      2.563,                                     // bow_x
      0,                                         // bow_y
      -2.983,                                    // left_x
      -0.44,                                     // left_y
      -2.983,                                    // right_x
      0.44                                       // right_y
  };

  // realtime parameters of the first vessel (K class-I)
  realtimevessel_first _realtimevessel_first{
      Vector6d::Zero(),             // position
      Vector6d::Zero(),             // measurement
      Vector6d::Zero(),             // state
      Vector6d::Zero(),             // state4control
      Eigen::Vector3d::Zero(),      // setPoints
      0,                            // index for step point
      Eigen::Matrix3d::Identity(),  // CTG2B
      Eigen::Matrix3d::Identity(),  // CTB2G
      Eigen::Vector3d::Zero(),      // tau
      Eigen::Vector3d::Zero(),      // BalphaU
      (Eigen::Vector3d() << -M_PI / 2, M_PI / 10, -M_PI / 4)
          .finished(),                                   // alpha
      Eigen::Vector3i::Zero(),                           // alpha_deg
      (Eigen::Vector3d() << 0.01, 0.2, 0.2).finished(),  // u
      Eigen::Vector3i::Zero()                            // rotation
  };
  // realtime parameters of the second vessel (K class-II)
  realtimevessel_second _realtimevessel_second{
      Vector6d::Zero(),             // position
      Vector6d::Zero(),             // measurement
      Vector6d::Zero(),             // state
      Vector6d::Zero(),             // state4control
      Eigen::Vector3d::Zero(),      // setPoints
      0,                            // index for step point
      Eigen::Matrix3d::Identity(),  // CTG2B
      Eigen::Matrix3d::Identity(),  // CTB2G
      Eigen::Vector3d::Zero(),      // tau
      Eigen::Vector3d::Zero(),      // BalphaU
      (Eigen::Vector3d() << M_PI / 2, M_PI / 10, -M_PI / 4)
          .finished(),                                   // alpha
      Eigen::Vector3i::Zero(),                           // alpha_deg
      (Eigen::Vector3d() << 0.01, 0.2, 0.2).finished(),  // u
      Eigen::Vector3i::Zero()                            // rotation
  };
  // realtime parameters of the third vessel (X class)
  realtimevessel_third _realtimevessel_third{
      Vector6d::Zero(),             // position
      Vector6d::Zero(),             // measurement
      Vector6d::Zero(),             // state
      Vector6d::Zero(),             // state4control
      Eigen::Vector3d::Zero(),      // setPoints
      0,                            // index for step point
      Eigen::Matrix3d::Identity(),  // CTG2B
      Eigen::Matrix3d::Identity(),  // CTB2G
      Eigen::Vector3d::Zero(),      // tau
      Eigen::Vector3d::Zero(),      // BalphaU
      (Eigen::Vector3d() << -M_PI / 2, M_PI / 10, -M_PI / 4)
          .finished(),                                     // alpha
      Eigen::Vector3i::Zero(),                             // alpha_deg
      (Eigen::Vector3d() << 0.01, 0.02, 0.02).finished(),  // u
      Eigen::Vector3i::Zero()                              // rotation
  };
  // setpoints for control
  setpoints mysetpoints;
  // controller of each vessel
  controller_first _controller_first;
  controller_second _controller_second;
  controller_third _controller_third;

  // stop one thread for client's mosek solver
  void stopmosekthread(int threadindex) {
    ThreadMap::const_iterator it = _tmclients.find(threadindex);
    if (it != _tmclients.end()) {
      pthread_cancel(it->second);
      _tmclients.erase(threadindex);
    }
  }

  // get the real time gamepad response
  void updategamepad() {
    while (1) {
      if (MAXCONNECTION > 0) {
        mygamepad_first.updategamepad();
      }
      if (MAXCONNECTION > 1) {
        mygamepad_second.updategamepad();
      }
      if (MAXCONNECTION > 2) {
        mygamepad_third.updategamepad();
      }
    }
  }
  // get the real time motion response
  void updatemotion() {
    if (mymotioncapture.initializemotioncapture(myfile) == 0) {
      mymotioncapture.RequestPosition(_realtimevessel_first,
                                      _realtimevessel_second,
                                      _realtimevessel_third, myfile);
    }
  }
  void closemotioncapture() { mymotioncapture.stopRequest(myfile); }
  // save motion data to sqlite database
  void save2database() {
    while (1) {
      if (MAXCONNECTION > 0) {
        mydb.update_client_table(false, _realtimevessel_first);
      }
      if (MAXCONNECTION > 1) {
        mydb.update_client_table(false, _realtimevessel_second);
      }
      if (MAXCONNECTION > 2) {
        mydb.update_client_table(false, _realtimevessel_third);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // send and receive data from the first client (K class-I)
  void controller_first_pn() {
    boost::posix_time::ptime t_start(T_BOOST_CLOCK::local_time());
    boost::posix_time::ptime t_end(T_BOOST_CLOCK::local_time());
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;
    while (1) {
      // real-time control and optimization for each client
      t_start = T_BOOST_CLOCK::local_time();

      if (index_controlmode_first == 1) {
        _controller_first.headingcontrolleronestep(
            _realtimevessel_first, mygamepad_first.getGamepadXforce(),
            mygamepad_first.getGamepadYforce(), myfile_first);
      } else if (index_controlmode_first == 2) {
        _controller_first.pidcontrolleronestep(_realtimevessel_first,
                                               myfile_first);
      } else {
        _controller_first.fullymanualcontroller(
            mygamepad_first.getGamepadXforce(),
            mygamepad_first.getGamepadYforce(),
            mygamepad_first.getGamepadZmoment(), _realtimevessel_first,
            myfile_first);
      }
      t_end = T_BOOST_CLOCK::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      if (mt_elapsed > sample_mtime) {
        if (FILEORNOT) {
          myfile_first = fopen(logsavepath_first.c_str(), "a+");
          fprintf(myfile_first, "First: Take too long for QP!\n");
          fclose(myfile_first);
        } else
          perror("First: Take too long for QP");
        // continue;
      } else {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(sample_mtime - mt_elapsed));
      }
      // realtimeprint_first();
    }
  }

  // send and receive data from the second client (K class-II)
  void controller_second_pn() {
    boost::posix_time::ptime t_start(T_BOOST_CLOCK::local_time());
    boost::posix_time::ptime t_end(T_BOOST_CLOCK::local_time());
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;
    while (1) {
      // real-time control and optimization for each client
      t_start = T_BOOST_CLOCK::local_time();

      if (index_controlmode_second == 1) {
        _controller_second.headingcontrolleronestep(
            _realtimevessel_second, mygamepad_second.getGamepadXforce(),
            mygamepad_second.getGamepadYforce(), myfile_second);
      } else if (index_controlmode_second == 2) {
        _controller_second.pidcontrolleronestep(_realtimevessel_second,
                                                myfile_second);
      } else {
        _controller_second.fullymanualcontroller(
            mygamepad_second.getGamepadXforce(),
            mygamepad_second.getGamepadYforce(),
            mygamepad_second.getGamepadZmoment(), _realtimevessel_second,
            myfile_second);
      }
      t_end = T_BOOST_CLOCK::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      if (mt_elapsed > sample_mtime) {
        if (FILEORNOT) {
          myfile_second = fopen(logsavepath_second.c_str(), "a+");
          fprintf(myfile_second, "Second: Take too long for QP!\n");
          fclose(myfile_second);
        } else
          perror("Second: Take too long for QP");
        // continue;
      } else {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(sample_mtime - mt_elapsed));
      }
      // realtimeprint_second();
    }
  }

  // send and receive data from the third client (X class-I)
  void controller_third_pn() {
    boost::posix_time::ptime t_start(T_BOOST_CLOCK::local_time());
    boost::posix_time::ptime t_end(T_BOOST_CLOCK::local_time());
    boost::posix_time::time_duration t_elapsed = t_end - t_start;
    long int mt_elapsed = 0;
    while (1) {
      // real-time control and optimization for each client
      t_start = T_BOOST_CLOCK::local_time();

      if (index_controlmode_third == 1) {
        _controller_third.headingcontrolleronestep(
            _realtimevessel_third, mygamepad_third.getGamepadXforce(),
            mygamepad_third.getGamepadYforce(), myfile_third);
      } else if (index_controlmode_third == 2) {
        _controller_third.pidcontrolleronestep(_realtimevessel_third,
                                               myfile_third);
      } else {
        _controller_third.fullymanualcontroller(
            mygamepad_third.getGamepadXforce(),
            mygamepad_third.getGamepadYforce(),
            mygamepad_third.getGamepadZmoment(), _realtimevessel_third,
            myfile_third);
      }
      t_end = T_BOOST_CLOCK::local_time();
      t_elapsed = t_end - t_start;
      mt_elapsed = t_elapsed.total_milliseconds();
      if (mt_elapsed > sample_mtime) {
        if (FILEORNOT) {
          myfile_third = fopen(logsavepath_third.c_str(), "a+");
          fprintf(myfile_third, "Third: Take too long for QP!\n");
          fclose(myfile_third);
        } else
          perror("Third: Take too long for QP");
        // continue;
      } else {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(sample_mtime - mt_elapsed));
      }
      realtimeprint_third();
    }
  }

  // function for send data to all clients using PN
  void send2allclients_pn() {
    if (MAXCONNECTION == 1) {
      send2firstvessel(&_realtimevessel_first, myfile);
    } else if (MAXCONNECTION == 2) {
      send2bothvessel(&_realtimevessel_first, &_realtimevessel_second, myfile);
    } else if (MAXCONNECTION == 3) {
      send2triplevessel(&_realtimevessel_first, &_realtimevessel_second,
                        &_realtimevessel_third, myfile);
    }
  }
  // create sqlite database
  void createtables() {
    for (int i = 0; i != MAXCONNECTION; ++i) {
      mydb.update_mastertable(i);
      mydb.create_client_table(i);
    }
  }

  // setup the straightline of each vessel
  void setStraightline_first(double _desired_initialx, double _desired_initialy,
                             double _desired_initialtheta, double _delta_value,
                             double _desired_velocity, int _indicator) {
    mysetpoints.gosingledimension_first(
        _realtimevessel_first, _desired_initialx, _desired_initialy,
        _desired_initialtheta, _delta_value, _desired_velocity, _indicator);
  }
  void setStraightline_second(double _desired_initialx,
                              double _desired_initialy,
                              double _desired_initialtheta, double _delta_value,
                              double _desired_velocity, int _indicator) {
    mysetpoints.gosingledimension_second(
        _realtimevessel_second, _desired_initialx, _desired_initialy,
        _desired_initialtheta, _delta_value, _desired_velocity, _indicator);
  }
  void setStraightline_third(double _desired_initialx, double _desired_initialy,
                             double _desired_initialtheta, double _delta_value,
                             double _desired_velocity, int _indicator) {
    mysetpoints.gosingledimension_third(
        _realtimevessel_third, _desired_initialx, _desired_initialy,
        _desired_initialtheta, _delta_value, _desired_velocity, _indicator);
  }

  void setStraightline_triple(
      int _total_time_spent, double _desired_initialx_first,
      double _desired_initialy_first, double _desired_initialtheta_first,
      double _delta_value_first, int _indicator_first,
      double _desired_initialx_second, double _desired_initialy_second,
      double _desired_initialtheta_second, double _delta_value_second,
      int _indicator_second, double _desired_initialx_third,
      double _desired_initialy_third, double _desired_initialtheta_third,
      double _delta_value_third, int _indicator_third) {
    mysetpoints.gosingledimension_triple(
        _realtimevessel_first, _realtimevessel_second, _realtimevessel_third,
        _total_time_spent, _desired_initialx_first, _desired_initialy_first,
        _desired_initialtheta_first, _delta_value_first, _indicator_first,
        _desired_initialx_second, _desired_initialy_second,
        _desired_initialtheta_second, _delta_value_second, _indicator_second,
        _desired_initialx_third, _desired_initialy_third,
        _desired_initialtheta_third, _delta_value_third, _indicator_third);
  }
  // reset realtime data of each vessel
  void resetallvessels() {
    // reset the data of the first vessel
    _realtimevessel_first.Measurement.setZero();
    _realtimevessel_first.Position.setZero();
    _realtimevessel_first.State.setZero();
    _realtimevessel_first.tau.setZero();
    _realtimevessel_first.BalphaU.setZero();
    _realtimevessel_first.alpha.setZero();
    _realtimevessel_first.alpha_deg.setZero();
    _realtimevessel_first.u.setZero();
    _realtimevessel_first.rotation.setZero();
    // reset the data of the second vessel
    _realtimevessel_second.Measurement.setZero();
    _realtimevessel_second.Position.setZero();
    _realtimevessel_second.State.setZero();
    _realtimevessel_second.tau.setZero();
    _realtimevessel_second.BalphaU.setZero();
    _realtimevessel_second.alpha.setZero();
    _realtimevessel_second.alpha_deg.setZero();
    _realtimevessel_second.u.setZero();
    _realtimevessel_second.rotation.setZero();
    // reset the data of the third vessel
    _realtimevessel_third.Measurement.setZero();
    _realtimevessel_third.Position.setZero();
    _realtimevessel_third.State.setZero();
    _realtimevessel_third.tau.setZero();
    _realtimevessel_third.BalphaU.setZero();
    _realtimevessel_third.alpha.setZero();
    _realtimevessel_third.alpha_deg.setZero();
    _realtimevessel_third.u.setZero();
    _realtimevessel_third.rotation.setZero();
  }  // setZero for realtime data of each vessel

  void realtimeprint_first() {
    std::cout << "First: Desired force:" << std::endl
              << _realtimevessel_first.tau << std::endl;
    std::cout << "First:Estimated force:" << std::endl
              << _realtimevessel_first.BalphaU << std::endl;
    std::cout << "First: setPoints:" << std::endl
              << _realtimevessel_first.setPoints << std::endl;
  }
  void realtimeprint_second() {
    std::cout << "Second: Desired force:" << std::endl
              << _realtimevessel_second.tau << std::endl;
    std::cout << "Second: Estimated force:" << std::endl
              << _realtimevessel_second.BalphaU << std::endl;
    std::cout << "Second: setPoints:" << std::endl
              << _realtimevessel_second.setPoints << std::endl;
    std::cout << "Second: CTG2B:" << std::endl
              << _realtimevessel_second.CTG2B << std::endl;
    std::cout << "Second: CTB2G:" << std::endl
              << _realtimevessel_second.CTB2G << std::endl;
  }
  void realtimeprint_third() {
    std::cout << "Desired force:" << std::endl
              << _realtimevessel_third.tau << std::endl;
    std::cout << "Estimated force:" << std::endl
              << _realtimevessel_third.BalphaU << std::endl;
    std::cout << "thruster angle:" << std::endl
              << _realtimevessel_third.alpha_deg << std::endl;
    std::cout << "thruster speed:" << std::endl
              << _realtimevessel_third.rotation << std::endl;
  }
};

#endif /* _THREADLOOP_HPP_*/
