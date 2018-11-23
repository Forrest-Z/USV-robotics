#ifndef QTMOUTPUT_H
#define QTMOUTPUT_H

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <Eigen/Core>
#include <algorithm>
#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "RTProtocol.h"
#include "constants.h"
#include "qtmInput.h"
#include "realtimedata.h"

const double max_velocity_u = 1;              // m/s
const double max_velocity_v = 0.5;            // m/s
const double max_velocity_orientation = 0.1;  // rad/s
const double qtm_max_position = 30000;        // mm
const int num_average_point_yaw = 50;         // delay 2 s when frequenc=50Hz
const int num_average_point_surge = 50;       // delay 2 s when frequenc=50Hz
const int num_average_point_sway = 50;        // delay 2 s when frequenc=50
const int num_average_yaw_velocity = 20;      // delay 2 s when frequenc=50Hz
const int num_average_surge_velocity = 20;    // delay 2 s when frequenc=50Hz
const int num_average_sway_velocity = 20;     // delay 2 s when frequenc=50Hz
// average moving low pass to eliminate noise
using T_BOOST_CLOCK =
    boost::date_time::microsec_clock<boost::posix_time::ptime>;
typedef Eigen::Matrix<double, num_average_point_yaw, 1> VectorAYaw;
typedef Eigen::Matrix<double, num_average_point_surge, 1> VectorASurge;
typedef Eigen::Matrix<double, num_average_point_sway, 1> VectorASway;
typedef Eigen::Matrix<double, num_average_yaw_velocity, 1> VectorAYawV;
typedef Eigen::Matrix<double, num_average_surge_velocity, 1> VectorASurgeV;
typedef Eigen::Matrix<double, num_average_sway_velocity, 1> VectorASwayV;

class motiondataprocess {
 public:
  motiondataprocess();
  ~motiondataprocess();
  Eigen::Vector3d movingaverageposition(double _m_fx, double _m_fy,
                                        double _rad_orientation);
  Eigen::Vector3d movingaveragevelocity(
      const Eigen::Vector3d& _average_position_vector);

  // update real time orientation based on the index_step
  void updaterealtimeorientation(double& _rad_orientation, int _index_step);
  void setsampletime(double _sampletime);

 private:
  // low-pass position at former time step
  double frames_sample_time;
  Eigen::Vector3d formeraverageposition;
  VectorAYaw average_yaw;
  VectorASurge average_surge;
  VectorASway average_sway;
  VectorAYawV average_yaw_velocity;
  VectorASurgeV average_surge_velocity;
  VectorASwayV average_sway_velocity;

  void initializedata();
  double movingaverage_yaw(double _dtheta);
  double movingaverage_surge(double _dx);
  double movingaverage_sway(double _dy);
  double movingaverage_yaw_velocity(double _vtheta);
  double movingaverage_surge_velocity(double _vx);
  double movingaverage_sway_velocity(double _vy);
  Eigen::Vector3d cal_velocity(const Eigen::Vector3d& _average_position_vector);
};

class CDataPacket;

class COutput {
 public:
  COutput();
  void HandleDataFrame(FILE* logfile, CRTProtocol* poRTProtocol);
  void HandleDataFrame(FILE* logfile, CRTProtocol* poRTProtocol,
                       realtimevessel_first& _realtimevessel_first,
                       realtimevessel_second& _realtimevessel_second,
                       realtimevessel_third& _realtimevessel_third);
  void PrintTimingData();
  void ResetCounters();
  void setframes_elapsed_time(double _qtm_frames_elapsed_time);

 private:
  struct Marker {
    unsigned int nX;
    unsigned int nY;
  };

  void PrintData6D(FILE* logfile, CRTPacket* poRTPacket,
                   CRTProtocol* poRTProtocol);
  void PrintData6DRes(FILE* logfile, CRTPacket* poRTPacket,
                      CRTProtocol* poRTProtocol);
  void PrintData6DEuler(FILE* logfile, CRTPacket* poRTPacket,
                        CRTProtocol* poRTProtocol);
  void PrintData6DEuler(CRTPacket* poRTPacket,
                        realtimevessel_first& _realtimevessel_first);
  void PrintData6DEuler(CRTPacket* poRTPacket,
                        realtimevessel_first& _realtimevessel_first,
                        realtimevessel_second& _realtimevessel_second);
  void PrintData6DEuler(CRTPacket* poRTPacket,
                        realtimevessel_first& _realtimevessel_first,
                        realtimevessel_second& _realtimevessel_second,
                        realtimevessel_third& _realtimevessel_third);
  void PrintData6DEulerRes(FILE* logfile, CRTPacket* poRTPacket,
                           CRTProtocol* poRTProtocol);

  void updaterealtimevesseldata_first(realtimevessel_first& _realtimevessel,
                                      float _fX, float _fY, float _fZ,
                                      float _fAng1, float _fAng2, float _fAng3);
  void updaterealtimevesseldata_second(realtimevessel_second& _realtimevessel,
                                       float _fX, float _fY, float _fZ,
                                       float _fAng1, float _fAng2,
                                       float _fAng3);
  void updaterealtimevesseldata_third(realtimevessel_third& _realtimevessel,
                                      float _fX, float _fY, float _fZ,
                                      float _fAng1, float _fAng2, float _fAng3);

  void resetmeasurement(Vector6d& _measurement, Vector6d& _position);

  // calculate the real time coordinate transform matrix
  void calculateCoordinateTransform(Eigen::Matrix3d& _CTG2B,
                                    Eigen::Matrix3d& _CTB2G,
                                    double realtime_orientation,
                                    double desired_orientation);
  // How many cameras can be measured noise on
  static const int mcnMaxCameras = 20;
  // How many markers can be measured noise on
  static const int mcnMaxMarkers = 20;
  // How many samples to calculate max noise upon
  static const int mcnUseSamples = 100;
  static const int mcnMaxNoise = 30;  // Max noise in subpixels to display

  char msDist[100];
  float* mfDist;
  int mn2DFrames;

  bool mbWriteLogFileHeader;

  boost::posix_time::ptime time_start;
  double mfLastScreenUpdateTime;
  Marker masPrev2DMarkers[mcnMaxCameras][mcnMaxMarkers];

  int mnMaxPlotYPos;
  bool mbOutputModeScrolling;

  double mfCurrentRecvTime;
  double mfLastRecvTime;
  double mfRecvTimeDiff;
  double mfMaxRecvTimeDiff;
  double mfMinRecvTimeDiff;
  double mfRecFreq;
  double mfCameraFreq;
  unsigned long long mnLastTimeStamp;
  unsigned int mnLastFrameNumber;
  unsigned int mnMissingFrames;
  unsigned int mnReceivedFrames;
  int mnFrameNumberDiff;
  unsigned int mnMaxFrameNumberDiff;

  double qtm_frames_elapsed_time;
  motiondataprocess motiondataprocess_first;
  motiondataprocess motiondataprocess_second;
  motiondataprocess motiondataprocess_third;
};

#endif  // OUTPUT_H