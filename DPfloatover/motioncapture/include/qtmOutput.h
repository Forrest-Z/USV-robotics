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
const double max_velocity_orientation = 0.5;  // rad/s
const double qtm_max_position = 30000;        // mm
const int num_average_point_velocity = 200;   // delay 2 s when frequenc=50Hz
const int num_average_point_yaw = 50;         // delay 2 s when frequenc=50Hz
const int num_average_point_surge = 20;       // delay 2 s when frequenc=50Hz
const int num_average_point_sway = 20;        // delay 2 s when frequenc=50Hz
// average moving low pass to eliminate noise
using T_BOOST_CLOCK =
    boost::date_time::microsec_clock<boost::posix_time::ptime>;
typedef Eigen::Matrix<double, 3, num_average_point_velocity> Matrix3100d;
typedef Eigen::Matrix<double, num_average_point_yaw, 1> VectorAYaw;
typedef Eigen::Matrix<double, num_average_point_surge, 1> VectorASurge;
typedef Eigen::Matrix<double, num_average_point_sway, 1> VectorASway;

class CDataPacket;

class COutput {
 public:
  COutput();
  void HandleDataFrame(FILE* logfile, bool bLogMinimum,
                       CRTProtocol* poRTProtocol);
  void HandleDataFrame(FILE* logfile, CRTProtocol* poRTProtocol,
                       realtimevessel_first& _realtimevessel_first,
                       realtimevessel_second& _realtimevessel_second,
                       realtimevessel_third& _realtimevessel_third);
  void PrintTimingData();
  void ResetCounters();
  void Print6DOFSettings(CRTProtocol* poRTProtocol);

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
  void PrintData6DEuler(CRTPacket* poRTPacket, CRTProtocol* poRTProtocol,
                        realtimevessel_first& _realtimevessel_first);
  void PrintData6DEuler(CRTPacket* poRTPacket, CRTProtocol* poRTProtocol,
                        realtimevessel_first& _realtimevessel_first,
                        realtimevessel_second& _realtimevessel_second);
  void PrintData6DEuler(CRTPacket* poRTPacket, CRTProtocol* poRTProtocol,
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

  void initializemovingaverage();
  Eigen::Vector3d movingaverage_velocity_first(double _dx, double _dy,
                                               double _dtheta);
  double movingaverage_yaw_first(double _dtheta);
  double movingaverage_surge_first(double _dx);
  double movingaverage_sway_first(double _dy);

  Eigen::Vector3d movingaverage_velocity_second(double _dx, double _dy,
                                                double _dtheta);
  double movingaverage_yaw_second(double _dtheta);
  double movingaverage_surge_second(double _dx);
  double movingaverage_sway_second(double _dy);

  Eigen::Vector3d movingaverage_velocity_third(double _dx, double _dy,
                                               double _dtheta);
  double movingaverage_yaw_third(double _dtheta);
  double movingaverage_surge_third(double _dx);
  double movingaverage_sway_third(double _dy);
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

  // moving average for velocity calculation of I vessel
  Matrix3100d Matrix_average_first;
  Eigen::Vector3d average_vector_first;
  VectorAYaw average_yaw_first;
  VectorASurge average_surge_first;
  VectorASway average_sway_first;

  // moving average for velocity calculation of II vessel
  Matrix3100d Matrix_average_second;
  Eigen::Vector3d average_vector_second;
  VectorAYaw average_yaw_second;
  VectorASurge average_surge_second;
  VectorASway average_sway_second;

  // moving average for velocity calculation of III vessel
  Matrix3100d Matrix_average_third;
  Eigen::Vector3d average_vector_third;
  VectorAYaw average_yaw_third;
  VectorASurge average_surge_third;
  VectorASway average_sway_third;
};

#endif  // OUTPUT_H