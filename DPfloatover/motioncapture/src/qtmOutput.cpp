#include "../include/qtmOutput.h"

#define WRITE_ANALOG_HEADERS_TO_FILE

motiondataprocess::motiondataprocess() : index_step(0) { initializedata(); }
motiondataprocess::~motiondataprocess() {}
void motiondataprocess::initializedata() {
  formeraverageposition.setZero();
  average_yaw.setZero();
  average_surge.setZero();
  average_sway.setZero();
  average_yaw_velocity.setZero();
  average_surge_velocity.setZero();
  average_sway_velocity.setZero();
}

Eigen::Vector3d motiondataprocess::movingaverageposition(
    double _m_fx, double _m_fy, double _rad_orientation) {
  Eigen::Vector3d position_vector = Eigen::Vector3d::Zero();
  position_vector << movingaverage_surge(_m_fx), movingaverage_sway(_m_fy),
      movingaverage_yaw(_rad_orientation);
  return position_vector;
}

Eigen::Vector3d motiondataprocess::movingaveragevelocity(
    const Eigen::Vector3d& _average_position_vector) {
  Eigen::Vector3d velocity_global = cal_velocity(_average_position_vector);
  Eigen::Vector3d average_velocity_global = Eigen::Vector3d::Zero();
  average_velocity_global << movingaverage_surge_velocity(velocity_global(0)),
      movingaverage_sway_velocity(velocity_global(1)),
      movingaverage_yaw_velocity(velocity_global(2));
  return average_velocity_global;
}

// moving average lowpass to remove noise in yaw
double motiondataprocess::movingaverage_yaw(double _dtheta) {
  // pop_front
  VectorAYaw t_average_yaw = VectorAYaw::Zero();
  int index = num_average_point_yaw - 1;
  t_average_yaw.head(index) = average_yaw.tail(index);
  // push back
  t_average_yaw(index) = _dtheta;
  average_yaw = t_average_yaw;
  // calculate the mean value
  return average_yaw.mean();
}

// moving average lowpass to remove noise in surge
double motiondataprocess::movingaverage_surge(double _dx) {
  // pop_front
  VectorASurge t_average_surge = VectorASurge::Zero();
  int index = num_average_point_surge - 1;
  t_average_surge.head(index) = average_surge.tail(index);
  // push back
  t_average_surge(index) = _dx;
  average_surge = t_average_surge;
  // calculate the mean value
  return average_surge.mean();
}
// moving average lowpass to remove noise in sway
double motiondataprocess::movingaverage_sway(double _dy) {
  // pop_front
  VectorASway t_average_sway = VectorASway::Zero();
  int index = num_average_point_sway - 1;
  t_average_sway.head(index) = average_sway.tail(index);
  // push back
  t_average_sway(index) = _dy;
  average_sway = t_average_sway;
  // calculate the mean value
  return average_sway.mean();
}

// moving average lowpass to remove noise in yaw velocity
double motiondataprocess::movingaverage_yaw_velocity(double _vtheta) {
  // pop_front
  VectorAYawV t_average_yawV = VectorAYawV::Zero();
  int index = num_average_yaw_velocity - 1;
  t_average_yawV.head(index) = average_yaw_velocity.tail(index);
  // push back
  t_average_yawV(index) = _vtheta;
  average_yaw_velocity = t_average_yawV;
  // calculate the mean value
  return average_yaw_velocity.mean();
}
// moving average lowpass to remove noise in surge velocity
double motiondataprocess::movingaverage_surge_velocity(double _vx) {
  // pop_front
  VectorASurgeV t_average_surgeV = VectorASurgeV::Zero();
  int index = num_average_surge_velocity - 1;
  t_average_surgeV.head(index) = average_surge_velocity.tail(index);
  // push back
  t_average_surgeV(index) = _vx;
  average_surge_velocity = t_average_surgeV;
  // calculate the mean value
  return average_surge_velocity.mean();
}
// moving average lowpass to remove noise in sway velocity
double motiondataprocess::movingaverage_sway_velocity(double _vy) {
  // pop_front
  VectorASwayV t_average_swayV = VectorASwayV::Zero();
  int index = num_average_sway_velocity - 1;
  t_average_swayV.head(index) = average_sway_velocity.tail(index);
  // push back
  t_average_swayV(index) = _vy;
  average_sway_velocity = t_average_swayV;
  // calculate the mean value
  return average_sway_velocity.mean();
}

Eigen::Vector3d motiondataprocess::cal_velocity(
    const Eigen::Vector3d& _average_position_vector) {
  Eigen::Vector3d velocity =
      (_average_position_vector - formeraverageposition) / motion_sample_time;
  formeraverageposition = _average_position_vector;
  return velocity;
}

COutput::COutput()
    : mfDist(NULL),
      mbWriteLogFileHeader(true),
      time_start(T_BOOST_CLOCK::local_time()),
      mbOutputModeScrolling(false) {}

void COutput::HandleDataFrame(FILE* logfile, bool bLogMinimum,
                              CRTProtocol* poRTProtocol) {
  CRTPacket* poRTPacket;

  mnReceivedFrames++;

  if (poRTProtocol) {
    poRTPacket = poRTProtocol->GetRTPacket();
    if (poRTPacket->GetComponentCount() == 0 ||
        poRTPacket->GetType() != CRTPacket::PacketData) {
      return;
    }
  } else {
    return;
  }

  unsigned int nFrameNumber = poRTPacket->GetFrameNumber();
  unsigned long long nTimeStamp = poRTPacket->GetTimeStamp();

  if (nFrameNumber == 1 && mnLastFrameNumber != 0) {
    // Start from the beginning in case we are running rt from file
    ResetCounters();
    mnReceivedFrames = 1;
  }

  // Update packet receive time.
  boost::posix_time::ptime time_now(T_BOOST_CLOCK::local_time());
  boost::posix_time::time_duration t_d = time_now - time_start;

  mfCurrentRecvTime = (double)(t_d.total_milliseconds() / 1000.0);

  if (mnReceivedFrames > 1) {
    mnFrameNumberDiff = nFrameNumber - mnLastFrameNumber;

    if (mnFrameNumberDiff <= 0) {
      // Frame repeated (should never happen).
      ResetCounters();
      mnReceivedFrames = 1;
    } else {
      // New frame received.
      mfCameraFreq =
          (mnFrameNumberDiff * 1000000.0) / (nTimeStamp - mnLastTimeStamp);
      mnMaxFrameNumberDiff =
          std::max((unsigned int)mnFrameNumberDiff, mnMaxFrameNumberDiff);
      mfRecvTimeDiff = mfCurrentRecvTime - mfLastRecvTime;
      mfMaxRecvTimeDiff = std::max(mfRecvTimeDiff, mfMaxRecvTimeDiff);
      mfMinRecvTimeDiff = std::min(mfRecvTimeDiff, mfMinRecvTimeDiff);
      mfRecFreq = 1.0 / mfRecvTimeDiff;

      if (mnFrameNumberDiff > 1) {
        mnMissingFrames += mnFrameNumberDiff - 1;
        printf("Missing %d frame%s. Frame number: %d", mnFrameNumberDiff,
               (mnFrameNumberDiff == 1) ? "" : "s", mnLastFrameNumber + 1);
        printf(" to %d\n\n", nFrameNumber - 1);
      }
    }
  }
  mnLastTimeStamp = nTimeStamp;
  mnLastFrameNumber = nFrameNumber;
  mfLastRecvTime = mfCurrentRecvTime;

  if (mfLastScreenUpdateTime == 0.0 ||
      (mfCurrentRecvTime - mfLastScreenUpdateTime) > 0.1) {
    mfLastScreenUpdateTime = mfCurrentRecvTime;
  } else {
    return;
  }

  PrintData6D(logfile, poRTPacket, poRTProtocol);
  PrintData6DRes(logfile, poRTPacket, poRTProtocol);
  PrintData6DEuler(logfile, poRTPacket, poRTProtocol);
  PrintData6DEulerRes(logfile, poRTPacket, poRTProtocol);

}  // PrintData

void COutput::HandleDataFrame(FILE* logfile, CRTProtocol* poRTProtocol,
                              realtimevessel_first& _realtimevessel_first,
                              realtimevessel_second& _realtimevessel_second,
                              realtimevessel_third& _realtimevessel_third) {
  CRTPacket* poRTPacket;

  if (poRTProtocol) {
    poRTPacket = poRTProtocol->GetRTPacket();
    if (poRTPacket->GetComponentCount() == 0 ||
        poRTPacket->GetType() != CRTPacket::PacketData) {
      return;
    }
  } else {
    return;
  }

  if (MAXCONNECTION == 1)
    PrintData6DEuler(poRTPacket, poRTProtocol, _realtimevessel_first);
  else if (MAXCONNECTION == 2)
    PrintData6DEuler(poRTPacket, poRTProtocol, _realtimevessel_first,
                     _realtimevessel_second);
  else if (MAXCONNECTION == 3)
    PrintData6DEuler(poRTPacket, poRTProtocol, _realtimevessel_first,
                     _realtimevessel_second, _realtimevessel_third);
}  // OutputData

void COutput::PrintData6D(FILE* logfile, CRTPacket* poRTPacket,
                          CRTProtocol* poRTProtocol) {
  float fX, fY, fZ;
  float afRotMatrix[9];

  if (poRTPacket->GetComponentSize(CRTPacket::Component6d)) {
    unsigned int nCount = poRTPacket->Get6DOFBodyCount();

    if (nCount > 0) {
      fprintf(logfile, "------------------ 6 DOF -----------------\n");
      for (unsigned int i = 0; i < nCount; i++) {
        char* label = (char*)poRTProtocol->Get6DOFBodyName(i);
        char emptyString[] = "";
        if (label == NULL) {
          label = emptyString;
        }
        poRTPacket->Get6DOFBody(i, fX, fY, fZ, afRotMatrix);

        fprintf(
            logfile,
            "%15s : %f\t%f\t%f\t -    %f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t\n",
            label, fX, fY, fZ, afRotMatrix[0], afRotMatrix[1], afRotMatrix[2],
            afRotMatrix[3], afRotMatrix[4], afRotMatrix[5], afRotMatrix[6],
            afRotMatrix[7], afRotMatrix[8]);
      }
      fprintf(logfile, "\n");
    }
  }
}  // PrintData6D

void COutput::PrintData6DRes(FILE* logfile, CRTPacket* poRTPacket,
                             CRTProtocol* poRTProtocol) {
  float fX, fY, fZ, fResidual;
  float afRotMatrix[9];

  if (poRTPacket->GetComponentSize(CRTPacket::Component6dRes)) {
    unsigned int nCount = poRTPacket->Get6DOFResidualBodyCount();

    if (nCount > 0) {
      fprintf(logfile, "------------- 6 DOF Residual -------------\n");
      for (unsigned int i = 0; i < nCount; i++) {
        char* label = (char*)poRTProtocol->Get6DOFBodyName(i);
        char emptyString[] = "";
        if (label == NULL) {
          label = emptyString;
        }
        poRTPacket->Get6DOFResidualBody(i, fX, fY, fZ, afRotMatrix, fResidual);

        fprintf(logfile,
                "%15s : %f\t%f\t%f\t -    %f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t "
                "-   %f\n",
                label, fX, fY, fZ, afRotMatrix[0], afRotMatrix[1],
                afRotMatrix[2], afRotMatrix[3], afRotMatrix[4], afRotMatrix[5],
                afRotMatrix[6], afRotMatrix[7], afRotMatrix[8], fResidual);
      }
      fprintf(logfile, "\n");
    }
  }
}  // PrintData6DRes

void COutput::PrintData6DEuler(FILE* logfile, CRTPacket* poRTPacket,
                               CRTProtocol* poRTProtocol) {
  float fX, fY, fZ, fAng1, fAng2, fAng3;

  if (poRTPacket->GetComponentSize(CRTPacket::Component6dEuler)) {
    unsigned int nCount = poRTPacket->Get6DOFEulerBodyCount();

    if (nCount > 0) {
      fprintf(logfile, "--------------- 6 DOF Euler --------------\n");
      for (unsigned int i = 0; i < nCount; i++) {
        char* label = (char*)poRTProtocol->Get6DOFBodyName(i);
        char emptyString[] = "";
        if (label == NULL) {
          label = emptyString;
        }
        poRTPacket->Get6DOFEulerBody(i, fX, fY, fZ, fAng1, fAng2, fAng3);

        fprintf(logfile, "%15s : %f\t%f\t%f\t -   %f\t%f\t%f\n", label, fX, fY,
                fZ, fAng1, fAng2, fAng3);
      }
      fprintf(logfile, "\n");
    }
  }
}  // PrintData6DEuler

void COutput::PrintData6DEuler(CRTPacket* poRTPacket, CRTProtocol* poRTProtocol,
                               realtimevessel_first& _realtimevessel_first) {
  float fX, fY, fZ, fAng1, fAng2, fAng3;  // mm, mm, mm, deg, deg, deg (QTM)

  if (poRTPacket->GetComponentSize(CRTPacket::Component6dEuler)) {
    unsigned int nCount = poRTPacket->Get6DOFEulerBodyCount();
    // determine if we have rigid body
    if (nCount > 0) {
      poRTPacket->Get6DOFEulerBody(0, fX, fY, fZ, fAng1, fAng2, fAng3);
      // determine if the measured data is out of range or NaN
      updaterealtimevesseldata_first(_realtimevessel_first, fX, fY, fZ, fAng1,
                                     fAng2, fAng3);

    } else {
      resetmeasurement(_realtimevessel_first.Measurement,
                       _realtimevessel_first.Position);

      // logfile = fopen(logsavepath.c_str(), "a+");
      fprintf(stdout, "No rigid body found!\n");
      // fclose(logfile);
    }
  }
}  // PrintData6DEuler (the first vessel)

void COutput::PrintData6DEuler(CRTPacket* poRTPacket, CRTProtocol* poRTProtocol,
                               realtimevessel_first& _realtimevessel_first,
                               realtimevessel_second& _realtimevessel_second) {
  if (poRTPacket->GetComponentSize(CRTPacket::Component6dEuler)) {
    unsigned int nCount = poRTPacket->Get6DOFEulerBodyCount();

    if (nCount > 0) {
      float fX_first, fY_first, fZ_first, fAng1_first, fAng2_first,
          fAng3_first;  // mm, mm, mm, deg, deg, deg (QTM)
      float fX_second, fY_second, fZ_second, fAng1_second, fAng2_second,
          fAng3_second;  // mm, mm, mm, deg, deg, deg (QTM)
      // the first vessel
      poRTPacket->Get6DOFEulerBody(0, fX_first, fY_first, fZ_first, fAng1_first,
                                   fAng2_first, fAng3_first);
      // determine if the measured data is out of range or NaN
      updaterealtimevesseldata_first(_realtimevessel_first, fX_first, fY_first,
                                     fZ_first, fAng1_first, fAng2_first,
                                     fAng3_first);

      // the second vessel
      poRTPacket->Get6DOFEulerBody(1, fX_second, fY_second, fZ_second,
                                   fAng1_second, fAng2_second, fAng3_second);
      // determine if the measured data is out of range or NaN
      updaterealtimevesseldata_second(_realtimevessel_second, fX_second,
                                      fY_second, fZ_second, fAng1_second,
                                      fAng2_second, fAng3_second);

    } else {
      resetmeasurement(_realtimevessel_first.Measurement,
                       _realtimevessel_first.Position);
      resetmeasurement(_realtimevessel_second.Measurement,
                       _realtimevessel_second.Position);

      // logfile = fopen(logsavepath.c_str(), "a+");
      fprintf(stdout, "No rigid body found!\n");
      // fclose(logfile);
    }
  }
}  // PrintData6DEuler (the first and second vessel)

void COutput::PrintData6DEuler(CRTPacket* poRTPacket, CRTProtocol* poRTProtocol,
                               realtimevessel_first& _realtimevessel_first,
                               realtimevessel_second& _realtimevessel_second,
                               realtimevessel_third& _realtimevessel_third) {
  float fX, fY, fZ, fAng1, fAng2, fAng3;  // mm, mm, mm, deg, deg, deg (QTM)
  double m_fx, m_fy, rad_orientation, raw_u, raw_v, raw_r;
  if (poRTPacket->GetComponentSize(CRTPacket::Component6dEuler)) {
    unsigned int nCount = poRTPacket->Get6DOFEulerBodyCount();

    if (nCount > 0) {
      // the first vessel
      poRTPacket->Get6DOFEulerBody(0, fX, fY, fZ, fAng1, fAng2, fAng3);
      // determine if the measured data is out of range or NaN
      updaterealtimevesseldata_first(_realtimevessel_first, fX, fY, fZ, fAng1,
                                     fAng2, fAng3);

      // the second vessel
      poRTPacket->Get6DOFEulerBody(1, fX, fY, fZ, fAng1, fAng2, fAng3);
      // determine if the measured data is out of range or NaN
      updaterealtimevesseldata_second(_realtimevessel_second, fX, fY, fZ, fAng1,
                                      fAng2, fAng3);

      // the third vessel
      poRTPacket->Get6DOFEulerBody(2, fX, fY, fZ, fAng1, fAng2, fAng3);
      // determine if the measured data is out of range or NaN
      updaterealtimevesseldata_third(_realtimevessel_third, fX, fY, fZ, fAng1,
                                     fAng2, fAng3);
    } else {
      resetmeasurement(_realtimevessel_first.Measurement,
                       _realtimevessel_first.Position);
      resetmeasurement(_realtimevessel_second.Measurement,
                       _realtimevessel_second.Position);
      resetmeasurement(_realtimevessel_third.Measurement,
                       _realtimevessel_third.Position);

      // logfile = fopen(logsavepath.c_str(), "a+");
      fprintf(stdout, "No rigid body found!\n");
      // fclose(logfile);
    }
  }
}  // PrintData6DEuler (the first/second/third vessel)

void COutput::PrintData6DEulerRes(FILE* logfile, CRTPacket* poRTPacket,
                                  CRTProtocol* poRTProtocol) {
  float fX, fY, fZ, fAng1, fAng2, fAng3, fResidual;

  if (poRTPacket->GetComponentSize(CRTPacket::Component6dEulerRes)) {
    unsigned int nCount = poRTPacket->Get6DOFEulerResidualBodyCount();

    if (nCount > 0) {
      fprintf(logfile, "---------- 6 DOF Euler Residual ----------\n");
      for (unsigned int i = 0; i < nCount; i++) {
        char* label = (char*)poRTProtocol->Get6DOFBodyName(i);
        char emptyString[] = "";
        if (label == NULL) {
          label = emptyString;
        }
        poRTPacket->Get6DOFEulerResidualBody(i, fX, fY, fZ, fAng1, fAng2, fAng3,
                                             fResidual);

        fprintf(logfile, "%15s : %f\t%f\t%f\t -   %f\t%f\t%f\t -    %f\n",
                label, fX, fY, fZ, fAng1, fAng2, fAng3, fResidual);
      }
      fprintf(logfile, "\n");
    }
  }
}  // PrintData6DEulerRes

void COutput::PrintTimingData() {
  printf("\n\nReceived %d data frames in %f seconds\n\n", mnReceivedFrames,
         mfCurrentRecvTime);
  printf("Average receive frequency = %.1f\n",
         mnReceivedFrames / mfCurrentRecvTime);
  printf("Camera frequency = %.1f Hz\n", mfCameraFreq);
  printf("Missed frames = %d\n", mnMissingFrames);
  printf("Max frame receive time diff = %f ms\n", mfMaxRecvTimeDiff);
  printf("Min frame receive time diff = %f ms\n\n", mfMinRecvTimeDiff);
}  // PrintTimingData

void COutput::ResetCounters() {
  // Reset statistic counters
  mfRecvTimeDiff = 0.0;
  mfMaxRecvTimeDiff = 0.0;
  mfMinRecvTimeDiff = 100000.0;
  mfLastRecvTime = 0.0;
  mfRecFreq = 0.0;
  mfLastScreenUpdateTime = 0.0;
  mnLastFrameNumber = 0xffffffff;
  mnMaxFrameNumberDiff = 0;
  mnMissingFrames = 0;
  mnReceivedFrames = 0;
  mnFrameNumberDiff = 0;
  mnMaxFrameNumberDiff = 0;
  mnLastTimeStamp = 0;

}  // ResetCounters

void COutput::updaterealtimevesseldata_first(
    realtimevessel_first& _realtimevessel, float _fX, float _fY, float _fZ,
    float _fAng1, float _fAng2, float _fAng3) {
  // determine if the measured data is out of range or NaN
  if ((abs(_fX) < qtm_max_position) && (abs(_fY) < qtm_max_position)) {
    double m_fx = _fX / 1000;
    double m_fy = _fY / 1000;
    double rad_orientation = _fAng3 * M_PI / 180;
    _realtimevessel.Position(0) = m_fx;
    _realtimevessel.Position(1) = m_fy;
    _realtimevessel.Position(2) = _fZ / 1000;
    _realtimevessel.Position(3) = _fAng1;
    _realtimevessel.Position(4) = _fAng2;
    _realtimevessel.Position(5) = _fAng3;

    Eigen::Vector3d average_position_vector =
        motiondataprocess_first.movingaverageposition(m_fx, m_fy,
                                                      rad_orientation);

    _realtimevessel.Measurement.head(3) = average_position_vector;

    calculateCoordinateTransform(_realtimevessel.CTG2B, _realtimevessel.CTB2G,
                                 average_position_vector(2),
                                 _realtimevessel.setPoints(2));

    _realtimevessel.Measurement.tail(3) =
        _realtimevessel.CTG2B *
        motiondataprocess_first.movingaveragevelocity(average_position_vector);
  }
}

void COutput::updaterealtimevesseldata_second(
    realtimevessel_second& _realtimevessel, float _fX, float _fY, float _fZ,
    float _fAng1, float _fAng2,
    float _fAng3) {  // determine if the measured data is out of range or NaN
  if ((abs(_fX) < qtm_max_position) && (abs(_fY) < qtm_max_position)) {
    double m_fx = _fX / 1000;
    double m_fy = _fY / 1000;
    double rad_orientation = _fAng3 * M_PI / 180;
    _realtimevessel.Position(0) = m_fx;
    _realtimevessel.Position(1) = m_fy;
    _realtimevessel.Position(2) = _fZ / 1000;
    _realtimevessel.Position(3) = _fAng1;
    _realtimevessel.Position(4) = _fAng2;
    _realtimevessel.Position(5) = _fAng3;

    Eigen::Vector3d average_position_vector =
        motiondataprocess_second.movingaverageposition(m_fx, m_fy,
                                                       rad_orientation);

    _realtimevessel.Measurement.head(3) = average_position_vector;

    calculateCoordinateTransform(_realtimevessel.CTG2B, _realtimevessel.CTB2G,
                                 average_position_vector(2),
                                 _realtimevessel.setPoints(2));

    _realtimevessel.Measurement.tail(3) =
        _realtimevessel.CTG2B *
        motiondataprocess_second.movingaveragevelocity(average_position_vector);
  }
}

void COutput::updaterealtimevesseldata_third(
    realtimevessel_third& _realtimevessel, float _fX, float _fY, float _fZ,
    float _fAng1, float _fAng2,
    float _fAng3) {  // determine if the measured data is out of range or NaN
  if ((abs(_fX) < qtm_max_position) && (abs(_fY) < qtm_max_position)) {
    double m_fx = _fX / 1000;
    double m_fy = _fY / 1000;
    double rad_orientation = _fAng3 * M_PI / 180;
    _realtimevessel.Position(0) = m_fx;
    _realtimevessel.Position(1) = m_fy;
    _realtimevessel.Position(2) = _fZ / 1000;
    _realtimevessel.Position(3) = _fAng1;
    _realtimevessel.Position(4) = _fAng2;
    _realtimevessel.Position(5) = _fAng3;

    Eigen::Vector3d average_position_vector =
        motiondataprocess_third.movingaverageposition(m_fx, m_fy,
                                                      rad_orientation);

    _realtimevessel.Measurement.head(3) = average_position_vector;

    calculateCoordinateTransform(_realtimevessel.CTG2B, _realtimevessel.CTB2G,
                                 average_position_vector(2),
                                 _realtimevessel.setPoints(2));

    _realtimevessel.Measurement.tail(3) =
        _realtimevessel.CTG2B *
        motiondataprocess_third.movingaveragevelocity(average_position_vector);
  }
}

void COutput::resetmeasurement(Vector6d& _measurement, Vector6d& _position) {
  _measurement.setZero();
  _position.setZero();
}

// calculate the real time coordinate transform matrix
void COutput::calculateCoordinateTransform(Eigen::Matrix3d& _CTG2B,
                                           Eigen::Matrix3d& _CTB2G,
                                           double realtime_orientation,
                                           double desired_orientation) {
  double cvalue = 0.0;
  double svalue = 0.0;
  if (abs(realtime_orientation - desired_orientation) < M_PI / 18) {
    // use the fixed setpoint orientation to prevent measurement noise
    cvalue = std::cos(desired_orientation);
    svalue = std::sin(desired_orientation);
  } else {
    // if larger than 5 deg, we use the realtime orientation
    cvalue = std::cos(realtime_orientation);
    svalue = std::sin(realtime_orientation);
  }

  _CTG2B(0, 0) = cvalue;
  _CTG2B(1, 1) = cvalue;
  _CTG2B(0, 1) = svalue;
  _CTG2B(1, 0) = -svalue;
  _CTB2G(0, 0) = cvalue;
  _CTB2G(1, 1) = cvalue;
  _CTB2G(0, 1) = -svalue;
  _CTB2G(1, 0) = svalue;
}