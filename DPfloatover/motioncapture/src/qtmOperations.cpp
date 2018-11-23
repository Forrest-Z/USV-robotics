#include "../include/qtmOperations.h"

COperations::COperations(CInput* poInput, COutput* poOutput,
                         CRTProtocol* poRTProtocol)
    : qtm_frames_elapsed(motion_sample_time) {
  mpoInput = poInput;
  mpoOutput = poOutput;
  mpoRTProtocol = poRTProtocol;
}

void COperations::DataTransfer(realtimevessel_first& _realtimevessel_first,
                               realtimevessel_second& _realtimevessel_second,
                               realtimevessel_third& _realtimevessel_third,
                               FILE* _file) {
  CRTPacket::EPacketType ePacketType;
  unsigned int nComponentType;
  CRTProtocol::EStreamRate eStreamRate;
  int nRateArgument;

  mpoOutput->ResetCounters();

  // nComponentType = CRTProtocol::cComponent2d;
  nComponentType = CRTProtocol::cComponent6dEuler;
  eStreamRate = CRTProtocol::RateAllFrames;
  nRateArgument = 0;

  bool bDataAvailable;

  if ((nComponentType & CRTProtocol::cComponent6d) |
      (nComponentType & CRTProtocol::cComponent6dRes) |
      (nComponentType & CRTProtocol::cComponent6dEuler) |
      (nComponentType & CRTProtocol::cComponent6dEulerRes)) {
    mpoRTProtocol->Read6DOFSettings(bDataAvailable);
  }

  mpoRTProtocol->StreamFrames(eStreamRate, nRateArgument, nComponentType);

  // initialize timer to calculate real time frequency
  boost::posix_time::ptime t_start =
      boost::posix_time::second_clock::local_time();
  boost::posix_time::ptime t_end =
      boost::posix_time::second_clock::local_time();
  boost::posix_time::time_duration t_elapsed = t_end - t_start;
  long int mt_elapsed = 0;
  // Main data read loop
  while (1) {
    if (mpoRTProtocol->ReceiveRTPacket(ePacketType, true) > 0) {
      switch (ePacketType) {
        case CRTPacket::PacketError:  // sHeader.nType 0 indicates an error
        {
          CRTPacket* poRTPacket = mpoRTProtocol->GetRTPacket();
          _file = fopen(logsavepath.c_str(), "a+");
          fprintf(_file, "Error at StreamFrames: %s\n",
                  poRTPacket->GetErrorString());
          fclose(_file);
          break;
        }
        case CRTPacket::PacketData:  // Data received
        {
          mpoOutput->setframes_elapsed_time(qtm_frames_elapsed);
          mpoOutput->HandleDataFrame(
              _file, mpoRTProtocol, _realtimevessel_first,
              _realtimevessel_second, _realtimevessel_third);
          break;
        }
        case CRTPacket::PacketNoMoreData:  // No more data
          break;
        default:
          break;
      }
    }
    t_end = boost::posix_time::second_clock::local_time();
    t_elapsed = t_end - t_start;
    mt_elapsed = t_elapsed.total_milliseconds();
    if (mt_elapsed <= 0)
      qtm_frames_elapsed = motion_sample_time;
    else
      qtm_frames_elapsed = (double)mt_elapsed / 1000;  // seconds
    t_start = t_end;
    printf("%f\n", qtm_frames_elapsed);
  }
  mpoRTProtocol->StreamFramesStop();

}  // DataTransfer
