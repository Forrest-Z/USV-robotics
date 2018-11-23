#ifndef QTMOPERATIONS_H
#define QTMOPERATIONS_H

#include "RTProtocol.h"
#include "qtmInput.h"
#include "qtmOutput.h"

class COperations {
 public:
  COperations(CInput* mpoInput, COutput* poOutput, CRTProtocol* poRTProtocol);
  void DataTransfer(realtimevessel_first& _realtimevessel_first,
                    realtimevessel_second& _realtimevessel_second,
                    realtimevessel_third& _realtimevessel_third, FILE* _file);

 private:
  double qtm_frames_elapsed;
  CInput* mpoInput;
  COutput* mpoOutput;
  CRTProtocol* mpoRTProtocol;
};

#endif
