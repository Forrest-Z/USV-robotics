/*
***********************************************************************
* motioncapture.hpp:
* real-time SDK for QTM, which could enable real-time motion capture.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _MOTIONCAPTURE_H
#define _MOTIONCAPTURE_H

#include <math.h>
#include <stdio.h>
#include <Eigen/Core>
#include <chrono>
#include <iostream>
#include <thread>
#include "constants.h"
#include "qtmOperations.h"
#include "realtimedata.h"

#define LATEST_SELECTABLE_MAJOR_VERSION 1
#define LATEST_SELECTABLE_MINOR_VERSION 14
typedef Eigen::Matrix<double, 6, MAXCONNECTION> Mat4Allclient_6DoF;

class motioncapture {
 public:
  explicit motioncapture(int index_default = 1)
      : acquisitionindex(index_default) {}
  ~motioncapture() {
    delete poOperations;
    delete poRTProtocol;
    delete poOutput;
    delete poInput;
  }

  int initializemotioncapture(FILE *_file) {
    // connect to QTM server
    poInput = new CInput();

    char pServerAddr[32] = "192.168.253.1";
    unsigned short nBasePort = 22222;

    poOutput = new COutput();
    poRTProtocol = new CRTProtocol();
    poOperations = new COperations(poInput, poOutput, poRTProtocol);
    if (poRTProtocol->Connect(pServerAddr, nBasePort, 1, 14, false)) {
      char pVer[64];
      if (poRTProtocol->GetQTMVersion(pVer, sizeof(pVer))) {
        _file = fopen(logsavepath.c_str(), "a+");
        fprintf(_file, "Connected.%s.\n", pVer);
        fclose(_file);
        acquisitionindex = 0;
      }

    } else {
      _file = fopen(logsavepath.c_str(), "a+");
      fprintf(_file, "Failed to connect to QTM RT Server. %s\n",
              poRTProtocol->GetErrorString());
      delete poOperations;
      delete poRTProtocol;
      delete poOutput;
      delete poInput;
      fclose(_file);
      acquisitionindex = 1;
    }
    return acquisitionindex;
  }
  // request and update the motion data, this function could be called by
  // multi-thread
  void RequestPosition(realtimevessel_first &_realtimevessel_first,
                       realtimevessel_second &_realtimevessel_second,
                       realtimevessel_third &_realtimevessel_third,
                       FILE *_file) {
    poOperations->DataTransfer(_realtimevessel_first, _realtimevessel_second,
                               _realtimevessel_third, _file);
  }
  // suspend the QTM clients
  void stopRequest(FILE *_file) {
    poRTProtocol->StreamFramesStop();
    _file = fopen(logsavepath.c_str(), "a+");
    fprintf(_file, "Shut down QTM client.\n");
    fclose(_file);
  }

  int getmcapstatus() const { return acquisitionindex; }

 private:
  // raw data

  // data acquisition successful or not
  int acquisitionindex;

  // QTM RT protocal

  CInput *poInput = NULL;
  COutput *poOutput = NULL;
  CRTProtocol *poRTProtocol = NULL;
  COperations *poOperations = NULL;
};

#endif /*_MOTIONCAPTURE_H*/
