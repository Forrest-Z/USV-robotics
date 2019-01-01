/*
***********************************************************************
* vesselshape.h:
* function for read the shape of vessel from csv file
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/
#ifndef _VESSELSHAPE_H_
#define _VESSELSHAPE_H_

#include <map>
#include "csvstream.h"

struct vesselshapedata {
  unsigned totalnum;
  std::vector<double> x;
  std::vector<double> y;
};

class vesselshape {
 public:
  explicit vesselshape(const std::string &_name = "test.csv")
      : filename(_name) {}
  ~vesselshape() {}

  void readvesselshape(vesselshapedata &_vesselshapedata) {
    // Open file
    csvstream csvin(filename);

    // Rows have key = column name, value = cell datum
    std::map<std::string, std::string> row;

    _vesselshapedata.totalnum = 0;
    _vesselshapedata.x.clear();
    _vesselshapedata.y.clear();
    // Read file
    while (csvin >> row) {
      _vesselshapedata.totalnum++;
      for (auto &col : row) {
        const std::string &column_name = col.first;
        const std::string &datum = col.second;
        double value = stod(datum);
        if (column_name == "X") _vesselshapedata.x.push_back(value);
        if (column_name == "Y") _vesselshapedata.y.push_back(value);
      }
    }
  }

  void setfilename(const std::string &_name) { filename = _name; }

 private:
  std::string filename;
};
#endif  //_VESSELSHAPE_H_