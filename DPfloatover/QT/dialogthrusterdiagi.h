#ifndef DIALOGTHRUSTERDIAGI_H
#define DIALOGTHRUSTERDIAGI_H

#include <QDialog>
#include "constants.h"
#include "globalvar.h"
#include "qcustomplot.h"  // the header file of QCustomPlot.
#include "realtimedata.h"

// const int arraylength_setpoint_circle = 4;
const double azimuth_radius = 0.5;
const int arraylength_tunnel = 13;
const double tunnel_angle = M_PI / 6;

namespace Ui {
class DialogThrusterDiagI;
}

class DialogThrusterDiagI : public QDialog {
  Q_OBJECT

 public:
  explicit DialogThrusterDiagI(QWidget *parent = 0);
  ~DialogThrusterDiagI();

 private:
  Ui::DialogThrusterDiagI *ui;
  double qcustomplot_max = 5;

  QVector<QCPCurveData> dataSpiral_tunnelcolor;
  QVector<QCPCurveData> dataSpiral_tunnelnocolor;
  QCPCurve *fermatSpiral_tunnelcolor;
  QCPCurve *fermatSpiral_tunnelnocolor;
  void initializeAllUI(QCustomPlot *customPlot);
  void initializevesselshape(QCustomPlot *customPlot, double translationx,
                             double translationy);
  void initializeSTBDProgressBar(QCustomPlot *customPlot);
  void updateProgressBar(double _center_x, double _center_y, double _percent,
                         double _orientation);
};

#endif  // DIALOGTHRUSTERDIAGI_H
