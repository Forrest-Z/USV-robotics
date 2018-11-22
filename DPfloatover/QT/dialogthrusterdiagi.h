#ifndef DIALOGTHRUSTERDIAGI_H
#define DIALOGTHRUSTERDIAGI_H

#include <QDialog>
#include <QTimer>
#include "constants.h"
#include "globalvar.h"
#include "qcustomplot.h"  // the header file of QCustomPlot.
#include "realtimedata.h"

// const int arraylength_setpoint_circle = 4;
const double azimuth_radius = 0.4;
const double ball_radius = 0.03;
const int arraylength_azimuth = 13;
const double tunnel_angle = M_PI / 6;

namespace Ui {
class DialogThrusterDiagI;
}

class DialogThrusterDiagI : public QDialog {
  Q_OBJECT

 public:
  explicit DialogThrusterDiagI(QWidget *parent = 0);
  ~DialogThrusterDiagI();

 private slots:
  void AzimuthDataSlot();

 private:
  QTimer dataTimer;
  Ui::DialogThrusterDiagI *ui;
  double qcustomplot_max = 5;

  // viewer for starboard azimuth thruster
  std::vector<double> STBD_position;
  QVector<QCPCurveData> dataSpiral_tunnelcolor_STBD;
  QVector<QCPCurveData> dataSpiral_tunnelnocolor_STBD;
  QVector<QCPCurveData> dataSpiral_ball_STBD;
  QCPCurve *fermatSpiral_tunnelcolor_STBD;
  QCPCurve *fermatSpiral_tunnelnocolor_STBD;
  QCPCurve *fermatSpiral_ball_STBD;
  QCPItemText *STBD_RPMText;
  QCPItemText *STBD_angleText;
  // viewer for port azimuth thruster
  std::vector<double> PORT_position;
  QVector<QCPCurveData> dataSpiral_tunnelcolor_PORT;
  QVector<QCPCurveData> dataSpiral_tunnelnocolor_PORT;
  QVector<QCPCurveData> dataSpiral_ball_PORT;
  QCPCurve *fermatSpiral_tunnelcolor_PORT;
  QCPCurve *fermatSpiral_tunnelnocolor_PORT;
  QCPCurve *fermatSpiral_ball_PORT;
  QCPItemText *PORT_RPMText;
  QCPItemText *PORT_angleText;
  void setupthrusterRealtimeData();
  void initializeAllUI(QCustomPlot *customPlot);
  void initializevesselshape(QCustomPlot *customPlot, double translationx,
                             double translationy);
  void initializeSTBDProgressBar(QCustomPlot *customPlot);

  void updateSTBDProgressBar(double _percent, double _orientation);
  void updateSTBDText(int _rpm, int _orientation);
  void initializePORTProgressBar(QCustomPlot *customPlot);
  void updatePORTProgressBar(double _percent, double _orientation);
  void updatePORTText(int _rpm, int _orientation);
};

#endif  // DIALOGTHRUSTERDIAGI_H
