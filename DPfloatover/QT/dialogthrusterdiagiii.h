#ifndef DIALOGTHRUSTERDIAGIII_H
#define DIALOGTHRUSTERDIAGIII_H

#include <QDialog>
#include <QTimer>
#include "constants.h"
#include "globalvar.h"
#include "qcustomplot.h"  // the header file of QCustomPlot.
#include "realtimedata.h"

const double azimuth_radius_third = 0.4;
const double ball_radius_third = 0.03;
const int arraylength_azimuth_third = 13;
const double tunnel_angle_third = M_PI / 6;

const double tunnel_width_third = 0.2;
const double tunnel_length_third = 0.8;

const int max_tunnel_rotation_third = 3000;
const int max_azimuth_rotation_third = 3000;

namespace Ui {
class DialogthrusterdiagIII;
}

class DialogthrusterdiagIII : public QDialog {
  Q_OBJECT

 public:
  explicit DialogthrusterdiagIII(QWidget *parent = 0);
  ~DialogthrusterdiagIII();

 private slots:
  void AzimuthDataSlot();

 private:
  QTimer dataTimer;
  Ui::DialogthrusterdiagIII *ui;
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

  // viewer for tunnel thruster 1
  std::vector<double> tunnel_first_position;
  QVector<QCPCurveData> dataSpiral_tunnel_first;
  QCPCurve *fermatSpiral_tunnel_first;
  QCPItemText *tunnel_first_RPMText;

  // viewer for tunnel thruster 2
  std::vector<double> tunnel_second_position;
  QVector<QCPCurveData> dataSpiral_tunnel_second;
  QCPCurve *fermatSpiral_tunnel_second;
  QCPItemText *tunnel_second_RPMText;

  void setupthrusterRealtimeData();
  void initializeAllUI(QCustomPlot *customPlot);
  void initializevesselshape(QCustomPlot *customPlot, double translationx,
                             double translationy);

  // starboard azimuth thruster
  void initializeSTBDProgressBar(QCustomPlot *customPlot);
  void updateSTBDProgressBar(double _percent, double _orientation);
  void updateSTBDText(int _rpm, int _orientation);
  // port azimuth thruster
  void initializePORTProgressBar(QCustomPlot *customPlot);
  void updatePORTProgressBar(double _percent, double _orientation);
  void updatePORTText(int _rpm, int _orientation);

  // tunnel 1
  void initializeTunnelProgressBar_first(QCustomPlot *customPlot);
  void updateTunnelProgressBar_first(int _rpm);
  // tunnel 2
  void initializeTunnelProgressBar_second(QCustomPlot *customPlot);
  void updateTunnelProgressBar_second(int _rpm);
};

#endif  // DIALOGTHRUSTERDIAGIII_H
