#ifndef DIALOG6DOF_H
#define DIALOG6DOF_H

#include <QDialog>
#include <QTimer>
#include <cmath>
#include <unordered_map>
#include "constants.h"
#include "globalvar.h"
#include "qcustomplot.h"  // the header file of QCustomPlot.
#include "realtimedata.h"

const uint32_t arraylength_6DoF = 296;  // length of array for 6DoF display
const int size_6dofdisplay = 990;

namespace Ui {
class Dialog6dof;
}

class Dialog6dof : public QDialog {
  Q_OBJECT

 public:
  explicit Dialog6dof(QWidget *parent = 0);
  ~Dialog6dof();

  void setupVesselRealtimeData();
 private slots:
  void motion6DOFdataSlot();

 private:
  Ui::Dialog6dof *ui;
  QTimer dataTimer;
  // data for 6DoF time series
  std::unordered_map<int, QVector<double>> motion_clients;
  QVector<double> motion6Dof_xaxis_data;
  QVector<double> motion6Dof_yaxis_data;
  std::vector<QCPAxisRect *> dofmotionplot;
  void initializeAllUI();
  void initialize6DOFmotion(QCustomPlot *customPlot);
};

#endif  // DIALOG6DOF_H
