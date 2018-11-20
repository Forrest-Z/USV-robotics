#include "dialogthrusterdiagi.h"
#include "ui_dialogthrusterdiagi.h"

DialogThrusterDiagI::DialogThrusterDiagI(QWidget *parent)
    : QDialog(parent),
      ui(new Ui::DialogThrusterDiagI),
      dataSpiral_tunnelcolor(arraylength_tunnel),
      dataSpiral_tunnelnocolor(arraylength_tunnel) {
  ui->setupUi(this);
  initializeAllUI(ui->customPlot_thruster);
}

DialogThrusterDiagI::~DialogThrusterDiagI() { delete ui; }

void DialogThrusterDiagI::initializeAllUI(QCustomPlot *customPlot) {
  // setup all colors used
  QPalette pal_background = palette();
  pal_background.setColor(QPalette::Background, QColor(68, 68, 68, 255));

  // set the whole dialog
  // this->setWindowState(Qt::WindowMaximized);
  this->setWindowFlags(Qt::Window);
  this->setAutoFillBackground(true);
  this->setPalette(pal_background);
  this->setStyleSheet("QLabel {color : white; }");  // set color for all Qlabels

  customPlot->setGeometry(10, 0, 1000, 1000);

  // setup background
  QBrush boxBrush(QColor(255, 255, 255, 255));
  customPlot->setBackground(boxBrush);  // setup color of background
  customPlot->setNotAntialiasedElements(QCP::aeAll);
  initializevesselshape(customPlot, 4.5, 3);
  initializeSTBDProgressBar(customPlot);
  // setup x,y axis
  customPlot->xAxis->setVisible(false);
  customPlot->yAxis->setVisible(false);
  customPlot->xAxis->setRange(-qcustomplot_max, qcustomplot_max);
  customPlot->yAxis->setRange(-qcustomplot_max, qcustomplot_max);
  customPlot->yAxis->setScaleRatio(customPlot->xAxis, 1.0);  // axis equal
}

void DialogThrusterDiagI::initializevesselshape(
    QCustomPlot *customPlot, double translationx /*= 0*/,
    double translationy /*= -4.7*/) {
  /* initialize the 2D vessel shape, whose graph index will be 0 */
  customPlot->addGraph();
  // convert position and orientation of vessel to global coordinate
  int arraylength = 592;
  QVector<double> vesselshape_x(arraylength, 0);
  QVector<double> vesselshape_y(arraylength, 0);

  double t_orient = 180 * M_PI / 180;
  double radius = 0.78;
  double c_value = radius * std::cos(t_orient);
  double s_value = radius * std::sin(t_orient);
  for (int i = 0; i != arraylength; ++i) {
    vesselshape_x[i] = c_value * globalvar::vesselshape_x[i] -
                       s_value * globalvar::vesselshape_y[i] + translationx;
    vesselshape_y[i] = s_value * globalvar::vesselshape_x[i] +
                       c_value * globalvar::vesselshape_y[i] + translationy;
  }

  customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
  QCPScatterStyle myQCPScatterStyle1(QCPScatterStyle::ssDisc,
                                     QColor(238, 156, 167, 255), 2);
  customPlot->graph(0)->setScatterStyle(myQCPScatterStyle1);
  customPlot->graph(0)->setData(vesselshape_x, vesselshape_y);
  customPlot->graph(0)->rescaleAxes(true);
}

//
void DialogThrusterDiagI::initializeSTBDProgressBar(QCustomPlot *customPlot) {
  // generate data for outer circle
  const int arraylength_outercircle = 50;
  QVector<QCPCurveData> dataSpiral_outercircle(arraylength_outercircle);

  for (unsigned i = 0; i != arraylength_outercircle; ++i) {
    double theta = 2 * i * M_PI / (arraylength_outercircle - 1);
    dataSpiral_outercircle[i] = QCPCurveData(
        i, azimuth_radius * std::cos(theta), azimuth_radius * std::sin(theta));
  }

  // create curve objects
  QCPCurve *fermatSpiral_outercircle =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_outercircle->data()->set(dataSpiral_outercircle, true);
  fermatSpiral_outercircle->setPen(QPen(QColor(68, 68, 68, 255), 2));
  fermatSpiral_outercircle->setBrush(QBrush(QColor(170, 170, 170)));

  // generate data for progress bar on the right
  int num_circle = arraylength_tunnel - 3;
  for (unsigned i = 0; i != num_circle; ++i) {
    double theta = i * tunnel_angle / (num_circle - 1) - tunnel_angle / 2;
    dataSpiral_tunnelcolor[i] = QCPCurveData(
        i, azimuth_radius * std::cos(theta), azimuth_radius * std::sin(theta));
    dataSpiral_tunnelnocolor[i] =
        QCPCurveData(i, azimuth_radius * std::cos(theta + M_PI),
                     azimuth_radius * std::sin(theta + M_PI));
  }
  double xvalue = -0.1;
  dataSpiral_tunnelcolor[num_circle] = QCPCurveData(
      num_circle, xvalue, azimuth_radius * std::sin(tunnel_angle / 2));
  dataSpiral_tunnelcolor[num_circle + 1] = QCPCurveData(
      num_circle + 1, xvalue, azimuth_radius * std::sin(-tunnel_angle / 2));
  dataSpiral_tunnelcolor[num_circle + 2] =
      QCPCurveData(num_circle + 2, azimuth_radius * std::cos(-tunnel_angle / 2),
                   azimuth_radius * std::sin(-tunnel_angle / 2));
  dataSpiral_tunnelnocolor[num_circle] = QCPCurveData(
      num_circle, xvalue, azimuth_radius * std::sin(tunnel_angle / 2 + M_PI));
  dataSpiral_tunnelnocolor[num_circle + 1] =
      QCPCurveData(num_circle + 1, xvalue,
                   azimuth_radius * std::sin(-tunnel_angle / 2 + M_PI));
  dataSpiral_tunnelnocolor[num_circle + 2] = QCPCurveData(
      num_circle + 2, azimuth_radius * std::cos(-tunnel_angle / 2 + M_PI),
      azimuth_radius * std::sin(-tunnel_angle / 2 + M_PI));
  // create curve objects
  fermatSpiral_tunnelcolor = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_tunnelcolor->data()->set(dataSpiral_tunnelcolor, true);
  fermatSpiral_tunnelcolor->setPen(QPen(QColor(101, 30, 62, 255), 1));
  fermatSpiral_tunnelcolor->setBrush(QBrush(QColor(99, 172, 229)));
  fermatSpiral_tunnelnocolor =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_tunnelnocolor->data()->set(dataSpiral_tunnelnocolor, true);
  fermatSpiral_tunnelnocolor->setPen(QPen(QColor(101, 30, 62, 255), 1));
  fermatSpiral_tunnelnocolor->setBrush(QBrush(QColor(255, 255, 255)));
}

void DialogThrusterDiagI::updateProgressBar(double _center_x, double _center_y,
                                            double _percent,
                                            double _orientation) {
  // generate data for circle
  double half_angle = tunnel_angle / 2;
  int num_circle = arraylength_tunnel - 3;
  for (unsigned i = 0; i != num_circle; ++i) {
    double theta =
        _orientation + i * tunnel_angle / (num_circle - 1) - half_angle;
    double cvalue = azimuth_radius * std::cos(theta);
    double svalue = azimuth_radius * std::sin(theta);
    dataSpiral_tunnelcolor[i] = QCPCurveData(i, cvalue, svalue);
    dataSpiral_tunnelnocolor[i] = QCPCurveData(i, -cvalue, -svalue);
  }

  double length = _percent * 2 * azimuth_radius * std::cos(half_angle);
  double Xb = azimuth_radius * std::cos(_orientation - half_angle);
  double Yb = azimuth_radius * std::sin(_orientation - half_angle);
  double X_pa = azimuth_radius * std::cos(_orientation + half_angle) -
                length * std::cos(_orientation);
  double Y_pa = azimuth_radius * std::sin(_orientation + half_angle) -
                length * std::sin(_orientation);
  double X_pb = Xb - length * std::cos(_orientation);
  double Y_pb = Yb - length * std::sin(_orientation);
  dataSpiral_tunnelcolor[num_circle] = QCPCurveData(num_circle, X_pa, Y_pa);
  dataSpiral_tunnelcolor[num_circle + 1] =
      QCPCurveData(num_circle + 1, X_pb, Y_pb);
  dataSpiral_tunnelcolor[num_circle + 2] =
      QCPCurveData(num_circle + 2, azimuth_radius * std::cos(-tunnel_angle / 2),
                   azimuth_radius * std::sin(-tunnel_angle / 2));
  dataSpiral_tunnelnocolor[num_circle] = QCPCurveData(
      num_circle, xvalue, azimuth_radius * std::sin(tunnel_angle / 2 + M_PI));
  dataSpiral_tunnelnocolor[num_circle + 1] =
      QCPCurveData(num_circle + 1, xvalue,
                   azimuth_radius * std::sin(-tunnel_angle / 2 + M_PI));
  dataSpiral_tunnelnocolor[num_circle + 2] = QCPCurveData(
      num_circle + 2, azimuth_radius * std::cos(-tunnel_angle / 2 + M_PI),
      azimuth_radius * std::sin(-tunnel_angle / 2 + M_PI));
  // create curve objects
  QCPCurve *fermatSpiral_tunnelcolor =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_tunnelcolor->data()->set(dataSpiral_tunnelcolor, true);
  fermatSpiral_tunnelcolor->setPen(QPen(QColor(101, 30, 62, 255), 1));
  fermatSpiral_tunnelcolor->setBrush(QBrush(QColor(99, 172, 229)));
  QCPCurve *fermatSpiral_tunnelnocolor =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_tunnelnocolor->data()->set(dataSpiral_tunnelnocolor, true);
  fermatSpiral_tunnelnocolor->setPen(QPen(QColor(101, 30, 62, 255), 1));
  fermatSpiral_tunnelnocolor->setBrush(QBrush(QColor(255, 255, 255)));
}