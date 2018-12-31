#include "dialogthrusterdiagi.h"
#include "ui_dialogthrusterdiagi.h"

DialogThrusterDiagI::DialogThrusterDiagI(QWidget *parent)
    : QDialog(parent),
      ui(new Ui::DialogThrusterDiagI),
      STBD_position({3.5, 3.7}),
      dataSpiral_tunnelcolor_STBD(arraylength_azimuth_first),
      dataSpiral_tunnelnocolor_STBD(arraylength_azimuth_first),
      dataSpiral_ball_STBD(arraylength_azimuth_first),
      PORT_position({3.5, 2.3}),
      dataSpiral_tunnelcolor_PORT(arraylength_azimuth_first),
      dataSpiral_tunnelnocolor_PORT(arraylength_azimuth_first),
      dataSpiral_ball_PORT(arraylength_azimuth_first),
      tunnel_first_position({-3, 3}),
      dataSpiral_tunnel_first(5),
      tunnel_second_position({-2, 3}),
      dataSpiral_tunnel_second(5) {
  ui->setupUi(this);
  initializeAllUI(ui->customPlot_thruster);
  setupthrusterRealtimeData();
}

DialogThrusterDiagI::~DialogThrusterDiagI() { delete ui; }

void DialogThrusterDiagI::initializeAllUI(QCustomPlot *customPlot) {
  // setup all colors used
  QPalette pal_background = palette();
  pal_background.setColor(QPalette::Background, QColor(68, 68, 68, 255));

  // set the whole dialog
  // this->setWindowState(Qt::WindowMaximized);
  this->setGeometry(0, 0, 1010, 420);
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
  initializePORTProgressBar(customPlot);
  initializeTunnelProgressBar_first(customPlot);
  initializeTunnelProgressBar_second(customPlot);
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
        i, STBD_position[0] + azimuth_radius_first * std::cos(theta),
        STBD_position[1] + azimuth_radius_first * std::sin(theta));
  }

  // create curve objects for outer circle
  QCPCurve *fermatSpiral_outercircle =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_outercircle->data()->set(dataSpiral_outercircle, true);
  fermatSpiral_outercircle->setPen(QPen(QColor(68, 68, 68, 255), 2));
  fermatSpiral_outercircle->setBrush(QBrush(QColor(170, 170, 170)));

  // generate data for progress bar on the right
  int num_circle = arraylength_azimuth_first - 3;
  for (unsigned i = 0; i != num_circle; ++i) {
    double theta =
        i * tunnel_angle_first / (num_circle - 1) - tunnel_angle_first / 2;
    double cvalue = azimuth_radius_first * std::cos(theta);
    double svalue = azimuth_radius_first * std::sin(theta);
    dataSpiral_tunnelcolor_STBD[i] =
        QCPCurveData(i, STBD_position[0] + cvalue, STBD_position[1] + svalue);
    dataSpiral_tunnelnocolor_STBD[i] =
        QCPCurveData(i, STBD_position[0] - cvalue, STBD_position[1] - svalue);
  }

  double xvalue = -0.1;
  dataSpiral_tunnelcolor_STBD[num_circle] =
      QCPCurveData(num_circle, STBD_position[0] + xvalue,
                   STBD_position[1] +
                       azimuth_radius_first * std::sin(tunnel_angle_first / 2));
  dataSpiral_tunnelcolor_STBD[num_circle + 1] = QCPCurveData(
      num_circle + 1, STBD_position[0] + xvalue,
      STBD_position[1] +
          azimuth_radius_first * std::sin(-tunnel_angle_first / 2));
  dataSpiral_tunnelcolor_STBD[num_circle + 2] = QCPCurveData(
      num_circle + 2,
      STBD_position[0] +
          azimuth_radius_first * std::cos(-tunnel_angle_first / 2),
      STBD_position[1] +
          azimuth_radius_first * std::sin(-tunnel_angle_first / 2));
  dataSpiral_tunnelnocolor_STBD[num_circle] =
      QCPCurveData(num_circle, STBD_position[0] + xvalue,
                   STBD_position[1] -
                       azimuth_radius_first * std::sin(tunnel_angle_first / 2));
  dataSpiral_tunnelnocolor_STBD[num_circle + 1] = QCPCurveData(
      num_circle + 1, STBD_position[0] + xvalue,
      STBD_position[1] -
          azimuth_radius_first * std::sin(-tunnel_angle_first / 2));
  dataSpiral_tunnelnocolor_STBD[num_circle + 2] = QCPCurveData(
      num_circle + 2,
      STBD_position[0] -
          azimuth_radius_first * std::cos(-tunnel_angle_first / 2),
      STBD_position[1] -
          azimuth_radius_first * std::sin(-tunnel_angle_first / 2));
  // create curve objects
  fermatSpiral_tunnelcolor_STBD =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_tunnelcolor_STBD->data()->set(dataSpiral_tunnelcolor_STBD, true);
  fermatSpiral_tunnelcolor_STBD->setPen(QPen(QColor(101, 30, 62, 255), 1));
  fermatSpiral_tunnelcolor_STBD->setBrush(QBrush(QColor(99, 172, 229)));
  fermatSpiral_tunnelnocolor_STBD =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_tunnelnocolor_STBD->data()->set(dataSpiral_tunnelnocolor_STBD,
                                               true);
  fermatSpiral_tunnelnocolor_STBD->setPen(QPen(QColor(101, 30, 62, 255), 1));
  fermatSpiral_tunnelnocolor_STBD->setBrush(QBrush(QColor(255, 255, 255)));

  // generate data for ball
  for (unsigned i = 0; i != arraylength_azimuth_first; ++i) {
    double theta = 2 * i * M_PI / (arraylength_azimuth_first - 1);
    dataSpiral_ball_STBD[i] =
        QCPCurveData(i, STBD_position[0] + azimuth_radius_first +
                            ball_radius_first * std::cos(theta),
                     STBD_position[1] + ball_radius_first * std::sin(theta));
  }

  // create curve objects for Ball
  fermatSpiral_ball_STBD = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_ball_STBD->data()->set(dataSpiral_ball_STBD, true);
  fermatSpiral_ball_STBD->setPen(QPen(Qt::black, 1));
  fermatSpiral_ball_STBD->setBrush(QBrush(Qt::black));

  // create static text for angle
  QCPItemText *angleText = new QCPItemText(customPlot);
  angleText->position->setCoords(STBD_position[0] - 4.5 * azimuth_radius_first,
                                 STBD_position[1] + 0.7 * azimuth_radius_first);
  angleText->setText("STBDAzi (°)");
  angleText->setFont(QFont("SansSerif", 10));
  angleText->setPositionAlignment(Qt::AlignTop | Qt::AlignLeft);
  angleText->setColor(Qt::black);
  // create realtime text for angle
  STBD_angleText = new QCPItemText(customPlot);
  STBD_angleText->position->setCoords(
      STBD_position[0] - 2 * azimuth_radius_first,
      STBD_position[1] + 0.7 * azimuth_radius_first);
  STBD_angleText->setText("0");
  STBD_angleText->setFont(QFont("SansSerif", 10));
  STBD_angleText->setPositionAlignment(Qt::AlignTop | Qt::AlignLeft);
  STBD_angleText->setColor(Qt::black);

  // create static text for RPM
  QCPItemText *RPMText = new QCPItemText(customPlot);
  RPMText->position->setCoords(STBD_position[0] - 4.5 * azimuth_radius_first,
                               STBD_position[1] - 0.5 * azimuth_radius_first);
  RPMText->setText("RPM");
  RPMText->setFont(QFont("SansSerif", 10));
  RPMText->setPositionAlignment(Qt::AlignBottom | Qt::AlignLeft);
  RPMText->setColor(Qt::black);
  // create realtime text for RPM
  STBD_RPMText = new QCPItemText(customPlot);
  STBD_RPMText->position->setCoords(
      STBD_position[0] - 2 * azimuth_radius_first,
      STBD_position[1] - 0.5 * azimuth_radius_first);
  STBD_RPMText->setText("0");
  STBD_RPMText->setFont(QFont("SansSerif", 10));
  STBD_RPMText->setPositionAlignment(Qt::AlignBottom | Qt::AlignLeft);
  STBD_RPMText->setColor(Qt::black);
}
//
void DialogThrusterDiagI::initializePORTProgressBar(QCustomPlot *customPlot) {
  // generate data for outer circle
  const int arraylength_outercircle = 50;
  QVector<QCPCurveData> dataSpiral_outercircle(arraylength_outercircle);

  for (unsigned i = 0; i != arraylength_outercircle; ++i) {
    double theta = 2 * i * M_PI / (arraylength_outercircle - 1);
    dataSpiral_outercircle[i] = QCPCurveData(
        i, PORT_position[0] + azimuth_radius_first * std::cos(theta),
        PORT_position[1] + azimuth_radius_first * std::sin(theta));
  }

  // create curve objects for outer circle
  QCPCurve *fermatSpiral_outercircle =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_outercircle->data()->set(dataSpiral_outercircle, true);
  fermatSpiral_outercircle->setPen(QPen(QColor(68, 68, 68, 255), 2));
  fermatSpiral_outercircle->setBrush(QBrush(QColor(170, 170, 170)));

  // generate data for progress bar on the right
  int num_circle = arraylength_azimuth_first - 3;
  for (unsigned i = 0; i != num_circle; ++i) {
    double theta =
        i * tunnel_angle_first / (num_circle - 1) - tunnel_angle_first / 2;
    double cvalue = azimuth_radius_first * std::cos(theta);
    double svalue = azimuth_radius_first * std::sin(theta);
    dataSpiral_tunnelcolor_PORT[i] =
        QCPCurveData(i, PORT_position[0] + cvalue, PORT_position[1] + svalue);
    dataSpiral_tunnelnocolor_PORT[i] =
        QCPCurveData(i, PORT_position[0] - cvalue, PORT_position[1] - svalue);
  }

  double xvalue = -0.1;
  dataSpiral_tunnelcolor_PORT[num_circle] =
      QCPCurveData(num_circle, PORT_position[0] + xvalue,
                   PORT_position[1] +
                       azimuth_radius_first * std::sin(tunnel_angle_first / 2));
  dataSpiral_tunnelcolor_PORT[num_circle + 1] = QCPCurveData(
      num_circle + 1, PORT_position[0] + xvalue,
      PORT_position[1] +
          azimuth_radius_first * std::sin(-tunnel_angle_first / 2));
  dataSpiral_tunnelcolor_PORT[num_circle + 2] = QCPCurveData(
      num_circle + 2,
      PORT_position[0] +
          azimuth_radius_first * std::cos(-tunnel_angle_first / 2),
      PORT_position[1] +
          azimuth_radius_first * std::sin(-tunnel_angle_first / 2));
  dataSpiral_tunnelnocolor_PORT[num_circle] =
      QCPCurveData(num_circle, PORT_position[0] + xvalue,
                   PORT_position[1] -
                       azimuth_radius_first * std::sin(tunnel_angle_first / 2));
  dataSpiral_tunnelnocolor_PORT[num_circle + 1] = QCPCurveData(
      num_circle + 1, PORT_position[0] + xvalue,
      PORT_position[1] -
          azimuth_radius_first * std::sin(-tunnel_angle_first / 2));
  dataSpiral_tunnelnocolor_PORT[num_circle + 2] = QCPCurveData(
      num_circle + 2,
      PORT_position[0] -
          azimuth_radius_first * std::cos(-tunnel_angle_first / 2),
      PORT_position[1] -
          azimuth_radius_first * std::sin(-tunnel_angle_first / 2));
  // create curve objects
  fermatSpiral_tunnelcolor_PORT =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_tunnelcolor_PORT->data()->set(dataSpiral_tunnelcolor_PORT, true);
  fermatSpiral_tunnelcolor_PORT->setPen(QPen(QColor(101, 30, 62, 255), 1));
  fermatSpiral_tunnelcolor_PORT->setBrush(QBrush(QColor(99, 172, 229)));
  fermatSpiral_tunnelnocolor_PORT =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_tunnelnocolor_PORT->data()->set(dataSpiral_tunnelnocolor_PORT,
                                               true);
  fermatSpiral_tunnelnocolor_PORT->setPen(QPen(QColor(101, 30, 62, 255), 1));
  fermatSpiral_tunnelnocolor_PORT->setBrush(QBrush(QColor(255, 255, 255)));

  // generate data for ball
  for (unsigned i = 0; i != arraylength_azimuth_first; ++i) {
    double theta = 2 * i * M_PI / (arraylength_azimuth_first - 1);
    dataSpiral_ball_PORT[i] =
        QCPCurveData(i, PORT_position[0] + azimuth_radius_first +
                            ball_radius_first * std::cos(theta),
                     PORT_position[1] + ball_radius_first * std::sin(theta));
  }

  // create curve objects for Ball
  fermatSpiral_ball_PORT = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_ball_PORT->data()->set(dataSpiral_ball_PORT, true);
  fermatSpiral_ball_PORT->setPen(QPen(Qt::black, 1));
  fermatSpiral_ball_PORT->setBrush(QBrush(Qt::black));

  // create static text for angle
  QCPItemText *angleText = new QCPItemText(customPlot);
  angleText->position->setCoords(PORT_position[0] - 4.5 * azimuth_radius_first,
                                 PORT_position[1] + 0.7 * azimuth_radius_first);
  angleText->setText("PORTAzi (°)");
  angleText->setFont(QFont("SansSerif", 10));
  angleText->setPositionAlignment(Qt::AlignTop | Qt::AlignLeft);
  angleText->setColor(Qt::black);
  // create realtime text for angle
  PORT_angleText = new QCPItemText(customPlot);
  PORT_angleText->position->setCoords(
      PORT_position[0] - 2 * azimuth_radius_first,
      PORT_position[1] + 0.7 * azimuth_radius_first);
  PORT_angleText->setText("0");
  PORT_angleText->setFont(QFont("SansSerif", 10));
  PORT_angleText->setPositionAlignment(Qt::AlignTop | Qt::AlignLeft);
  PORT_angleText->setColor(Qt::black);

  // create static text for RPM
  QCPItemText *RPMText = new QCPItemText(customPlot);
  RPMText->position->setCoords(PORT_position[0] - 4.5 * azimuth_radius_first,
                               PORT_position[1] - 0.5 * azimuth_radius_first);
  RPMText->setText("RPM");
  RPMText->setFont(QFont("SansSerif", 10));
  RPMText->setPositionAlignment(Qt::AlignBottom | Qt::AlignLeft);
  RPMText->setColor(Qt::black);
  // create realtime text for RPM
  PORT_RPMText = new QCPItemText(customPlot);
  PORT_RPMText->position->setCoords(
      PORT_position[0] - 2 * azimuth_radius_first,
      PORT_position[1] - 0.5 * azimuth_radius_first);
  PORT_RPMText->setText("0");
  PORT_RPMText->setFont(QFont("SansSerif", 10));
  PORT_RPMText->setPositionAlignment(Qt::AlignBottom | Qt::AlignLeft);
  PORT_RPMText->setColor(Qt::black);
}

void DialogThrusterDiagI::initializeTunnelProgressBar_first(
    QCustomPlot *customPlot) {
  // generate data for outer square
  const int arraylength_outersquare = 5;
  QVector<QCPCurveData> dataSpiral_outersquare(arraylength_outersquare);

  dataSpiral_outersquare[0] =
      QCPCurveData(0, tunnel_first_position[0] + tunnel_width_first / 2,
                   tunnel_first_position[1] + tunnel_length_first / 2);
  dataSpiral_outersquare[1] =
      QCPCurveData(1, tunnel_first_position[0] - tunnel_width_first / 2,
                   tunnel_first_position[1] + tunnel_length_first / 2);
  dataSpiral_outersquare[2] =
      QCPCurveData(2, tunnel_first_position[0] - tunnel_width_first / 2,
                   tunnel_first_position[1] - tunnel_length_first / 2);
  dataSpiral_outersquare[3] =
      QCPCurveData(3, tunnel_first_position[0] + tunnel_width_first / 2,
                   tunnel_first_position[1] - tunnel_length_first / 2);
  dataSpiral_outersquare[4] =
      QCPCurveData(4, tunnel_first_position[0] + tunnel_width_first / 2,
                   tunnel_first_position[1] + tunnel_length_first / 2);
  // create curve objects for outer circle
  QCPCurve *fermatSpiral_outersquare =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_outersquare->data()->set(dataSpiral_outersquare, true);
  fermatSpiral_outersquare->setPen(QPen(QColor(0, 0, 0, 255), 2));
  fermatSpiral_outersquare->setBrush(QBrush(QColor(255, 255, 255)));

  // generate data for progress data
  dataSpiral_tunnel_first[0] =
      QCPCurveData(0, tunnel_first_position[0] + tunnel_width_first / 2,
                   tunnel_first_position[1]);
  dataSpiral_tunnel_first[1] =
      QCPCurveData(1, tunnel_first_position[0] + tunnel_width_first / 2,
                   tunnel_first_position[1] + tunnel_length_first / 3);
  dataSpiral_tunnel_first[2] =
      QCPCurveData(2, tunnel_first_position[0] - tunnel_width_first / 2,
                   tunnel_first_position[1] + tunnel_length_first / 3);
  dataSpiral_tunnel_first[3] =
      QCPCurveData(3, tunnel_first_position[0] - tunnel_width_first / 2,
                   tunnel_first_position[1]);
  dataSpiral_tunnel_first[4] =
      QCPCurveData(4, tunnel_first_position[0] + tunnel_width_first / 2,
                   tunnel_first_position[1]);

  fermatSpiral_tunnel_first =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_tunnel_first->data()->set(dataSpiral_tunnel_first, true);
  fermatSpiral_tunnel_first->setPen(QPen(QColor(0, 0, 0, 255), 2));
  fermatSpiral_tunnel_first->setBrush(QBrush(QColor(99, 172, 229)));
  // create realtime text for angle
  tunnel_first_RPMText = new QCPItemText(customPlot);
  tunnel_first_RPMText->position->setCoords(
      tunnel_first_position[0],
      tunnel_first_position[1] + 0.8 * tunnel_length_first);
  tunnel_first_RPMText->setText("0");
  tunnel_first_RPMText->setFont(QFont("SansSerif", 10));
  tunnel_first_RPMText->setPositionAlignment(Qt::AlignTop | Qt::AlignLeft);
  tunnel_first_RPMText->setColor(Qt::black);
}

void DialogThrusterDiagI::updateTunnelProgressBar_first(int _rpm) {
  // generate data for progress data
  double progress_y =
      tunnel_length_first * _rpm / max_tunnel_rotation_first / 2;
  dataSpiral_tunnel_first[0] =
      QCPCurveData(0, tunnel_first_position[0] + tunnel_width_first / 2,
                   tunnel_first_position[1]);
  dataSpiral_tunnel_first[1] =
      QCPCurveData(1, tunnel_first_position[0] + tunnel_width_first / 2,
                   tunnel_first_position[1] + progress_y);
  dataSpiral_tunnel_first[2] =
      QCPCurveData(2, tunnel_first_position[0] - tunnel_width_first / 2,
                   tunnel_first_position[1] + progress_y);
  dataSpiral_tunnel_first[3] =
      QCPCurveData(3, tunnel_first_position[0] - tunnel_width_first / 2,
                   tunnel_first_position[1]);
  dataSpiral_tunnel_first[4] =
      QCPCurveData(4, tunnel_first_position[0] + tunnel_width_first / 2,
                   tunnel_first_position[1]);
  fermatSpiral_tunnel_first->data()->set(dataSpiral_tunnel_first, true);
  // create realtime text for angle
  tunnel_first_RPMText->setText(QString::number(_rpm));
}

void DialogThrusterDiagI::updateTunnelProgressBar_second(int _rpm) {
  // generate data for progress data
  double progress_y =
      tunnel_length_first * _rpm / max_tunnel_rotation_first / 2;
  dataSpiral_tunnel_second[0] =
      QCPCurveData(0, tunnel_second_position[0] + tunnel_width_first / 2,
                   tunnel_second_position[1]);
  dataSpiral_tunnel_second[1] =
      QCPCurveData(1, tunnel_second_position[0] + tunnel_width_first / 2,
                   tunnel_second_position[1] + progress_y);
  dataSpiral_tunnel_second[2] =
      QCPCurveData(2, tunnel_second_position[0] - tunnel_width_first / 2,
                   tunnel_second_position[1] + progress_y);
  dataSpiral_tunnel_second[3] =
      QCPCurveData(3, tunnel_second_position[0] - tunnel_width_first / 2,
                   tunnel_second_position[1]);
  dataSpiral_tunnel_second[4] =
      QCPCurveData(4, tunnel_second_position[0] + tunnel_width_first / 2,
                   tunnel_second_position[1]);
  fermatSpiral_tunnel_second->data()->set(dataSpiral_tunnel_second, true);
  // create realtime text for angle
  tunnel_second_RPMText->setText(QString::number(_rpm));
}

void DialogThrusterDiagI::initializeTunnelProgressBar_second(
    QCustomPlot *customPlot) {
  // generate data for outer square
  const int arraylength_outersquare = 5;
  QVector<QCPCurveData> dataSpiral_outersquare(arraylength_outersquare);

  dataSpiral_outersquare[0] =
      QCPCurveData(0, tunnel_second_position[0] + tunnel_width_first / 2,
                   tunnel_second_position[1] + tunnel_length_first / 2);
  dataSpiral_outersquare[1] =
      QCPCurveData(1, tunnel_second_position[0] - tunnel_width_first / 2,
                   tunnel_second_position[1] + tunnel_length_first / 2);
  dataSpiral_outersquare[2] =
      QCPCurveData(2, tunnel_second_position[0] - tunnel_width_first / 2,
                   tunnel_second_position[1] - tunnel_length_first / 2);
  dataSpiral_outersquare[3] =
      QCPCurveData(3, tunnel_second_position[0] + tunnel_width_first / 2,
                   tunnel_second_position[1] - tunnel_length_first / 2);
  dataSpiral_outersquare[4] =
      QCPCurveData(4, tunnel_second_position[0] + tunnel_width_first / 2,
                   tunnel_second_position[1] + tunnel_length_first / 2);
  // create curve objects for outer circle
  QCPCurve *fermatSpiral_outersquare =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_outersquare->data()->set(dataSpiral_outersquare, true);
  fermatSpiral_outersquare->setPen(QPen(QColor(0, 0, 0, 255), 2));
  fermatSpiral_outersquare->setBrush(QBrush(QColor(255, 255, 255)));

  // generate data for progress data
  dataSpiral_tunnel_second[0] =
      QCPCurveData(0, tunnel_second_position[0] + tunnel_width_first / 2,
                   tunnel_second_position[1]);
  dataSpiral_tunnel_second[1] =
      QCPCurveData(1, tunnel_second_position[0] + tunnel_width_first / 2,
                   tunnel_second_position[1] + tunnel_length_first / 3);
  dataSpiral_tunnel_second[2] =
      QCPCurveData(2, tunnel_second_position[0] - tunnel_width_first / 2,
                   tunnel_second_position[1] + tunnel_length_first / 3);
  dataSpiral_tunnel_second[3] =
      QCPCurveData(3, tunnel_second_position[0] - tunnel_width_first / 2,
                   tunnel_second_position[1]);
  dataSpiral_tunnel_second[4] =
      QCPCurveData(4, tunnel_second_position[0] + tunnel_width_first / 2,
                   tunnel_second_position[1]);

  fermatSpiral_tunnel_second =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_tunnel_second->data()->set(dataSpiral_tunnel_second, true);
  fermatSpiral_tunnel_second->setPen(QPen(QColor(0, 0, 0, 255), 2));
  fermatSpiral_tunnel_second->setBrush(QBrush(QColor(99, 172, 229)));

  // create realtime text for angle
  tunnel_second_RPMText = new QCPItemText(customPlot);
  tunnel_second_RPMText->position->setCoords(
      tunnel_second_position[0],
      tunnel_second_position[1] + 0.8 * tunnel_length_first);
  tunnel_second_RPMText->setText("0");
  tunnel_second_RPMText->setFont(QFont("SansSerif", 10));
  tunnel_second_RPMText->setPositionAlignment(Qt::AlignTop | Qt::AlignLeft);
  tunnel_second_RPMText->setColor(Qt::black);
}

void DialogThrusterDiagI::updateSTBDProgressBar(double _percent,
                                                double _orientation) {
  // generate data for circle
  double half_angle = tunnel_angle_first / 2;
  int num_circle = arraylength_azimuth_first - 3;
  for (unsigned i = 0; i != num_circle; ++i) {
    double theta =
        _orientation + i * tunnel_angle_first / (num_circle - 1) - half_angle;
    double cvalue = azimuth_radius_first * std::cos(theta);
    double svalue = azimuth_radius_first * std::sin(theta);
    dataSpiral_tunnelcolor_STBD[i] =
        QCPCurveData(i, STBD_position[0] + cvalue, STBD_position[1] + svalue);
    dataSpiral_tunnelnocolor_STBD[i] =
        QCPCurveData(i, STBD_position[0] - cvalue, STBD_position[1] - svalue);
  }

  double length = _percent * 2 * azimuth_radius_first * std::cos(half_angle);
  double Xb = azimuth_radius_first * std::cos(_orientation - half_angle);
  double Yb = azimuth_radius_first * std::sin(_orientation - half_angle);
  double X_pa = azimuth_radius_first * std::cos(_orientation + half_angle) -
                length * std::cos(_orientation);
  double Y_pa = azimuth_radius_first * std::sin(_orientation + half_angle) -
                length * std::sin(_orientation);
  double X_pb = Xb - length * std::cos(_orientation);
  double Y_pb = Yb - length * std::sin(_orientation);
  dataSpiral_tunnelcolor_STBD[num_circle] = QCPCurveData(
      num_circle, STBD_position[0] + X_pa, STBD_position[1] + Y_pa);
  dataSpiral_tunnelcolor_STBD[num_circle + 1] = QCPCurveData(
      num_circle + 1, STBD_position[0] + X_pb, STBD_position[1] + Y_pb);
  dataSpiral_tunnelcolor_STBD[num_circle + 2] = QCPCurveData(
      num_circle + 2, STBD_position[0] + Xb, STBD_position[1] + Yb);
  dataSpiral_tunnelnocolor_STBD[num_circle] = QCPCurveData(
      num_circle, STBD_position[0] + X_pb, STBD_position[1] + Y_pb);
  dataSpiral_tunnelnocolor_STBD[num_circle + 1] = QCPCurveData(
      num_circle + 1, STBD_position[0] + X_pa, STBD_position[1] + Y_pa);
  dataSpiral_tunnelnocolor_STBD[num_circle + 2] = QCPCurveData(
      num_circle + 2, STBD_position[0] - Xb, STBD_position[1] - Yb);

  // update curve objects for azimuth thruster
  fermatSpiral_tunnelcolor_STBD->data()->set(dataSpiral_tunnelcolor_STBD, true);
  fermatSpiral_tunnelnocolor_STBD->data()->set(dataSpiral_tunnelnocolor_STBD,
                                               true);

  // generate data for ball
  double ball_cvalue =
      STBD_position[0] + azimuth_radius_first * std::cos(_orientation);
  double ball_svalue =
      STBD_position[1] + azimuth_radius_first * std::sin(_orientation);
  for (unsigned i = 0; i != arraylength_azimuth_first; ++i) {
    double theta = 2 * i * M_PI / (arraylength_azimuth_first - 1);
    dataSpiral_ball_STBD[i] =
        QCPCurveData(i, ball_cvalue + ball_radius_first * std::cos(theta),
                     ball_svalue + ball_radius_first * std::sin(theta));
  }

  // create curve objects for Ball
  fermatSpiral_ball_STBD->data()->set(dataSpiral_ball_STBD, true);
}

void DialogThrusterDiagI::updatePORTProgressBar(double _percent,
                                                double _orientation) {
  // generate data for circle
  double half_angle = tunnel_angle_first / 2;
  int num_circle = arraylength_azimuth_first - 3;
  for (unsigned i = 0; i != num_circle; ++i) {
    double theta =
        _orientation + i * tunnel_angle_first / (num_circle - 1) - half_angle;
    double cvalue = azimuth_radius_first * std::cos(theta);
    double svalue = azimuth_radius_first * std::sin(theta);
    dataSpiral_tunnelcolor_PORT[i] =
        QCPCurveData(i, PORT_position[0] + cvalue, PORT_position[1] + svalue);
    dataSpiral_tunnelnocolor_PORT[i] =
        QCPCurveData(i, PORT_position[0] - cvalue, PORT_position[1] - svalue);
  }

  double length = _percent * 2 * azimuth_radius_first * std::cos(half_angle);
  double Xb = azimuth_radius_first * std::cos(_orientation - half_angle);
  double Yb = azimuth_radius_first * std::sin(_orientation - half_angle);
  double X_pa = azimuth_radius_first * std::cos(_orientation + half_angle) -
                length * std::cos(_orientation);
  double Y_pa = azimuth_radius_first * std::sin(_orientation + half_angle) -
                length * std::sin(_orientation);
  double X_pb = Xb - length * std::cos(_orientation);
  double Y_pb = Yb - length * std::sin(_orientation);
  dataSpiral_tunnelcolor_PORT[num_circle] = QCPCurveData(
      num_circle, PORT_position[0] + X_pa, PORT_position[1] + Y_pa);
  dataSpiral_tunnelcolor_PORT[num_circle + 1] = QCPCurveData(
      num_circle + 1, PORT_position[0] + X_pb, PORT_position[1] + Y_pb);
  dataSpiral_tunnelcolor_PORT[num_circle + 2] = QCPCurveData(
      num_circle + 2, PORT_position[0] + Xb, PORT_position[1] + Yb);
  dataSpiral_tunnelnocolor_PORT[num_circle] = QCPCurveData(
      num_circle, PORT_position[0] + X_pb, PORT_position[1] + Y_pb);
  dataSpiral_tunnelnocolor_PORT[num_circle + 1] = QCPCurveData(
      num_circle + 1, PORT_position[0] + X_pa, PORT_position[1] + Y_pa);
  dataSpiral_tunnelnocolor_PORT[num_circle + 2] = QCPCurveData(
      num_circle + 2, PORT_position[0] - Xb, PORT_position[1] - Yb);

  // update curve objects for azimuth thruster
  fermatSpiral_tunnelcolor_PORT->data()->set(dataSpiral_tunnelcolor_PORT, true);
  fermatSpiral_tunnelnocolor_PORT->data()->set(dataSpiral_tunnelnocolor_PORT,
                                               true);

  // generate data for ball
  double ball_cvalue =
      PORT_position[0] + azimuth_radius_first * std::cos(_orientation);
  double ball_svalue =
      PORT_position[1] + azimuth_radius_first * std::sin(_orientation);
  for (unsigned i = 0; i != arraylength_azimuth_first; ++i) {
    double theta = 2 * i * M_PI / (arraylength_azimuth_first - 1);
    dataSpiral_ball_PORT[i] =
        QCPCurveData(i, ball_cvalue + ball_radius_first * std::cos(theta),
                     ball_svalue + ball_radius_first * std::sin(theta));
  }

  // create curve objects for Ball
  fermatSpiral_ball_PORT->data()->set(dataSpiral_ball_PORT, true);
}

void DialogThrusterDiagI::updateSTBDText(int _rpm, int _orientation) {
  STBD_angleText->setText(QString::number(_orientation));
  STBD_RPMText->setText(QString::number(_rpm));
}

void DialogThrusterDiagI::updatePORTText(int _rpm, int _orientation) {
  PORT_angleText->setText(QString::number(_orientation));
  PORT_RPMText->setText(QString::number(_rpm));
}

void DialogThrusterDiagI::setupthrusterRealtimeData() {
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(AzimuthDataSlot()));
  // connect(&dataTimer, SIGNAL(timeout()), this, SLOT(motion6DOFdataSlot()));
  dataTimer.start(VIEWERREFRESH);
}

void DialogThrusterDiagI::AzimuthDataSlot() {
  Eigen::Vector3i _alpha_deg_first =
      globalvar::_threadloop.getrealtimealphadeg_first();
  Eigen::Vector3i _rotation_first =
      globalvar::_threadloop.getrealtimerotation_first();

  int rpm_port = _rotation_first(1);
  int angle_port = _alpha_deg_first(1);
  int rpm_star = _rotation_first(2);
  int angle_star = _alpha_deg_first(2);
  updateSTBDText(rpm_star, angle_star);
  updatePORTText(rpm_port, angle_port);
  updateSTBDProgressBar((double)rpm_star / max_azimuth_rotation_first,
                        -angle_star * M_PI / 180);
  updatePORTProgressBar((double)rpm_port / max_azimuth_rotation_first,
                        -angle_port * M_PI / 180);
  updateTunnelProgressBar_first(_rotation_first(0));
  updateTunnelProgressBar_second(_rotation_first(0));
  ui->customPlot_thruster->replot();
}