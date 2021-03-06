#include "display2ddialog.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QMetaEnum>
#include <QScreen>
#include "ui_display2ddialog.h"

Display2DDialog::Display2DDialog(QWidget *parent)
    : QDialog(parent),
      ui(new Ui::Display2DDialog),
      V_Qcolor({QColor(209, 17, 65), QColor(0, 177, 89), QColor(0, 174, 219)}) {
  ui->setupUi(this);
  initializePlanarMotionData();
  initializeAllUI();
  setupVesselRealtimeData();
  setWindowTitle(demoName);
  ui->status_text->clear();
}

Display2DDialog::~Display2DDialog() { delete ui; }

void Display2DDialog::initializeAllUI() {
  // setup all colors used
  QPalette pal_background = palette();
  QPalette pal_2D = palette();
  pal_background.setColor(QPalette::Background, QColor(68, 68, 68, 255));
  pal_2D.setColor(QPalette::Background, Qt::white);

  // set the whole dialog
  this->setGeometry(0, 0, size_2ddisplay, size_2ddisplay);
  this->setWindowFlags(Qt::Window);
  this->setAutoFillBackground(true);
  this->setPalette(pal_background);
  this->setStyleSheet("QLabel {color : white; }");  // set color for all Qlabels
  // setGeometry(500, 250, 1500, 950);

  // set position and size
  ui->customPlot_2Dmotion->setGeometry(10, 10, size_2ddisplay, size_2ddisplay);
  ui->status_text->setGeometry(20, size_2ddisplay + 15, 300, 25);
  ui->label_mousepos->setGeometry(500, size_2ddisplay + 15, 200, 25);

  // load image as pixmap
  ui->label_north->setGeometry(QRect(size_2ddisplay - 80, 20, 10, 10));
  QString imagepath =
      "/home/skloe/Coding/CPP1X/USV/DPfloatover/QT/build/images/north.png";
  QImage image(imagepath);
  ui->label_north->setPixmap(QPixmap::fromImage((image)));
  ui->label_north->adjustSize();

  // initialize the time series of 6DOF motion and planar motion
  initializePlanarMotion(ui->customPlot_2Dmotion);
  initializeCircle(ui->customPlot_2Dmotion);
  // install event filter for mouse
  ui->customPlot_2Dmotion->installEventFilter(this);
}

// start the real-time viewer for 6DOF and 2DOF motion of vessels
void Display2DDialog::setupVesselRealtimeData() {
  demoName = "Real-time vessel Display";
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(vesselshapeDataSlot()));
  // Interval 0 means to refresh as fast as possible
  dataTimer.start(VIEWERREFRESH);
}

void Display2DDialog::simplerealtimeDataSlot() {
  double secs =
      QCPAxisTickerDateTime::dateTimeToKey(QDateTime::currentDateTime());

  // update data to make phase move:
  double temp = planarmotion_y[0][0];
  planarmotion_y[0].pop_front();
  planarmotion_y[0].push_back(temp + qrand() / (double)RAND_MAX);

  // update the plot
  ui->customPlot_2Dmotion->graph()->setData(planarmotion_x[0],
                                            planarmotion_y[0]);
  ui->customPlot_2Dmotion->graph()->rescaleValueAxis(true);
  ui->customPlot_2Dmotion->replot();

  // calculate frames per second:
  double key = secs;
  static double lastFpsKey;
  static int frameCount;
  ++frameCount;
  if (key - lastFpsKey > 2)  // average fps over 2 seconds
  {
    ui->status_text->setText(
        QString("%1 FPS").arg(frameCount / (key - lastFpsKey), 0, 'f', 0));
    lastFpsKey = key;
    frameCount = 0;
  }
}

// real-time planar motion of each vessel
void Display2DDialog::vesselshapeDataSlot() {
  Vector6d _state_first = globalvar::_threadloop.getrealtime6dmotion_first();
  Vector6d _state_second = globalvar::_threadloop.getrealtime6dmotion_second();
  Vector6d _state_third = globalvar::_threadloop.getrealtime6dmotion_third();
  Eigen::Vector3d _setpoint_first = globalvar::_threadloop.getSetpoints_first();
  Eigen::Vector3d _setpoint_second =
      globalvar::_threadloop.getSetpoints_second();
  Eigen::Vector3d _setpoint_third = globalvar::_threadloop.getSetpoints_third();

  // the first vessel
  if (MAXCONNECTION > 0) {
    convertvessel(_state_first(0), _state_first(1), _state_first(5),
                  planarmotion_y[0], planarmotion_x[0], 0);
    updatetrajectoryvector(_state_first(0), _state_first(1), trajectory_y[0],
                           trajectory_x[0]);
    updatesetpointvector(_setpoint_first(0), _setpoint_first(1), setpoints_y[0],
                         setpoints_x[0]);
    updatesetpointcircle(_setpoint_first(1), _setpoint_first(0), 0);
    ui->customPlot_2Dmotion->graph(0)->setData(planarmotion_x[0],
                                               planarmotion_y[0]);
    ui->customPlot_2Dmotion->graph(MAXCONNECTION)
        ->setData(trajectory_x[0], trajectory_y[0]);
    ui->customPlot_2Dmotion->graph(2 * MAXCONNECTION)
        ->setData(setpoints_x[0], setpoints_y[0]);

    updateheadingarrow(setheadingarrows[0], _setpoint_first(2) * 180 / M_PI);
    updateheadingarrow(realtimeheadingarrows[0], _state_first(5));
  }

  if (MAXCONNECTION > 1) {
    convertvessel(_state_second(0), _state_second(1), _state_second(5),
                  planarmotion_y[1], planarmotion_x[1], 1);
    updatetrajectoryvector(_state_second(0), _state_second(1), trajectory_y[1],
                           trajectory_x[1]);
    updatesetpointvector(_setpoint_second(0), _setpoint_second(1),
                         setpoints_y[1], setpoints_x[1]);
    updatesetpointcircle(_setpoint_second(1), _setpoint_second(0), 1);
    ui->customPlot_2Dmotion->graph(1)->setData(planarmotion_x[1],
                                               planarmotion_y[1]);
    ui->customPlot_2Dmotion->graph(MAXCONNECTION + 1)
        ->setData(trajectory_x[1], trajectory_y[1]);
    ui->customPlot_2Dmotion->graph(1 + 2 * MAXCONNECTION)
        ->setData(setpoints_x[1], setpoints_y[1]);
    updateheadingarrow(setheadingarrows[1], _setpoint_second(2) * 180 / M_PI);
    updateheadingarrow(realtimeheadingarrows[1], _state_second(5));
  }
  if (MAXCONNECTION > 2) {
    convertvessel(_state_third(0), _state_third(1), _state_third(5),
                  planarmotion_y[2], planarmotion_x[2], 2);
    updatetrajectoryvector(_state_third(0), _state_third(1), trajectory_y[2],
                           trajectory_x[2]);
    updatesetpointvector(_setpoint_third(0), _setpoint_third(1), setpoints_y[2],
                         setpoints_x[2]);
    updatesetpointcircle(_setpoint_third(1), _setpoint_third(0), 2);
    ui->customPlot_2Dmotion->graph(2)->setData(planarmotion_x[2],
                                               planarmotion_y[2]);
    ui->customPlot_2Dmotion->graph(2 + MAXCONNECTION)
        ->setData(trajectory_x[2], trajectory_y[2]);
    ui->customPlot_2Dmotion->graph(2 + 2 * MAXCONNECTION)
        ->setData(setpoints_x[2], setpoints_y[2]);
    updateheadingarrow(setheadingarrows[2], _setpoint_third(2) * 180 / M_PI);
    updateheadingarrow(realtimeheadingarrows[2], _state_third(5));
  }
  ui->customPlot_2Dmotion->replot();

  // calculate frames per second:
  double key =
      QCPAxisTickerDateTime::dateTimeToKey(QDateTime::currentDateTime());
  static double lastFpsKey;
  static int frameCount;
  ++frameCount;
  if (key - lastFpsKey > 2)  // average fps over 2 seconds
  {
    ui->status_text->setText(
        QString("%1 FPS").arg(frameCount / (key - lastFpsKey), 0, 'f', 0));
    lastFpsKey = key;
    frameCount = 0;
  }
}

void Display2DDialog::convertvessel(double origin_x, double origin_y,
                                    double t_orient, QVector<double> &t_datax,
                                    QVector<double> &t_datay, int index) {
  // we should exchange x, y between the display coordinate and global
  // coordinate
  // convert degree to rad
  t_orient = t_orient * M_PI / 180;
  double c_value = std::cos(t_orient);
  double s_value = std::sin(t_orient);
  std::vector<double> vesselshape_x = vesselsshapedata[index].x;
  std::vector<double> vesselshape_y = vesselsshapedata[index].y;
  for (unsigned i = 0; i != vesselsshapedata[index].totalnum; ++i) {
    t_datax[i] = c_value * (vesselshape_x[i] - CoG4viewer[index][0]) -
                 s_value * (vesselshape_y[i] - CoG4viewer[index][1]) + origin_x;
    t_datay[i] = s_value * (vesselshape_x[i] - CoG4viewer[index][0]) +
                 c_value * (vesselshape_y[i] - CoG4viewer[index][1]) + origin_y;
  }
}

void Display2DDialog::readvesselsshape() {
  for (int i = 0; i != 3; ++i) vesselsshapedata[i] = vesselshapedata{0};

  const std::string vesselshape_first =
      "/home/skloe/Coding/CPP1X/USV/DPfloatover/QT/build/geometry/"
      "firstvessel.csv";
  const std::string vesselshape_second =
      "/home/skloe/Coding/CPP1X/USV/DPfloatover/QT/build/geometry/"
      "secondvessel.csv";
  const std::string vesselshape_third =
      "/home/skloe/Coding/CPP1X/USV/DPfloatover/QT/build/geometry/"
      "thirdvessel.csv";
  vesselshape myvesselshape(vesselshape_first);  // Open file
  myvesselshape.readvesselshape(vesselsshapedata[0]);
  myvesselshape.setfilename(vesselshape_second);
  myvesselshape.readvesselshape(vesselsshapedata[1]);
  myvesselshape.setfilename(vesselshape_third);
  myvesselshape.readvesselshape(vesselsshapedata[2]);

  for (int i = 0; i != 3; ++i) {
    planarmotion_x[i] = QVector<double>(vesselsshapedata[i].totalnum, 0);
    planarmotion_y[i] = QVector<double>(vesselsshapedata[i].totalnum, 0);
  }
}
// pop_front and push_back
void Display2DDialog::updatetrajectoryvector(double origin_x, double origin_y,
                                             QVector<double> &t_trajectory_x,
                                             QVector<double> &t_trajectory_y) {
  t_trajectory_x.pop_front();
  t_trajectory_x.push_back(origin_x);
  t_trajectory_y.pop_front();
  t_trajectory_y.push_back(origin_y);
}
// update the setpoint vector
void Display2DDialog::updatesetpointvector(double set_x, double set_y,
                                           QVector<double> &t_setpoint_x,
                                           QVector<double> &t_setpoint_y) {
  t_setpoint_x[0] = set_x;
  t_setpoint_y[0] = set_y;
}

// initialize UI for real-time planar motion
void Display2DDialog::initializePlanarMotion(QCustomPlot *customPlot) {
  //  initialize realtime viewer
  //  QBrush boxBrush(QColor(68, 68, 68, 255));
  //  ui->customPlot_2Dmotion->setBackground(boxBrush);  // setup color of
  //  background
  customPlot->setNotAntialiasedElements(QCP::aeAll);
  customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  //  QPen mypen;
  //  mypen.setWidth(0.1);
  //  mypen.setColor(QColor(30, 40, 255, 150));
  //  customPlot->graph()->setPen(mypen);

  // 2d display
  for (int c_index = 0; c_index != MAXCONNECTION; ++c_index) {
    customPlot->addGraph();
    // convert position and orientation of vessel to global coordinate
    convertvessel(0, 1, 180, planarmotion_y[c_index], planarmotion_x[c_index],
                  c_index);

    customPlot->graph(c_index)->setLineStyle(QCPGraph::lsNone);
    QCPScatterStyle myQCPScatterStyle(QCPScatterStyle::ssDisc,
                                      V_Qcolor[c_index], 2);
    customPlot->graph(c_index)->setScatterStyle(myQCPScatterStyle);
    customPlot->graph(c_index)->setData(planarmotion_x[c_index],
                                        planarmotion_y[c_index]);
    customPlot->graph(c_index)->rescaleAxes(true);
  }
  // trajectory display
  for (int c_index = 0; c_index != MAXCONNECTION; ++c_index) {
    customPlot->addGraph();

    customPlot->graph(c_index + MAXCONNECTION)->setLineStyle(QCPGraph::lsNone);
    QCPScatterStyle myQCPScatterStyle(QCPScatterStyle::ssDisc,
                                      V_Qcolor[c_index], 1);
    customPlot->graph(c_index + MAXCONNECTION)
        ->setScatterStyle(myQCPScatterStyle);
    customPlot->graph(c_index + MAXCONNECTION)
        ->setData(trajectory_x[c_index], trajectory_y[c_index]);
    customPlot->graph(c_index + MAXCONNECTION)->rescaleAxes(true);
  }
  // setpoint display
  for (int c_index = 0; c_index != MAXCONNECTION; ++c_index) {
    customPlot->addGraph();

    customPlot->graph(c_index + 2 * MAXCONNECTION)
        ->setLineStyle(QCPGraph::lsLine);
    // QCPScatterStyle myQCPScatterStyle(QCPScatterStyle::ssPlusCircle,
    //                                   V_Qcolor[c_index], 10);
    QCPScatterStyle myQCPScatterStyle(QCPScatterStyle::ssPlus,
                                      V_Qcolor[c_index], 10);
    customPlot->graph(c_index + 2 * MAXCONNECTION)
        ->setScatterStyle(myQCPScatterStyle);
    customPlot->graph(c_index + 2 * MAXCONNECTION)
        ->setData(setpoints_x[c_index], setpoints_y[c_index]);
    customPlot->graph(c_index + 2 * MAXCONNECTION)->rescaleAxes(true);
  }
  // setpoint circle display
  for (int c_index = 0; c_index != MAXCONNECTION; ++c_index) {
    // generate data for circle
    for (unsigned i = 0; i != arraylength_setpoint_circle; ++i) {
      double theta = i / (double)(arraylength_setpoint_circle - 1) * 2 * M_PI;
      setpoints_circle_larger[c_index][i] =
          QCPCurveData(i, radius_setpoint_larger * std::cos(theta),
                       radius_setpoint_larger * std::sin(theta));
      setpoints_circle_smaller[c_index][i] =
          QCPCurveData(i, radius_setpoint_smaller * std::cos(theta),
                       radius_setpoint_smaller * std::sin(theta));
    }

    setpoint_circle_curves_larger.push_back(
        new QCPCurve(customPlot->xAxis, customPlot->yAxis));
    setpoint_circle_curves_larger[c_index]->data()->set(
        setpoints_circle_larger[c_index], true);
    setpoint_circle_curves_larger[c_index]->setPen(QPen(V_Qcolor[c_index], 1));
    setpoint_circle_curves_smaller.push_back(
        new QCPCurve(customPlot->xAxis, customPlot->yAxis));
    setpoint_circle_curves_smaller[c_index]->data()->set(
        setpoints_circle_smaller[c_index], true);
    setpoint_circle_curves_smaller[c_index]->setPen(QPen(V_Qcolor[c_index], 1));
  }
  // setup x,y axis
  customPlot->xAxis->setRangeReversed(true);
  customPlot->yAxis->setRangeReversed(true);
  customPlot->xAxis->setRange(-30, 30);
  customPlot->yAxis->setRange(-30, 30);
  customPlot->yAxis->setScaleRatio(customPlot->xAxis, 1.0);  // axis equal
  //  customPlot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
}

// realtime cupture of the mouse position
bool Display2DDialog::eventFilter(QObject *target, QEvent *event) {
  if (target == ui->customPlot_2Dmotion && event->type() == QEvent::MouseMove) {
    QMouseEvent *_mouseEvent = static_cast<QMouseEvent *>(event);
    /*
         Here you have mouseEvent object so you have x,y coordinate of
         MouseEvent.
         Now if you want to convert x,y coordiante of mouse to plot coordinates
         you can easily use QCPAxis::pixelToCoord() method.

     */
    double x =
        ui->customPlot_2Dmotion->xAxis->pixelToCoord(_mouseEvent->pos().x());
    double y =
        ui->customPlot_2Dmotion->yAxis->pixelToCoord(_mouseEvent->pos().y());
    ui->label_mousepos->setText(QString("X = %1 , Y = %2").arg(y).arg(x));
  }
  return false;
}

void Display2DDialog::initializePlanarMotionData() {
  readvesselsshape();
  for (int i = 0; i != MAXCONNECTION; ++i) {
    trajectory_x[i] = QVector<double>(trajectorylength, 0);
    trajectory_y[i] = QVector<double>(trajectorylength, 0);
    setpoints_x[i] = QVector<double>(1, 0);
    setpoints_y[i] = QVector<double>(1, 0);

    setpoints_circle_larger[i] =
        QVector<QCPCurveData>(arraylength_setpoint_circle);
    setpoints_circle_smaller[i] =
        QVector<QCPCurveData>(arraylength_setpoint_circle);
  }
  CoG4viewer[0] = {2, 0};      // the first vessel
  CoG4viewer[1] = {2, 0};      // the second vessel
  CoG4viewer[2] = {2.937, 0};  // the third vessel
}

void Display2DDialog::initializeCircle(QCustomPlot *customPlot) {
  // generate data for circle
  const int arraylength_dgsparse = 100;
  QVector<QCPCurveData> dataSpiral_outer(arraylength_dgsparse);

  for (unsigned i = 0; i != arraylength_dgsparse; ++i) {
    double theta = i / (double)(arraylength_dgsparse - 1) * 2 * M_PI;
    dataSpiral_outer[i] = QCPCurveData(i, radius_heading * std::cos(theta),
                                       radius_heading * std::sin(theta));
  }
  // create curve objects
  QCPCurve *fermatSpiral_outer =
      new QCPCurve(customPlot->xAxis, customPlot->yAxis);
  fermatSpiral_outer->data()->set(dataSpiral_outer, true);
  fermatSpiral_outer->setPen(QPen(QColor(68, 68, 68, 255), 1));

  // generate data for text around circle
  const int step_angle_text = 10;  // step of angle and text
  const int num_angle_text = 36;   // thus, number of angles and text

  // setup text label in the circle
  QVector<QCPItemText *> dbgtext_sparse;
  for (int i = 0; i != num_angle_text; ++i) {
    double angle_t = i * step_angle_text * M_PI / 180;

    double text_posx = 1.05 * radius_heading * std::cos(angle_t);
    double text_posy = 1.05 * radius_heading * std::sin(angle_t);
    dbgtext_sparse.push_back(new QCPItemText(customPlot));
    dbgtext_sparse.back()->position->setCoords(text_posy, text_posx);

    if (i * step_angle_text > 180)
      dbgtext_sparse.back()->setText(
          QString::number(i * step_angle_text - 360));
    else
      dbgtext_sparse.back()->setText(QString::number(i * step_angle_text));

    dbgtext_sparse.back()->setFont(QFont("SansSerif", 2.5 * radius_heading));
    dbgtext_sparse.back()->setColor(Qt::black);
  }

  // initialize the arrow
  std::vector<double> t_angle = {90, 130, 90};
  for (int c_index = 0; c_index != MAXCONNECTION; ++c_index) {
    setheadingarrows.push_back(new QCPItemCurve(customPlot));
    setheadingarrows[c_index]->setPen(QPen(V_Qcolor[c_index], 0.5));
    setheadingarrows[c_index]->setHead(
        QCPLineEnding(QCPLineEnding::esSpikeArrow, 16.0, 20.0));
    updateheadingarrow(setheadingarrows[c_index], t_angle[c_index]);

    realtimeheadingarrows.push_back(new QCPItemCurve(customPlot));
    realtimeheadingarrows[c_index]->setPen(QPen(V_Qcolor[c_index], 0.5));
    realtimeheadingarrows[c_index]->setHead(
        QCPLineEnding(QCPLineEnding::esFlatArrow, 16.0, 20.0, true));
    updateheadingarrow(realtimeheadingarrows[c_index], t_angle[c_index]);
  }
}

void Display2DDialog::updateheadingarrow(QCPItemCurve *_arrow,
                                         double _orientation  // deg
                                         ) {
  _orientation *= M_PI / 180;
  _orientation = M_PI / 2 - _orientation;
  double cvalue = std::cos(_orientation);
  double svalue = std::sin(_orientation);
  _arrow->start->setCoords(0.98 * radius_heading * cvalue,
                           0.98 * radius_heading * svalue);
  _arrow->end->setCoords(radius_heading * cvalue, radius_heading * svalue);
  _arrow->startDir->setCoords(0.98 * cvalue, 0.98 * svalue);
  _arrow->endDir->setCoords(cvalue, svalue);
}

void Display2DDialog::updatesetpointcircle(double _setpointx, double _setpointy,
                                           int c_index) {
  // generate data for circle
  for (unsigned i = 0; i != arraylength_setpoint_circle; ++i) {
    double theta = i / (double)(arraylength_setpoint_circle - 1) * 2 * M_PI;
    double cvalue = std::cos(theta);
    double svalue = std::sin(theta);
    setpoints_circle_larger[c_index][i] =
        QCPCurveData(i, _setpointx + radius_setpoint_larger * cvalue,
                     _setpointy + radius_setpoint_larger * svalue);
    setpoints_circle_smaller[c_index][i] =
        QCPCurveData(i, _setpointx + radius_setpoint_smaller * cvalue,
                     _setpointy + radius_setpoint_smaller * svalue);
  }

  setpoint_circle_curves_larger[c_index]->data()->set(
      setpoints_circle_larger[c_index], true);
  setpoint_circle_curves_smaller[c_index]->data()->set(
      setpoints_circle_smaller[c_index], true);
}
