#include "dialog6dof.h"
#include "ui_dialog6dof.h"

Dialog6dof::Dialog6dof(QWidget *parent)
    : QDialog(parent),
      ui(new Ui::Dialog6dof),
      motion6Dof_xaxis_data(arraylength_6DoF, 0),
      motion6Dof_yaxis_data(arraylength_6DoF, 0) {
  ui->setupUi(this);
  initializeAllUI();
  setupVesselRealtimeData();
}

Dialog6dof::~Dialog6dof() { delete ui; }

void Dialog6dof::initializeAllUI() {
  // setup all colors used
  QPalette pal_background = palette();
  QPalette pal_2D = palette();
  pal_background.setColor(QPalette::Background, QColor(68, 68, 68, 255));
  pal_2D.setColor(QPalette::Background, Qt::white);

  // set the whole dialog
  this->setWindowState(Qt::WindowMaximized);
  this->setWindowFlags(Qt::Window);
  this->setAutoFillBackground(true);
  this->setPalette(pal_background);
  this->setStyleSheet("QLabel {color : white; }");  // set color for all Qlabels
  // setGeometry(500, 250, 1500, 950);

  // initialize the time series of 6DOF motion and planar motion
  initialize6DOFmotion(ui->customPlot_6DOF);
}

void Dialog6dof::initialize6DOFmotion(QCustomPlot *customPlot) {
  customPlot->setGeometry(10, 10, MAXCONNECTION * 450, size_6dofdisplay);
  customPlot->plotLayout()->clear();

  QVector<QString> str_6DOFmotion;
  str_6DOFmotion << "Surge (m)"
                 << "Sway (m)"
                 << "Heave (m)"
                 << "Roll (deg)"
                 << "Pitch (deg)"
                 << "Yaw (deg)";
  // prepare data
  for (unsigned i = 0; i != arraylength_6DoF; ++i)
    motion6Dof_xaxis_data[i] = i / (double)(arraylength_6DoF - 1) * 34 - 17;

  for (unsigned c_index = 0; c_index != MAXCONNECTION; ++c_index)
    for (unsigned j = 0; j != 6; ++j) {
      unsigned temp = j + 6 * c_index;
      motion_clients[temp] = motion6Dof_yaxis_data;
      dofmotionplot.push_back(new QCPAxisRect(customPlot));
      customPlot->plotLayout()->addElement(j, c_index, dofmotionplot[temp]);
      dofmotionplot[temp]->axis(QCPAxis::atLeft)->setLabel(str_6DOFmotion[j]);
      customPlot->addGraph(dofmotionplot[temp]->axis(QCPAxis::atBottom),
                           dofmotionplot[temp]->axis(QCPAxis::atLeft));
      customPlot->graph(temp)->setData(motion6Dof_xaxis_data,
                                       motion6Dof_xaxis_data);
      customPlot->graph(temp)->valueAxis()->setRange(-0.5, 0.5);
    }
}

void Dialog6dof::setupVesselRealtimeData() {
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(motion6DOFdataSlot()));
  // Interval 0 means to refresh as fast as possible
  dataTimer.start(VIEWERREFRESH);
}

// real-time 6DOF motion data
void Dialog6dof::motion6DOFdataSlot() {
  Vector6d position6DoF_first =
      globalvar::_threadloop.getrealtime6dmotion_first();
  Vector6d position6DoF_second =
      globalvar::_threadloop.getrealtime6dmotion_second();
  Vector6d position6DoF_third =
      globalvar::_threadloop.getrealtime6dmotion_third();

  // update data to make phase move:
  if (MAXCONNECTION > 0) {
    for (unsigned j = 0; j != 6; ++j) {
      auto searchdata = motion_clients.find(j);
      searchdata->second.pop_front();
      searchdata->second.push_back(position6DoF_first(j));

      ui->customPlot_6DOF->graph(j)->setData(motion6Dof_xaxis_data,
                                             searchdata->second);
      ui->customPlot_6DOF->graph(j)->rescaleValueAxis(true);
    }
  }
  if (MAXCONNECTION > 1) {
    for (unsigned j = 0; j != 6; ++j) {
      unsigned item4eachmotion = j + 6;
      auto searchdata = motion_clients.find(item4eachmotion);
      searchdata->second.pop_front();
      searchdata->second.push_back(position6DoF_second(j));

      ui->customPlot_6DOF->graph(item4eachmotion)
          ->setData(motion6Dof_xaxis_data, searchdata->second);
      ui->customPlot_6DOF->graph(item4eachmotion)->rescaleValueAxis(true);
    }
  }
  if (MAXCONNECTION > 2) {
    for (unsigned j = 0; j != 6; ++j) {
      unsigned item4eachmotion = j + 12;
      auto searchdata = motion_clients.find(item4eachmotion);
      searchdata->second.pop_front();
      searchdata->second.push_back(position6DoF_third(j));

      ui->customPlot_6DOF->graph(item4eachmotion)
          ->setData(motion6Dof_xaxis_data, searchdata->second);
      ui->customPlot_6DOF->graph(item4eachmotion)->rescaleValueAxis(true);
    }
  }

  ui->customPlot_6DOF->replot();
}
