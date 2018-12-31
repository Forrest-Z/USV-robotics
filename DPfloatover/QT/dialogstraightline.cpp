#include "dialogstraightline.h"
#include "ui_dialogstraightline.h"

Dialogstraightline::Dialogstraightline(QWidget *parent)
    : QDialog(parent), ui(new Ui::Dialogstraightline), index_vessel(0) {
  ui->setupUi(this);
  initializeStraightlineDiag();
}

Dialogstraightline::~Dialogstraightline() { delete ui; }

void Dialogstraightline::initializeStraightlineDiag() {
  // the first straight line data
  double desired_initialx = 0;
  double desired_initialy = 0;
  double desired_initialtheta = 0;
  double delta_value = 0;
  double desired_velocity = 0;
  int indicator = 0;

  globalvar::_threadloop.getstraightlinedata_first(
      desired_initialx, desired_initialy, desired_initialtheta, delta_value,
      desired_velocity, indicator);

  QString valestring = QString::number(desired_initialx);
  ui->LE_setxI0->setText(valestring);
  valestring = QString::number(desired_initialy);
  ui->LE_setyI0->setText(valestring);
  valestring = QString::number(desired_velocity);
  ui->LE_setvelocityI->setText(valestring);

  valestring = QString::number(delta_value);
  ui->LE_deltaI->setText(valestring);
  valestring = QString::number(indicator);
  ui->LE_indicatorI->setText(valestring);
  valestring = QString::number(desired_initialtheta);
  ui->LE_setthetaI->setText(valestring);
  // the second straight line data
  globalvar::_threadloop.getstraightlinedata_second(
      desired_initialx, desired_initialy, desired_initialtheta, delta_value,
      desired_velocity, indicator);

  valestring = QString::number(desired_initialx);
  ui->LE_setxII0->setText(valestring);
  valestring = QString::number(desired_initialy);
  ui->LE_setyII0->setText(valestring);
  valestring = QString::number(desired_velocity);
  ui->LE_setvelocityII->setText(valestring);

  valestring = QString::number(delta_value);
  ui->LE_deltaII->setText(valestring);
  valestring = QString::number(indicator);
  ui->LE_indicatorII->setText(valestring);
  valestring = QString::number(desired_initialtheta);
  ui->LE_setthetaII->setText(valestring);

  // the third straight line data
  globalvar::_threadloop.getstraightlinedata_third(
      desired_initialx, desired_initialy, desired_initialtheta, delta_value,
      desired_velocity, indicator);

  valestring = QString::number(desired_initialx);
  ui->LE_setxIII0->setText(valestring);
  valestring = QString::number(desired_initialy);
  ui->LE_setyIII0->setText(valestring);
  valestring = QString::number(desired_velocity);
  ui->LE_setvelocityIII->setText(valestring);

  valestring = QString::number(delta_value);
  ui->LE_deltaIII->setText(valestring);
  valestring = QString::number(indicator);
  ui->LE_indicatorIII->setText(valestring);
  valestring = QString::number(desired_initialtheta);
  ui->LE_setthetaIII->setText(valestring);
}

void Dialogstraightline::on_RB_VI_clicked() {
  ui->LE_setxI0->setEnabled(true);
  ui->LE_setyI0->setEnabled(true);
  ui->LE_setvelocityI->setEnabled(true);
  ui->LE_deltaI->setEnabled(true);
  ui->LE_indicatorI->setEnabled(true);
  ui->LE_setthetaI->setEnabled(true);

  ui->LE_setxII0->setEnabled(false);
  ui->LE_setyII0->setEnabled(false);
  ui->LE_setvelocityII->setEnabled(false);
  ui->LE_deltaII->setEnabled(false);
  ui->LE_indicatorII->setEnabled(false);
  ui->LE_setthetaII->setEnabled(false);

  ui->LE_setxIII0->setEnabled(false);
  ui->LE_setyIII0->setEnabled(false);
  ui->LE_setvelocityIII->setEnabled(false);
  ui->LE_deltaIII->setEnabled(false);
  ui->LE_indicatorIII->setEnabled(false);
  ui->LE_setthetaIII->setEnabled(false);

  index_vessel = 0;
}

void Dialogstraightline::on_RB_VII_clicked() {
  ui->LE_setxI0->setEnabled(false);
  ui->LE_setyI0->setEnabled(false);
  ui->LE_setvelocityI->setEnabled(false);
  ui->LE_deltaI->setEnabled(false);
  ui->LE_indicatorI->setEnabled(false);
  ui->LE_setthetaI->setEnabled(false);

  ui->LE_setxII0->setEnabled(true);
  ui->LE_setyII0->setEnabled(true);
  ui->LE_setvelocityII->setEnabled(true);
  ui->LE_deltaII->setEnabled(true);
  ui->LE_indicatorII->setEnabled(true);
  ui->LE_setthetaII->setEnabled(true);

  ui->LE_setxIII0->setEnabled(false);
  ui->LE_setyIII0->setEnabled(false);
  ui->LE_setvelocityIII->setEnabled(false);
  ui->LE_deltaIII->setEnabled(false);
  ui->LE_indicatorIII->setEnabled(false);
  ui->LE_setthetaIII->setEnabled(false);

  index_vessel = 1;
}

void Dialogstraightline::on_RB_VIII_clicked() {
  ui->LE_setxI0->setEnabled(false);
  ui->LE_setyI0->setEnabled(false);
  ui->LE_setvelocityI->setEnabled(false);
  ui->LE_deltaI->setEnabled(false);
  ui->LE_indicatorI->setEnabled(false);
  ui->LE_setthetaI->setEnabled(false);

  ui->LE_setxII0->setEnabled(false);
  ui->LE_setyII0->setEnabled(false);
  ui->LE_setvelocityII->setEnabled(false);
  ui->LE_deltaII->setEnabled(false);
  ui->LE_indicatorII->setEnabled(false);
  ui->LE_setthetaII->setEnabled(false);

  ui->LE_setxIII0->setEnabled(true);
  ui->LE_setyIII0->setEnabled(true);
  ui->LE_setvelocityIII->setEnabled(true);
  ui->LE_deltaIII->setEnabled(true);
  ui->LE_indicatorIII->setEnabled(true);
  ui->LE_setthetaIII->setEnabled(true);

  index_vessel = 2;
}

void Dialogstraightline::on_setupsetpoint_clicked() {
  if (index_vessel == 0) {
    double desired_initialx = ui->LE_setxI0->text().toDouble();
    double desired_initialy = ui->LE_setyI0->text().toDouble();
    double desired_velocity = ui->LE_setvelocityI->text().toDouble();
    double delta_value = ui->LE_deltaI->text().toDouble();
    int indicator = ui->LE_indicatorI->text().toInt();
    double desired_initialtheta = ui->LE_setthetaI->text().toDouble();
    desired_initialtheta *= (M_PI / 180);

    //  update setpoints
    globalvar::_threadloop.updatestraightlinesetpoints_t(
        desired_initialx, desired_initialy, desired_initialtheta, delta_value,
        desired_velocity, indicator, index_vessel);
    // ui operation
    ui->getcurrentposition->setEnabled(false);
    ui->setupsetpoint->setEnabled(false);
    ui->resetsetpoint->setEnabled(true);

    ui->LE_setxI0->setEnabled(false);
    ui->LE_setyI0->setEnabled(false);
    ui->LE_setvelocityI->setEnabled(false);
    ui->LE_deltaI->setEnabled(false);
    ui->LE_indicatorI->setEnabled(false);
    ui->LE_setthetaI->setEnabled(false);

  } else if (index_vessel == 1) {
    double desired_initialx = ui->LE_setxII0->text().toDouble();
    double desired_initialy = ui->LE_setyII0->text().toDouble();
    double desired_velocity = ui->LE_setvelocityII->text().toDouble();
    double delta_value = ui->LE_deltaII->text().toDouble();
    int indicator = ui->LE_indicatorII->text().toInt();
    double desired_initialtheta = ui->LE_setthetaII->text().toDouble();
    desired_initialtheta *= (M_PI / 180);

    //  update setpoints
    globalvar::_threadloop.updatestraightlinesetpoints_t(
        desired_initialx, desired_initialy, desired_initialtheta, delta_value,
        desired_velocity, indicator, index_vessel);
    // ui operation
    ui->getcurrentposition->setEnabled(false);
    ui->setupsetpoint->setEnabled(false);
    ui->resetsetpoint->setEnabled(true);

    ui->LE_setxII0->setEnabled(false);
    ui->LE_setyII0->setEnabled(false);
    ui->LE_setvelocityII->setEnabled(false);
    ui->LE_deltaII->setEnabled(false);
    ui->LE_indicatorII->setEnabled(false);
    ui->LE_setthetaII->setEnabled(false);

  } else {
    double desired_initialx = ui->LE_setxIII0->text().toDouble();
    double desired_initialy = ui->LE_setyIII0->text().toDouble();
    double desired_velocity = ui->LE_setvelocityIII->text().toDouble();
    double delta_value = ui->LE_deltaIII->text().toDouble();
    int indicator = ui->LE_indicatorIII->text().toInt();
    double desired_initialtheta = ui->LE_setthetaIII->text().toDouble();
    desired_initialtheta *= (M_PI / 180);

    //  update setpoints
    globalvar::_threadloop.updatestraightlinesetpoints_t(
        desired_initialx, desired_initialy, desired_initialtheta, delta_value,
        desired_velocity, indicator, index_vessel);
    // ui operation
    ui->getcurrentposition->setEnabled(false);
    ui->setupsetpoint->setEnabled(false);
    ui->resetsetpoint->setEnabled(true);

    ui->LE_setxIII0->setEnabled(false);
    ui->LE_setyIII0->setEnabled(false);
    ui->LE_setvelocityIII->setEnabled(false);
    ui->LE_deltaIII->setEnabled(false);
    ui->LE_indicatorIII->setEnabled(false);
    ui->LE_setthetaIII->setEnabled(false);
  }
}

void Dialogstraightline::on_resetsetpoint_clicked() {
  // stop update setpoints
  globalvar::_threadloop.closeupdatesetpoints();
  // ui operation
  ui->getcurrentposition->setEnabled(true);
  ui->setupsetpoint->setEnabled(true);
  ui->resetsetpoint->setEnabled(false);
  if (index_vessel == 0) {
    ui->LE_setxI0->setEnabled(true);
    ui->LE_setyI0->setEnabled(true);
    ui->LE_setvelocityI->setEnabled(true);
    ui->LE_deltaI->setEnabled(true);
    ui->LE_indicatorI->setEnabled(true);
    ui->LE_setthetaI->setEnabled(true);
  } else if (index_vessel == 1) {
    ui->LE_setxII0->setEnabled(true);
    ui->LE_setyII0->setEnabled(true);
    ui->LE_setvelocityII->setEnabled(true);
    ui->LE_deltaII->setEnabled(true);
    ui->LE_indicatorII->setEnabled(true);
    ui->LE_setthetaII->setEnabled(true);
  } else {
    ui->LE_setxIII0->setEnabled(true);
    ui->LE_setyIII0->setEnabled(true);
    ui->LE_setvelocityIII->setEnabled(true);
    ui->LE_deltaIII->setEnabled(true);
    ui->LE_indicatorIII->setEnabled(true);
    ui->LE_setthetaIII->setEnabled(true);
  }
}

void Dialogstraightline::on_getcurrentposition_clicked() {
  if (index_vessel == 0) {
    Eigen::Vector3d _setpoints = globalvar::_threadloop.getSetpoints_first();
    QString valestring = QString::number(_setpoints(0));
    ui->LE_setxI0->setText(valestring);
    valestring = QString::number(_setpoints(1));
    ui->LE_setyI0->setText(valestring);
    valestring = QString::number(_setpoints(2) * 180 / M_PI);
    ui->LE_setthetaI->setText(valestring);
  } else if (index_vessel == 1) {
    Eigen::Vector3d _setpoints = globalvar::_threadloop.getSetpoints_second();
    QString valestring = QString::number(_setpoints(0));
    ui->LE_setxII0->setText(valestring);
    valestring = QString::number(_setpoints(1));
    ui->LE_setyII0->setText(valestring);
    valestring = QString::number(_setpoints(2) * 180 / M_PI);
    ui->LE_setthetaII->setText(valestring);
  } else {
    Eigen::Vector3d _setpoints = globalvar::_threadloop.getSetpoints_third();
    QString valestring = QString::number(_setpoints(0));
    ui->LE_setxIII0->setText(valestring);
    valestring = QString::number(_setpoints(1));
    ui->LE_setyIII0->setText(valestring);
    valestring = QString::number(_setpoints(2) * 180 / M_PI);
    ui->LE_setthetaIII->setText(valestring);
  }
}
