#include "dialogbox.h"
#include "ui_dialogbox.h"

DialogBox::DialogBox(QWidget *parent) : QDialog(parent), ui(new Ui::DialogBox) {
  ui->setupUi(this);
  initializeDialogBox();
}

DialogBox::~DialogBox() { delete ui; }

void DialogBox::on_PB_setupI_clicked() {
  double startx = ui->LE_startxI->text().toDouble();
  double starty = ui->LE_startyI->text().toDouble();
  double deltax = ui->LE_deltaxI->text().toDouble();
  double deltay = ui->LE_deltayI->text().toDouble();
  double velocity = ui->LE_velocityI->text().toDouble();
  double theta = ui->LE_thetaI->text().toDouble();
  theta *= (M_PI / 180);
  //  update setpoints
  globalvar::_threadloop.updatebox_t(velocity, theta, startx, starty, deltax,
                                     deltay, 0);
  // ui operation
  ui->PB_setupI->setEnabled(false);
  ui->PB_setupII->setEnabled(false);
  ui->PB_resetI->setEnabled(true);
  ui->PB_resetII->setEnabled(false);
  ui->LE_startxI->setEnabled(false);
  ui->LE_startyI->setEnabled(false);
  ui->LE_deltaxI->setEnabled(false);
  ui->LE_deltayI->setEnabled(false);
  ui->LE_velocityI->setEnabled(false);
  ui->LE_thetaI->setEnabled(false);

  ui->LE_startxII->setEnabled(false);
  ui->LE_startyII->setEnabled(false);
  ui->LE_deltaxII->setEnabled(false);
  ui->LE_deltayII->setEnabled(false);
  ui->LE_velocityII->setEnabled(false);
  ui->LE_thetaII->setEnabled(false);
}

void DialogBox::on_PB_resetI_clicked() {
  // stop update setpoints
  globalvar::_threadloop.closeupdatesetpoints();
  // ui operation
  ui->PB_setupI->setEnabled(true);
  ui->PB_setupII->setEnabled(true);
  ui->PB_resetI->setEnabled(false);
  ui->PB_resetII->setEnabled(false);
  ui->LE_startxI->setEnabled(true);
  ui->LE_startyI->setEnabled(true);
  ui->LE_deltaxI->setEnabled(true);
  ui->LE_deltayI->setEnabled(true);
  ui->LE_velocityI->setEnabled(true);
  ui->LE_thetaI->setEnabled(true);

  ui->LE_startxII->setEnabled(true);
  ui->LE_startyII->setEnabled(true);
  ui->LE_deltaxII->setEnabled(true);
  ui->LE_deltayII->setEnabled(true);
  ui->LE_velocityII->setEnabled(true);
  ui->LE_thetaII->setEnabled(true);
}

void DialogBox::on_PB_setupII_clicked() {
  double startx = ui->LE_startxII->text().toDouble();
  double starty = ui->LE_startyII->text().toDouble();
  double deltax = ui->LE_deltaxII->text().toDouble();
  double deltay = ui->LE_deltayII->text().toDouble();
  double velocity = ui->LE_velocityII->text().toDouble();
  double theta = ui->LE_thetaII->text().toDouble();
  theta *= (M_PI / 180);
  //  update setpoints
  globalvar::_threadloop.updatebox_t(velocity, theta, startx, starty, deltax,
                                     deltay, 1);
  // ui operation
  ui->PB_setupI->setEnabled(false);
  ui->PB_setupII->setEnabled(false);
  ui->PB_resetI->setEnabled(false);
  ui->PB_resetII->setEnabled(true);
  ui->LE_startxI->setEnabled(false);
  ui->LE_startyI->setEnabled(false);
  ui->LE_deltaxI->setEnabled(false);
  ui->LE_deltayI->setEnabled(false);
  ui->LE_velocityI->setEnabled(false);
  ui->LE_thetaI->setEnabled(false);

  ui->LE_startxII->setEnabled(false);
  ui->LE_startyII->setEnabled(false);
  ui->LE_deltaxII->setEnabled(false);
  ui->LE_deltayII->setEnabled(false);
  ui->LE_velocityII->setEnabled(false);
  ui->LE_thetaII->setEnabled(false);
}

void DialogBox::on_PB_resetII_clicked() {
  // stop update setpoints
  globalvar::_threadloop.closeupdatesetpoints();
  // ui operation
  ui->PB_setupI->setEnabled(true);
  ui->PB_setupII->setEnabled(true);
  ui->PB_resetI->setEnabled(false);
  ui->PB_resetII->setEnabled(false);
  ui->LE_startxI->setEnabled(true);
  ui->LE_startyI->setEnabled(true);
  ui->LE_deltaxI->setEnabled(true);
  ui->LE_deltayI->setEnabled(true);
  ui->LE_velocityI->setEnabled(true);
  ui->LE_thetaI->setEnabled(true);

  ui->LE_startxII->setEnabled(true);
  ui->LE_startyII->setEnabled(true);
  ui->LE_deltaxII->setEnabled(true);
  ui->LE_deltayII->setEnabled(true);
  ui->LE_velocityII->setEnabled(true);
  ui->LE_thetaII->setEnabled(true);
}

void DialogBox::initializeDialogBox() {
  double desired_velocity = 0;
  double desired_theta = 0;
  double desired_initialx = 0;
  double desired_initialy = 0;
  double deltax = 0;
  double deltay = 0;

  globalvar::_threadloop.getboxdata_first(desired_velocity, desired_theta,
                                          desired_initialx, desired_initialy,
                                          deltax, deltay);

  QString valestring = QString::number(desired_initialx);
  ui->LE_startxI->setText(valestring);
  valestring = QString::number(desired_initialy);
  ui->LE_startyI->setText(valestring);
  valestring = QString::number(deltax);
  ui->LE_deltaxI->setText(valestring);
  valestring = QString::number(deltay);
  ui->LE_deltayI->setText(valestring);

  valestring = QString::number(desired_velocity);
  ui->LE_velocityI->setText(valestring);
  valestring = QString::number(desired_theta);
  ui->LE_thetaI->setText(valestring);

  globalvar::_threadloop.getboxdata_second(desired_velocity, desired_theta,
                                           desired_initialx, desired_initialy,
                                           deltax, deltay);

  valestring = QString::number(desired_initialx);
  ui->LE_startxII->setText(valestring);
  valestring = QString::number(desired_initialy);
  ui->LE_startyII->setText(valestring);
  valestring = QString::number(deltax);
  ui->LE_deltaxII->setText(valestring);
  valestring = QString::number(deltay);
  ui->LE_deltayII->setText(valestring);

  valestring = QString::number(desired_velocity);
  ui->LE_velocityII->setText(valestring);
  valestring = QString::number(desired_theta);
  ui->LE_thetaII->setText(valestring);
}
