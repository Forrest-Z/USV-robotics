#include "dialogsetparameter.h"
#include <QLineEdit>
#include "ui_dialogsetparameter.h"

Dialogsetparameter::Dialogsetparameter(QWidget *parent)
    : QDialog(parent), ui(new Ui::Dialogsetparameter) {
  ui->setupUi(this);
  initializePIDData();
}

Dialogsetparameter::~Dialogsetparameter() { delete ui; }

void Dialogsetparameter::initializePIDData() {
  // P matrix of I vessel
  Eigen::Vector3d pmatrix_first = globalvar::_threadloop.getpmatrix_first();
  QString valestring = QString::number(pmatrix_first(0));
  ui->LE_PXI->setText(valestring);
  valestring = QString::number(pmatrix_first(1));
  ui->LE_PYI->setText(valestring);
  valestring = QString::number(pmatrix_first(2));
  ui->LE_PTHETAI->setText(valestring);
  // I matrix of I vessel
  Eigen::Vector3d Imatrix_first = globalvar::_threadloop.getImatrix_first();
  valestring = QString::number(Imatrix_first(0));
  ui->LE_IXI->setText(valestring);
  valestring = QString::number(Imatrix_first(1));
  ui->LE_IYI->setText(valestring);
  valestring = QString::number(Imatrix_first(2));
  ui->LE_ITHETAI->setText(valestring);
  // D matrix of I vessel
  Eigen::Vector3d Dmatrix_first = globalvar::_threadloop.getDmatrix_first();
  valestring = QString::number(Dmatrix_first(0));
  ui->LE_DXI->setText(valestring);
  valestring = QString::number(Dmatrix_first(1));
  ui->LE_DYI->setText(valestring);
  valestring = QString::number(Dmatrix_first(2));
  ui->LE_DTHETAI->setText(valestring);

  // P matrix of II vessel
  Eigen::Vector3d pmatrix_second = globalvar::_threadloop.getpmatrix_second();
  valestring = QString::number(pmatrix_second(0));
  ui->LE_PXII->setText(valestring);
  valestring = QString::number(pmatrix_second(1));
  ui->LE_PYII->setText(valestring);
  valestring = QString::number(pmatrix_second(2));
  ui->LE_PTHETAII->setText(valestring);
  // I matrix of II vessel
  Eigen::Vector3d Imatrix_second = globalvar::_threadloop.getImatrix_second();
  valestring = QString::number(Imatrix_second(0));
  ui->LE_IXII->setText(valestring);
  valestring = QString::number(Imatrix_second(1));
  ui->LE_IYII->setText(valestring);
  valestring = QString::number(Imatrix_second(2));
  ui->LE_ITHETAII->setText(valestring);
  // D matrix of II vessel
  Eigen::Vector3d Dmatrix_second = globalvar::_threadloop.getDmatrix_second();
  valestring = QString::number(Dmatrix_second(0));
  ui->LE_DXII->setText(valestring);
  valestring = QString::number(Dmatrix_second(1));
  ui->LE_DYII->setText(valestring);
  valestring = QString::number(Dmatrix_second(2));
  ui->LE_DTHETAII->setText(valestring);

  // P matrix of III vessel
  Eigen::Vector3d pmatrix_third = globalvar::_threadloop.getpmatrix_third();
  valestring = QString::number(pmatrix_third(0));
  ui->LE_PXIII->setText(valestring);
  valestring = QString::number(pmatrix_third(1));
  ui->LE_PYIII->setText(valestring);
  valestring = QString::number(pmatrix_third(2));
  ui->LE_PTHETAIII->setText(valestring);
  // I matrix of III vessel
  Eigen::Vector3d Imatrix_third = globalvar::_threadloop.getImatrix_third();
  valestring = QString::number(Imatrix_third(0));
  ui->LE_IXIII->setText(valestring);
  valestring = QString::number(Imatrix_third(1));
  ui->LE_IYIII->setText(valestring);
  valestring = QString::number(Imatrix_third(2));
  ui->LE_ITHETAIII->setText(valestring);
  // D matrix of III vessel
  Eigen::Vector3d Dmatrix_third = globalvar::_threadloop.getDmatrix_third();
  valestring = QString::number(Dmatrix_third(0));
  ui->LE_DXIII->setText(valestring);
  valestring = QString::number(Dmatrix_third(1));
  ui->LE_DYIII->setText(valestring);
  valestring = QString::number(Dmatrix_third(2));
  ui->LE_DTHETAIII->setText(valestring);
}

void Dialogsetparameter::on_PIDsetupI_clicked() {
  double PX = ui->LE_PXI->text().toDouble();
  double PY = ui->LE_PYI->text().toDouble();
  double PTheta = ui->LE_PTHETAI->text().toDouble();
  double IX = ui->LE_IXI->text().toDouble();
  double IY = ui->LE_IYI->text().toDouble();
  double ITheta = ui->LE_ITHETAI->text().toDouble();
  double DX = ui->LE_DXI->text().toDouble();
  double DY = ui->LE_DYI->text().toDouble();
  double DTheta = ui->LE_DTHETAI->text().toDouble();
  globalvar::_threadloop.setPID_first(PX, PY, PTheta, IX, IY, ITheta, DX, DY,
                                      DTheta);
}

void Dialogsetparameter::on_GB_II_clicked() {
  double PX = ui->LE_PXII->text().toDouble();
  double PY = ui->LE_PYII->text().toDouble();
  double PTheta = ui->LE_PTHETAII->text().toDouble();
  double IX = ui->LE_IXII->text().toDouble();
  double IY = ui->LE_IYII->text().toDouble();
  double ITheta = ui->LE_ITHETAII->text().toDouble();
  double DX = ui->LE_DXII->text().toDouble();
  double DY = ui->LE_DYII->text().toDouble();
  double DTheta = ui->LE_DTHETAII->text().toDouble();
  globalvar::_threadloop.setPID_second(PX, PY, PTheta, IX, IY, ITheta, DX, DY,
                                       DTheta);
}

void Dialogsetparameter::on_GB_III_clicked() {
  double PX = ui->LE_PXIII->text().toDouble();
  double PY = ui->LE_PYIII->text().toDouble();
  double PTheta = ui->LE_PTHETAIII->text().toDouble();
  double IX = ui->LE_IXIII->text().toDouble();
  double IY = ui->LE_IYIII->text().toDouble();
  double ITheta = ui->LE_ITHETAIII->text().toDouble();
  double DX = ui->LE_DXIII->text().toDouble();
  double DY = ui->LE_DYIII->text().toDouble();
  double DTheta = ui->LE_DTHETAIII->text().toDouble();
  globalvar::_threadloop.setPID_third(PX, PY, PTheta, IX, IY, ITheta, DX, DY,
                                      DTheta);
}
