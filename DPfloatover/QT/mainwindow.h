#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <fstream>
#include "display2ddialog.h"
#include "globalvar.h"
#include "thrusterdiag.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

 private slots:
  void on_action2D_triggered();

  void on_actionThruster_P_triggered();

  void on_PB_start_clicked();

  void on_PB_connection_clicked();

  void on_PB_suspend_clicked();

  void readfilebyline();

  void on_PB_enablePLC_clicked();

  void on_RB_headholdI_clicked();

  void on_RB_PIDI_clicked();

  void on_RB_manualI_clicked();

  void on_RB_MPCI_clicked();

  void on_RB_headholdII_clicked();

  void on_RB_PIDII_clicked();

  void on_RB_manualII_clicked();

  void on_RB_MPCII_clicked();

  void on_RB_headholdIII_clicked();

  void on_RB_PIDIII_clicked();

  void on_RB_manualIII_clicked();

  void on_RB_MPCIII_clicked();

  void on_RB_fixedpointI_clicked();

  void on_RB_straightlineI_clicked();

  void on_RB_fixedpointII_clicked();

  void on_RB_straightlineII_clicked();

  void on_RB_fixedpointIII_clicked();

  void on_RB_straightlineIII_clicked();

 private:
  Ui::MainWindow *ui;
  Display2DDialog *myDisplay2DDialog;
  ThrusterDiag *myThrusterDiag;
  QStringListModel *_model;
  Qt::WindowFlags flags = nullptr;

  void GetCurrentPath();
  void initializeLoglist();
  void updatelog();
};

#endif  // MAINWINDOW_H
