/*
***********************************************************************
* mainwindow.h: mainwindow of DP controller
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <fstream>
#include "dialogabout.h"
#include "dialogbox.h"
#include "dialogcooperation.h"
#include "dialogfixedsetpoint.h"
#include "dialogrotation.h"
#include "dialogsetparameter.h"
#include "dialogstraightline.h"
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
  void updatestatus();

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

  void on_actionGamepad_G_triggered();

  void on_actionSetpoints_triggered();

  void on_actionStraightLine_triggered();

  void on_actionRotation_triggered();

  void on_actionBox_triggered();

  void on_actionCooperation_triggered();

  void on_actionLicensing_triggered();

 private:
  Ui::MainWindow *ui;
  Display2DDialog *myDisplay2DDialog;
  ThrusterDiag *myThrusterDiag;
  Dialogsetparameter *myDialogsetparameter;
  DialogBox *myDialogBox;
  Dialogfixedsetpoint *myDialogfixedsetpoint;
  DialogRotation *myDialogRotation;
  Dialogstraightline *myDialogstraightline;
  DialogCooperation *myDialogCooperation;
  Dialogabout *myDialogabout;
  QStringListModel *_model;
  Qt::WindowFlags flags = nullptr;

  QString logfilepath;
  QFile inputfile;

  QPixmap onPixRed;
  QPixmap onPixGreen;

  void GetCurrentPath();
  void initializeLoglist();
  void updatelogAndStatus();
  void initializestatus();
};

#endif  // MAINWINDOW_H
