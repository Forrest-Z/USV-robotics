/*
***********************************************************************
* dialogfixedsetpoint.h:
* function for setup the fixed setpoint
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef DIALOGFIXEDSETPOINT_H
#define DIALOGFIXEDSETPOINT_H

#include <QDialog>
#include "constants.h"
#include "globalvar.h"
#include "realtimedata.h"

namespace Ui {
class Dialogfixedsetpoint;
}

class Dialogfixedsetpoint : public QDialog {
  Q_OBJECT

 public:
  explicit Dialogfixedsetpoint(QWidget *parent = 0);
  ~Dialogfixedsetpoint();

 private slots:
  void on_setupsetpointI_clicked();

  void on_resetsetpointI_clicked();

  void on_setupsetpointII_clicked();

  void on_resetsetpointII_clicked();

  void on_setupsetpointIII_clicked();

  void on_resetsetpointIII_clicked();

 private:
  Ui::Dialogfixedsetpoint *ui;
  void initializeDialogfixedsetpoint();
};

#endif  // DIALOGFIXEDSETPOINT_H
